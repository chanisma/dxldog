"""모터 선정용 토크 시뮬레이션.

1) 필요한 토크 측정: 모터 힘 제한을 크게 두고 5가지 동작을 시켜 관절 종류별 토크(99% 최댓값, RMS)를 잰다.
2) 후보 모터 검증: 힘 제한을 각 모터의 정지 토크(12 V)로 두고 같은 동작을 시켜 실제로 해내는지 본다.
실행: ../.venv/Scripts/python torque_study.py
"""
import math
import sys

import mujoco
import numpy as np

from dog_model import LEGS, MassBudget, build, pose_targets

CANDIDATES = {   # 이름: (정지 토크 Nm @12V, 모터 질량 kg, 가격 USD, 12개 가격)
    "XL430-W250": (1.5, 0.065, 27.50),
    "XC430-W240": (1.9, 0.065, 137.89),
    "XM430-W210": (3.0, 0.082, 310.39),
    "XM430-W350": (4.1, 0.082, 310.39),
}
JOINT_TYPES = ["hip_x", "hip_y", "knee"]
DT = 0.001


def feet_all(x, z):
    return {leg: (x, z) for leg in LEGS}


def trot_feet(t, T=0.36, step=0.06, lift=0.04, height=-0.145, x0=0.0):
    phase = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}
    out = {}
    for leg, ph in phase.items():
        p = (t / T + ph) % 1.0
        if p < 0.5:
            x, z = step / 2 - step * (p / 0.5), height
        else:
            s = (p - 0.5) / 0.5
            x, z = -step / 2 + step * s, height + lift * math.sin(math.pi * s)
        out[leg] = (x + x0, z)
    return out


def motions():
    """(이름, 시간 길이, t → 발 목표) 목록. 모든 동작은 엎드린 자세(발 높이 −8 cm)에서 시작한다."""
    lie, stand = -0.08, -0.155

    def lerp(a, b, s):
        return a + (b - a) * min(max(s, 0), 1)
    return [
        ("엎드림→서기 1.5 s", 3.0, lambda t: feet_all(0.0, lerp(lie, stand, (t - 0.5) / 1.5))),
        ("빠르게 일어서기 0.4 s", 2.0, lambda t: feet_all(0.0, lerp(lie, stand, (t - 0.5) / 0.4))),
        ("서서 버티기", 3.0, lambda t: feet_all(0.0, lerp(lie, stand, t / 1.0))),
        ("트롯 걷기", 6.0, lambda t: feet_all(0.0, lerp(lie, -0.145, t / 1.0)) if t < 1.5 else trot_feet(t - 1.5)),
        ("한 다리 들기", 5.0, lambda t: one_leg(t, lie, stand)),
    ]


def one_leg(t, lie, stand):
    if t < 1.0:
        return feet_all(0.0, lie + (stand - lie) * t)
    shift = min((t - 1.0) / 1.0, 1.0) * 0.045            # 발을 앞으로 → 몸이 뒤로
    feet = feet_all(shift, stand)
    if t > 2.5:
        lift = min((t - 2.5) / 0.5, 1.0) * 0.06
        feet["FL"] = (shift, stand + lift)
    return feet


def run(mass, max_torque, motion):
    name, dur, feet_fn = motion
    m = build(mass, max_torque=max_torque)
    d = mujoco.MjData(m)
    q0 = pose_targets(feet_fn(0.0))
    d.qpos[7:] = q0
    d.qpos[2] = 0.08 + 0.012
    d.ctrl[:] = q0
    mujoco.mj_forward(m, d)
    taus, heights = [], []
    n = int(dur / DT)
    for k in range(n):
        t = k * DT
        try:
            d.ctrl[:] = pose_targets(feet_fn(t))
        except ValueError:
            pass
        mujoco.mj_step(m, d)
        if t > 0.3:
            taus.append(np.abs(d.actuator_force.copy()))
            heights.append(d.qpos[2])
    taus = np.array(taus)
    w, x, y, z = d.qpos[3:7]
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x)))))
    res = {}
    for i, jt in enumerate(JOINT_TYPES):
        cols = taus[:, i::3]
        res[jt] = (float(np.percentile(cols, 99)), float(np.sqrt((cols ** 2).mean())))
    return res, float(d.qpos[2]), roll, pitch, float(np.max(heights))


def demand(mass):
    print(f"\n[필요 토크] 총 질량 {mass.total():.2f} kg  (단위 Nm, 99% 최댓값 / RMS)")
    print(f"{'동작':22s}" + "".join(f"{jt:>16s}" for jt in JOINT_TYPES))
    worst = {jt: 0.0 for jt in JOINT_TYPES}
    for mo in motions():
        res, h, r, p, _ = run(mass, 50.0, mo)
        print(f"{mo[0]:22s}" + "".join(f"{res[jt][0]:9.2f} / {res[jt][1]:4.2f}" for jt in JOINT_TYPES)
              + f"   끝 높이 {h * 100:4.1f} cm")
        for jt in JOINT_TYPES:
            worst[jt] = max(worst[jt], res[jt][0])
    return worst


def verify(name):
    stall, motor_mass, price = CANDIDATES[name]
    mass = MassBudget(motor=motor_mass)
    print(f"\n[검증] {name}: 정지 토크 {stall} Nm, 총 질량 {mass.total():.2f} kg, 모터 12개 ${price * 12:,.0f}")
    for mo in motions():
        res, h, r, p, hmax = run(mass, stall, mo)
        sat = max(res[jt][0] for jt in JOINT_TYPES) / stall * 100
        ok = h > 0.12 and abs(r) < 20 and abs(p) < 20
        print(f"  {mo[0]:22s} {'성공' if ok else '실패'}  끝 높이 {h * 100:4.1f} cm, 기울기 {r:+5.1f}/{p:+5.1f}°, "
              f"최대 사용률 {sat:5.0f}%  (무릎 RMS {res['knee'][1]:.2f} Nm = 정지 토크의 {res['knee'][1] / stall * 100:.0f}%)")


if __name__ == "__main__":
    worst = demand(MassBudget(motor=0.065))
    demand(MassBudget(motor=0.082))
    for c in (sys.argv[1:] or CANDIDATES):
        verify(c)
