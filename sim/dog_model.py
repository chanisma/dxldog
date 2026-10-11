"""DYNAMIXEL SpotMicro(가칭 DXL Dog) MuJoCo 모델 — 모터 선정을 위한 토크 시뮬레이션용.

치수는 SpotMicro (mike4192/spotMicro 설정값, CC BY 4.0 설계) 를 따른다.
  엉덩이 간격 앞뒤 186 mm · 좌우 78 mm, 엉덩이 링크(옆 오프셋) 55 mm, 허벅지 107.5 mm, 정강이 130 mm
관절: 다리마다 hip_x(옆으로 벌리기, x축) · hip_y(앞뒤 흔들기, y축) · knee(무릎, y축) = 12개.
무릎 모터는 무릎 관절에 직접 붙는다(직결). 질량은 MassBudget 으로 바꿀 수 있다.
"""
from dataclasses import dataclass, field

import mujoco
import numpy as np

LEGS = ["FL", "FR", "RL", "RR"]
HIP_X, HIP_Y = 0.093, 0.039          # 몸통 중심 → hip_x 관절 (m)
HIP_LINK = 0.055                     # hip_x 관절 → hip_y 관절 옆 오프셋
THIGH, SHANK = 0.1075, 0.130
FOOT_R = 0.012


@dataclass
class MassBudget:
    motor: float = 0.065             # 모터 1개 (XL/XC430 65 g, XM430 82 g)
    body_frame: float = 0.35         # 몸통 프린트 부품
    battery: float = 0.15            # 3S 1800 mAh
    electronics: float = 0.15        # 컨트롤러(OpenRB/U2D2+허브), 배선 일부
    wiring: float = 0.10             # 케이블·나사
    shoulder_part: float = 0.035
    thigh_part: float = 0.050
    shank_part: float = 0.030
    foot: float = 0.010
    extra: dict = field(default_factory=dict)

    def body_total(self):
        return self.body_frame + self.battery + self.electronics + self.wiring + 4 * self.motor

    def total(self):
        return self.body_total() + 4 * (self.shoulder_part + self.motor + self.thigh_part + self.motor
                                         + self.shank_part + self.foot)


def build(mass: MassBudget, max_torque=50.0, kp=40.0, kv=1.0):
    """max_torque 를 아주 크게 주면 '필요한 토크'를 재고, 모터 정지 토크를 주면 '그 모터로 되는지'를 본다."""
    spec = mujoco.MjSpec()
    spec.compiler.degree = False                       # 관절 범위를 라디안으로 (기본값은 도!)
    spec.option.timestep = 0.001
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    wb = spec.worldbody
    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1], friction=[1.0, 0.005, 0.0001])
    torso = wb.add_body(name="torso", pos=[0, 0, 0.20])
    torso.add_freejoint(name="root")
    torso.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.12, 0.055, 0.035], mass=mass.body_total(),
                   rgba=[0.9, 0.7, 0.1, 1])
    torso.add_site(name="imu")
    for leg in LEGS:
        sx = 1 if leg[0] == "F" else -1
        sy = 1 if leg[1] == "L" else -1
        sh = torso.add_body(name=f"{leg}_shoulder", pos=[sx * HIP_X, sy * HIP_Y, 0])
        sh.add_joint(name=f"{leg}_hip_x", axis=[1, 0, 0], range=[-0.6, 0.6], damping=0.05, armature=0.01)
        sh.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.02, 0.0275, 0.017], pos=[0, sy * HIP_LINK / 2, 0],
                    mass=mass.shoulder_part + mass.motor, contype=0, conaffinity=0)
        th = sh.add_body(name=f"{leg}_thigh", pos=[0, sy * HIP_LINK, 0])
        th.add_joint(name=f"{leg}_hip_y", axis=[0, 1, 0], range=[-2.6, 2.6], damping=0.05, armature=0.01)
        th.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[0, 0, 0, 0, 0, -THIGH], size=[0.012, 0, 0],
                    mass=mass.thigh_part, contype=0, conaffinity=0)
        kn = th.add_body(name=f"{leg}_shank", pos=[0, 0, -THIGH])
        kn.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.017, 0.014, 0.023], mass=mass.motor,   # 무릎 모터
                    contype=0, conaffinity=0)
        kn.add_joint(name=f"{leg}_knee", axis=[0, 1, 0], range=[-2.8, -0.2], damping=0.05, armature=0.01)
        kn.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[0, 0, 0, 0, 0, -SHANK], size=[0.009, 0, 0],
                    mass=mass.shank_part, contype=0, conaffinity=0)
        kn.add_geom(name=f"{leg}_foot", type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=[0, 0, -SHANK], size=[FOOT_R, 0, 0],
                    mass=mass.foot, friction=[1.0, 0.005, 0.0001])
    for leg in LEGS:
        for j in ("hip_x", "hip_y", "knee"):
            a = spec.add_actuator(name=f"{leg}_{j}", target=f"{leg}_{j}", trntype=mujoco.mjtTrn.mjTRN_JOINT)
            a.set_to_position(kp=kp, kv=kv)
            a.forcelimited = True
            a.forcerange = [-max_torque, max_torque]
            a.ctrllimited = True                       # 지정하지 않으면 [0, 0] 으로 잘려 목표가 모두 0 이 된다
            a.ctrlrange = [-2.8, 2.8]
    return spec.compile()


# ---------- 다리 기구학 (옆에서 본 2관절, 무릎은 뒤로 접힘) ----------
def leg_ik(x, z):
    """엉덩이(hip_y) 관절 기준 발 위치 (x 앞+, z 위+) → (hip_y, knee) rad."""
    c = (x * x + z * z - THIGH ** 2 - SHANK ** 2) / (2 * THIGH * SHANK)
    if abs(c) > 1:
        raise ValueError("다리가 닿지 않는 위치")
    q2 = np.arccos(c)
    q1 = np.arctan2(x, -z) - np.arctan2(SHANK * np.sin(q2), THIGH + SHANK * np.cos(q2))
    # MuJoCo y축 회전: +q 이면 아래쪽 링크가 앞(+x)이 아니라 뒤로 → 부호 반전
    return -q1, q2


def pose_targets(feet):
    """{다리: (x, z)} → 액추에이터 12개 목표 (hip_x 는 0)."""
    q = []
    for leg in LEGS:
        hy, kn = leg_ik(*feet[leg])
        q += [0.0, hy, -kn]
    return np.array(q)
