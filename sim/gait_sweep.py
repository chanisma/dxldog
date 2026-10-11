"""걸음새를 부드럽게 하면 싼 모터로도 걸을 수 있는지: 걸음새 × 모터 조합 시험."""
import mujoco
import numpy as np

import torque_study as ts
from dog_model import MassBudget

GAITS = {   # 이름: (주기 s, 보폭 m, 발 드는 높이 m, 딛는 높이 m)
    "빠름 (0.36 s, 보폭 6, 들기 4 cm)": (0.36, 0.06, 0.04, -0.145),
    "보통 (0.45 s, 보폭 5, 들기 3 cm)": (0.45, 0.05, 0.03, -0.145),
    "느림 (0.6 s, 보폭 4, 들기 2.5 cm)": (0.60, 0.04, 0.025, -0.150),
}
MOTORS = {"XL430-W250": (1.5, 0.065), "XC430-W240": (1.9, 0.065), "XM430-W350": (4.1, 0.082)}


def trot_motion(T, step, lift, height):
    lie = -0.08
    return ("트롯", 8.0, lambda t: ts.feet_all(0.0, lie + (height - lie) * min(t, 1.0)) if t < 1.5
            else ts.trot_feet(t - 1.5, T, step, lift, height))


if __name__ == "__main__":
    print(f"{'걸음새':34s}" + "".join(f"{m:>28s}" for m in MOTORS))
    for g, params in GAITS.items():
        row = []
        for name, (stall, mm) in MOTORS.items():
            mass = MassBudget(motor=mm)
            res, h, r, p, _ = ts.run(mass, stall, trot_motion(*params))
            ok = h > 0.12 and abs(r) < 20 and abs(p) < 20
            knee_rms = res["knee"][1] / stall * 100
            hip_rms = res["hip_y"][1] / stall * 100
            row.append(f"{'성공' if ok else '실패'} RMS 무릎{knee_rms:3.0f}% 엉덩이{hip_rms:3.0f}%")
        print(f"{g:34s}" + "".join(f"{c:>28s}" for c in row))
