"""크기를 줄이면 XL430 으로 편하게 걸을 수 있는지: 다리 길이·몸 크기·프린트 부품 질량을 함께 줄여 시험."""
import importlib

import dog_model
import torque_study as ts
from gait_sweep import trot_motion

BASE = dict(HIP_X=0.093, HIP_Y=0.039, HIP_LINK=0.055, THIGH=0.1075, SHANK=0.130)
for scale in (1.0, 0.9, 0.85, 0.8):
    importlib.reload(dog_model)
    for k, v in BASE.items():
        setattr(dog_model, k, v * scale)
    importlib.reload(ts)
    ts.build, ts.pose_targets = dog_model.build, dog_model.pose_targets
    printed = scale ** 3                                     # 프린트 부품 질량은 부피에 비례
    mass = dog_model.MassBudget(motor=0.065, body_frame=0.35 * printed, shoulder_part=0.035 * printed,
                                thigh_part=0.05 * printed, shank_part=0.03 * printed)
    h = -0.145 * scale
    out = []
    for T, step, lift in ((0.45, 0.05 * scale, 0.03 * scale), (0.6, 0.04 * scale, 0.025 * scale)):
        res, hh, r, p, _ = ts.run(mass, 1.5, trot_motion(T, step, lift, h))
        ok = hh > 0.12 * scale and abs(r) < 20 and abs(p) < 20
        out.append(f"{'성공' if ok else '실패'} 무릎 RMS {res['knee'][1] / 1.5 * 100:3.0f}% 엉덩이 {res['hip_y'][1] / 1.5 * 100:3.0f}%")
    stand, *_ = ts.run(mass, 1.5, ts.motions()[2])
    print(f"크기 {scale * 100:3.0f}%  총 {mass.total():.2f} kg  서기 무릎 RMS {stand['knee'][1] / 1.5 * 100:3.0f}%  | 보통 걸음: {out[0]} | 느린 걸음: {out[1]}")
