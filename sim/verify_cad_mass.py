"""CAD 로 계산한 실제 프린트 부품 질량으로 모터 후보를 다시 검증한다."""
import torque_study as ts
from dog_model import MassBudget
from gait_sweep import trot_motion

CAD = dict(body_frame=0.092, shoulder_part=0.042, thigh_part=0.037, shank_part=0.027, foot=0.003)   # build_all.py 결과 (g → kg)
for name, (stall, mm, price) in ts.CANDIDATES.items():
    mass = MassBudget(motor=mm, **CAD)
    rows = []
    for label, (T, step, lift) in (("보통", (0.45, 0.05, 0.03)), ("빠름", (0.36, 0.06, 0.04))):
        res, h, r, p, _ = ts.run(mass, stall, trot_motion(T, step, lift, -0.145))
        ok = h > 0.12 and abs(r) < 20 and abs(p) < 20
        rows.append(f"{label} 걸음 {'성공' if ok else '실패'} (무릎 RMS {res['knee'][1] / stall * 100:3.0f}%, 엉덩이 {res['hip_y'][1] / stall * 100:3.0f}%)")
    st, *_ = ts.run(mass, stall, ts.motions()[2])
    print(f"{name:11s} 총 {mass.total():.2f} kg | 서기 무릎 RMS {st['knee'][1] / stall * 100:3.0f}% | " + " | ".join(rows))
