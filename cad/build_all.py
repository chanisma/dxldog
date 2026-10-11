"""모든 부품을 만들고 STL/STEP 로 내보낸 뒤, 부피·예상 질량을 출력한다.
실행: ../.venv/Scripts/python build_all.py [xl|xm|mixed|sts] [v1|v2]
  v1 (기본): 처음 설계 → out/<구성>/      v2: Spot 을 참고해 다듬은 설계 (dxldog_v2.py) → out/v2/<구성>/
  xl    : 12개 모두 XL430 (또는 XC430)
  xm    : 12개 모두 XM430
  mixed : 옆으로 벌리기 XL430, 앞뒤·무릎 XM430
  sts   : 12개 모두 Feetech STS3215 (LeRobot SO-101 모터)
"""
import sys
import time
from pathlib import Path

from build123d import Plane, export_step, export_stl, mirror

import x430 as X
from sts3215 import STS3215
from dxldog_cad import Build

VARIANTS = {"xl": Build(), "xm": Build(X.XM430, X.XM430, X.XM430), "mixed": Build(X.XL430, X.XM430, X.XM430),
            "sts": Build(STS3215, STS3215, STS3215)}
PLA = 1.24e-3          # g/mm³
FILL = 0.55            # 벽+채움 30 % 정도일 때 실제 질량 비율 (어림)
SHELL_FILL = 0.9       # 덮개처럼 얇은(2 mm) 부품은 거의 꽉 차게 인쇄된다


def build(variant="xl", style="v1"):
    if style == "v2":
        import dxldog_v2 as D
        b = D.VARIANTS[variant]
        out = Path(__file__).parent / "out" / "v2" / variant
    else:
        import dxldog_cad as D
        b = VARIANTS[variant]
        out = Path(__file__).parent / "out" / variant
    out.mkdir(parents=True, exist_ok=True)
    parts = {}
    t = time.perf_counter()
    parts.update(D.body_plates(b))
    if style == "v2":
        cov = D.covers(b)
        parts["cover_mid"], parts["cover_end"], parts["face_plate"] = cov["cover_mid"], cov["cover_end_F"], cov["face_F"]
        parts["rail"] = cov["rail_L"].translate((0, -D.RAIL_Y, 0))
    parts["shoulder_FL"] = D.shoulder(b)
    parts["thigh_FL"] = D.thigh(b)
    parts["shank_FL"] = D.shank(b)
    parts["foot"] = D.foot(b)
    parts.update(D.fit_gauges(b))
    # 거울 대칭: 왼쪽 ↔ 오른쪽 (XZ 평면)
    for k in ("shoulder", "thigh", "shank"):
        parts[f"{k}_FR"] = mirror(parts[f"{k}_FL"], about=Plane.XZ)
    total = 0.0
    print(f"[{variant} {style}] 만든 시간 {time.perf_counter() - t:.1f}s")
    count = {"body_pillar": 4, "foot": 4, "shoulder_FL": 2, "shoulder_FR": 2, "thigh_FL": 2, "thigh_FR": 2,
             "shank_FL": 2, "shank_FR": 2, "body_top": 1, "body_bottom": 1, "cover_end": 2, "face_plate": 2, "rail": 2}
    for name, p in parts.items():
        export_stl(p, str(out / f"{name}.stl"))
        export_step(p, str(out / f"{name}.step"))
        g = p.volume * PLA * (SHELL_FILL if name.startswith(("cover", "face", "rail")) else FILL)
        n = count.get(name, 1)
        if not name.startswith("gauge"):
            total += g * n
        bb = p.bounding_box()
        print(f"  {name:18s} ×{n}  부피 {p.volume / 1000:7.1f} cm³  약 {g:5.1f} g  크기 {bb.size.X:5.1f}×{bb.size.Y:5.1f}×{bb.size.Z:5.1f} mm  유효={p.is_valid}")
    print(f"  프린트 부품 합계 약 {total:.0f} g (뒷다리는 앞다리 부품을 그대로 사용 — 거울 대칭 불필요, 앞뒤 대칭 조립)")
    return parts, b


if __name__ == "__main__":
    build(sys.argv[1] if len(sys.argv) > 1 else "xl", sys.argv[2] if len(sys.argv) > 2 else "v1")
