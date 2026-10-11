"""CAD → 시뮬레이터 자산: 부품 메시(STL, 가볍게), 관절 위치, 모터 방향, 부품 질량을 software/dxldog/assets 에 저장한다.
시뮬레이터·Raspberry Pi 는 이 자산만 읽으므로 build123d(CAD 도구)가 없어도 된다.
실행: ../.venv/Scripts/python export_sim_assets.py [mixed|sts|xl|xm]   → software/dxldog/assets/<구성>/
"""
import json
import sys
from pathlib import Path

import numpy as np
from build123d import Plane, export_stl, mirror

from build_all import FILL, PLA, VARIANTS
from dxldog_cad import HIP_X, body_plates, foot, joints, shank, shoulder, thigh

ASSETS = Path(__file__).resolve().parents[1] / "software" / "dxldog" / "assets"


def frame_of(mp: Plane):
    v3 = lambda v: [float(v.X), float(v.Y), float(v.Z)]
    return {"origin": [round(c / 1000, 6) for c in v3(mp.origin)], "x": v3(mp.x_dir), "y": v3(mp.y_dir), "z": v3(mp.z_dir)}


def main(variant="mixed"):
    b = VARIANTS[variant]
    OUT = ASSETS / variant
    OUT.mkdir(parents=True, exist_ok=True)
    hx, hy, kn, ft, mp_hx, mp_hy, mp_kn = joints(b)
    parts = {"shoulder_L": shoulder(b), "thigh_L": thigh(b), "shank_L": shank(b), "foot": foot(b)}
    for k in ("shoulder", "thigh", "shank"):
        parts[f"{k}_R"] = mirror(parts[f"{k}_L"], about=Plane.XZ)
    plates = body_plates(b)
    parts["body_top"], parts["body_bottom"] = plates["body_top"], plates["body_bottom"]
    masses = {}
    for name, s in parts.items():
        export_stl(s, str(OUT / f"{name}.stl"), tolerance=0.25, angular_tolerance=0.35)
        masses[name] = round(s.volume * PLA * FILL / 1000, 4)          # kg
    masses["foot"] = round(parts["foot"].volume * 1.2e-3 * 1.0 / 1000, 4)   # TPU, 꽉 채움
    info = {
        "variant": variant,
        "motors": {"hip_x": b.hipx.name, "hip_y": b.hipy.name, "knee": b.knee.name},
        "motor_mass": {"hip_x": b.hipx.mass_g / 1000, "hip_y": b.hipy.mass_g / 1000, "knee": b.knee.mass_g / 1000},
        "hip_x_axis": [v / 1000 for v in hx], "hip_y_axis": [v / 1000 for v in hy], "knee_axis": [v / 1000 for v in kn],
        "foot_center": [v / 1000 for v in ft], "rear_shift": -2 * HIP_X / 1000,
        "motor_frames_FL": {"hip_x": frame_of(mp_hx), "hip_y": frame_of(mp_hy), "knee": frame_of(mp_kn)},
        "motor_box": [b.hipy.W / 1000, b.hipy.L / 1000, b.hipy.D / 1000], "motor_box_center_y": (b.hipy.FAR - b.hipy.NEAR) / 2 / 1000,
        "part_mass": masses,
        "note": "좌표: FL 다리, 설계 자세(다리 똑바로 아래), m. 메시 꼭짓점은 mm 단위 조립 좌표.",
    }
    (OUT / "frames.json").write_text(json.dumps(info, indent=1, ensure_ascii=False), encoding="utf-8")
    print("저장:", OUT, {k: v for k, v in masses.items()})


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "mixed")
