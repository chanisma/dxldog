"""CAD → 시뮬레이터 자산: 부품 메시(STL, 가볍게), 관절 위치, 모터 방향, 부품 질량을 software/dxldog/assets 에 저장한다.
시뮬레이터·Raspberry Pi 는 이 자산만 읽으므로 build123d(CAD 도구)가 없어도 된다.
실행: ../.venv/Scripts/python export_sim_assets.py [mixed|sts|xl|xm] [v1|v2]   → software/dxldog/assets/<구성>/
  v2 (기본): Spot 모양 (dxldog_v2.py). 덮개 메시 cover(노랑)·cover_dark(얼굴판·레일)를 더한다.  v1: 처음 설계
"""
import json
import sys
from pathlib import Path

import numpy as np
from build123d import Compound, Plane, export_stl, mirror

import dxldog_cad
import dxldog_v2
from build_all import FILL, PLA, SHELL_FILL, VARIANTS
from dxldog_cad import HIP_X, joints

ASSETS = Path(__file__).resolve().parents[1] / "software" / "dxldog" / "assets"


def frame_of(mp: Plane):
    v3 = lambda v: [float(v.X), float(v.Y), float(v.Z)]
    return {"origin": [round(c / 1000, 6) for c in v3(mp.origin)], "x": v3(mp.x_dir), "y": v3(mp.y_dir), "z": v3(mp.z_dir)}


def main(variant="mixed", style="v2"):
    D = dxldog_v2 if style == "v2" else dxldog_cad
    b = dxldog_v2.VARIANTS[variant] if style == "v2" else VARIANTS[variant]
    OUT = ASSETS / variant
    OUT.mkdir(parents=True, exist_ok=True)
    hx, hy, kn, ft, mp_hx, mp_hy, mp_kn = joints(b)
    parts = {"shoulder_L": D.shoulder(b), "thigh_L": D.thigh(b), "shank_L": D.shank(b), "foot": D.foot(b)}
    for k in ("shoulder", "thigh", "shank"):
        parts[f"{k}_R"] = mirror(parts[f"{k}_L"], about=Plane.XZ)
    plates = D.body_plates(b)
    parts["body_top"], parts["body_bottom"] = plates["body_top"], plates["body_bottom"]
    shells = set()
    if style == "v2":                                                  # 덮개: 색이 다른 두 메시로 (노랑, 검정)
        cov = dxldog_v2.covers(b)
        parts["cover"] = Compound([cov["cover_mid"], cov["cover_end_F"], cov["cover_end_R"]])
        parts["cover_dark"] = Compound([cov["face_F"], cov["face_R"], cov["rail_L"], cov["rail_R"]])
        shells = {"cover", "cover_dark"}
    else:
        for old in ("cover", "cover_dark"):                            # v1 로 되돌릴 때 v2 덮개가 남지 않게
            (OUT / f"{old}.stl").unlink(missing_ok=True)
    masses = {}
    for name, s in parts.items():
        export_stl(s, str(OUT / f"{name}.stl"), tolerance=0.25, angular_tolerance=0.35)
        masses[name] = round(s.volume * PLA * (SHELL_FILL if name in shells else FILL) / 1000, 4)          # kg
    masses["foot"] = round(parts["foot"].volume * 1.2e-3 * 1.0 / 1000, 4)   # TPU, 꽉 채움
    info = {
        "variant": variant, "style": style,
        "motors": {"hip_x": b.hipx.name, "hip_y": b.hipy.name, "knee": b.knee.name},
        "motor_mass": {"hip_x": b.hipx.mass_g / 1000, "hip_y": b.hipy.mass_g / 1000, "knee": b.knee.mass_g / 1000},
        "hip_x_axis": [v / 1000 for v in hx], "hip_y_axis": [v / 1000 for v in hy], "knee_axis": [v / 1000 for v in kn],
        "foot_center": [v / 1000 for v in ft], "rear_shift": -2 * HIP_X / 1000,
        "motor_frames_FL": {"hip_x": frame_of(mp_hx), "hip_y": frame_of(mp_hy), "knee": frame_of(mp_kn)},
        "motor_box": [b.hipy.W / 1000, b.hipy.L / 1000, b.hipy.D / 1000], "motor_box_center_y": (b.hipy.FAR - b.hipy.NEAR) / 2 / 1000,
        "part_mass": masses,
        "body_box": ({"center": [0, 0, (dxldog_v2.ZB + dxldog_v2.ZT) / 2000], "half": [dxldog_v2.XN / 1000, dxldog_v2.YW / 1000, (dxldog_v2.ZT - dxldog_v2.ZB) / 2000]}
                     if style == "v2" else {"center": [0, 0, 0], "half": [0.111, 0.048, dxldog_v2.z_plate(b) / 1000]}),   # 넘어질 때 바닥과 닿는 상자
        "note": "좌표: FL 다리, 설계 자세(다리 똑바로 아래), m. 메시 꼭짓점은 mm 단위 조립 좌표.",
    }
    (OUT / "frames.json").write_text(json.dumps(info, indent=1, ensure_ascii=False), encoding="utf-8")
    print("저장:", OUT, {k: v for k, v in masses.items()})


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "mixed", sys.argv[2] if len(sys.argv) > 2 else "v2")
