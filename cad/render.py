"""조립 미리보기: 네 다리를 자세에 맞게 놓고 MuJoCo 로 그려 PNG 로 저장한다.
실행: ../.venv/Scripts/python render.py [xl|xm|mixed|sts] [stand|rest|zero]
"""
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np
from build123d import Plane, export_stl, mirror
from PIL import Image

from build_all import VARIANTS
from dxldog_cad import HIP_X, body_plates, leg_parts, pose_leg

POSES = {   # (옆 벌리기°, 허벅지°, 무릎 굽힘°)  — 허벅지: 아래에서 앞쪽 +, 무릎: 앞쪽으로 굽힘 +
    "stand": (0, -67.1, 96.6),       # 몸 높이 15.5 cm, 발은 엉덩이보다 3.5 cm 뒤 (software config.POSES 와 같음)
    "rest": (0, -69.0, 120.0),       # 쉬는 자세 (몸 높이 약 12 cm)
    "zero": (0, 0, 0),               # 설계 자세 (다리 똑바로)
}
COLORS = {"print": [0.96, 0.62, 0.16, 1], "body": [0.93, 0.93, 0.95, 1], "motor": [0.16, 0.17, 0.19, 1], "foot": [0.1, 0.1, 0.1, 1]}


def robot_solids(variant, pose):
    b = VARIANTS[variant]
    leg = pose_leg(leg_parts(b), b, *POSES[pose])
    out = {}
    for name, s in leg.items():                     # FL, RL(앞 다리 부품을 뒤로 옮김), FR/RR(좌우 거울)
        out[f"FL_{name}"] = s
        out[f"RL_{name}"] = s.translate((-2 * HIP_X, 0, 0))
        m = mirror(s, about=Plane.XZ)
        out[f"FR_{name}"] = m
        out[f"RR_{name}"] = m.translate((-2 * HIP_X, 0, 0))
    for k, s in body_plates(b).items():
        if k != "body_pillar":
            out[k] = s
    return out


def color_of(name):
    if "m_" in name:
        return COLORS["motor"]
    if "foot" in name:
        return COLORS["foot"]
    if "body" in name:
        return COLORS["body"]
    return COLORS["print"]


def render(variant="xl", pose="stand", out_png=None, size=(960, 640)):
    solids = robot_solids(variant, pose)
    tmp = Path(tempfile.mkdtemp())
    spec = mujoco.MjSpec()
    spec.compiler.degree = False
    lowest = min(s.bounding_box().min.Z for s in solids.values())
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1], pos=[0, 0, lowest / 1000],
                            rgba=[0.85, 0.87, 0.9, 1])
    for i, (name, s) in enumerate(solids.items()):
        f = tmp / f"{i}.stl"
        export_stl(s, str(f), tolerance=0.3, angular_tolerance=0.3)
        spec.add_mesh(name=f"m{i}", file=str(f), scale=[1e-3] * 3)
        spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"m{i}", rgba=color_of(name),
                                contype=0, conaffinity=0)
    spec.worldbody.add_light(pos=[0.4, -0.6, 1.2], dir=[-0.3, 0.5, -1])
    spec.visual.global_.offwidth, spec.visual.global_.offheight = size
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, size[1], size[0])
    views = []
    for az, el, dist in ((135, -20, 0.75), (90, -5, 0.7), (180, -60, 0.8)):
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0, 0, (lowest / 1000) * 0.45]
        cam.azimuth, cam.elevation, cam.distance = az, el, dist
        r.update_scene(d, cam)
        views.append(r.render().copy())
    img = np.concatenate([views[0], np.concatenate([views[1][:, ::2], views[2][:, ::2]], axis=1)], axis=0)
    out_png = out_png or str(Path(__file__).parent / "out" / f"preview_{variant}_{pose}.png")
    Image.fromarray(img).save(out_png)
    print("저장:", out_png, "  바닥에서 몸 중심까지", round(-lowest, 1), "mm")
    return out_png


if __name__ == "__main__":
    render(*(sys.argv[1:3] if len(sys.argv) > 2 else (sys.argv[1] if len(sys.argv) > 1 else "xl", "stand")))
