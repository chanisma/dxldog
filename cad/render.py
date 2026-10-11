"""조립 미리보기: 네 다리를 자세에 맞게 놓고 MuJoCo 로 그려 PNG 로 저장한다.
실행: ../.venv/Scripts/python render.py [xl|xm|mixed|sts] [stand|rest|zero] [v1|v2]
  v1 = 처음 설계 (dxldog_cad.py), v2 = Spot 을 참고해 다듬은 설계 (dxldog_v2.py)
"""
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np
from build123d import Plane, export_stl, mirror
from PIL import Image

import dxldog_cad
import dxldog_v2
from build_all import VARIANTS
from dxldog_cad import HIP_X

POSES = {   # (옆 벌리기°, 허벅지°, 무릎 굽힘°)  — 허벅지: 아래에서 앞쪽 +, 무릎: 앞쪽으로 굽힘 +
    "stand": (0, -67.1, 96.6),       # 몸 높이 15.5 cm, 발은 엉덩이보다 3.5 cm 뒤 (software config.POSES 와 같음)
    "rest": (0, -69.0, 120.0),       # 쉬는 자세 (몸 높이 약 12 cm)
    "zero": (0, 0, 0),               # 설계 자세 (다리 똑바로)
}
COLORS = {"print": [0.96, 0.62, 0.16, 1], "body": [0.93, 0.93, 0.95, 1], "motor": [0.16, 0.17, 0.19, 1], "foot": [0.1, 0.1, 0.1, 1]}
# v2: Spot 색 (노랑 덮개·허벅지, 검은 정강이·얼굴판, 짙은 회색 엉덩이 통)
COLORS_V2 = {"cover": [0.97, 0.75, 0.08, 1], "thigh": [0.97, 0.75, 0.08, 1], "face": [0.06, 0.06, 0.07, 1],
             "rail": [0.08, 0.08, 0.09, 1], "shoulder": [0.24, 0.25, 0.27, 1], "shank": [0.11, 0.11, 0.12, 1],
             "foot": [0.04, 0.04, 0.04, 1], "motor": [0.13, 0.13, 0.14, 1], "body": [0.55, 0.56, 0.58, 1]}


def design(style):
    return dxldog_v2 if style == "v2" else dxldog_cad


def build_of(variant, style):
    return dxldog_v2.VARIANTS[variant] if style == "v2" else VARIANTS[variant]


def robot_solids(variant, pose, style="v1", angles=None):
    D, b = design(style), build_of(variant, style)
    leg = D.pose_leg(D.leg_parts(b), b, *(angles or POSES[pose]))
    out = {}
    for name, s in leg.items():                     # FL, RL(앞 다리 부품을 뒤로 옮김), FR/RR(좌우 거울)
        out[f"FL_{name}"] = s
        out[f"RL_{name}"] = s.translate((-2 * HIP_X, 0, 0))
        m = mirror(s, about=Plane.XZ)
        out[f"FR_{name}"] = m
        out[f"RR_{name}"] = m.translate((-2 * HIP_X, 0, 0))
    for k, s in D.body_plates(b).items():
        if k != "body_pillar":
            out[k] = s
    if style == "v2":
        out.update(dxldog_v2.covers(b))
    return out


def color_of(name, style="v1"):
    if style == "v2":
        if "m_" in name:
            return COLORS_V2["motor"]
        for k in ("cover", "face", "rail", "shoulder", "thigh", "shank", "foot"):
            if k in name:
                return COLORS_V2[k]
        return COLORS_V2["body"]
    if "m_" in name:
        return COLORS["motor"]
    if "foot" in name:
        return COLORS["foot"]
    if "body" in name:
        return COLORS["body"]
    return COLORS["print"]


def render(variant="xl", pose="stand", style="v1", out_png=None, size=(960, 640)):
    solids = robot_solids(variant, pose, style)
    tmp = Path(tempfile.mkdtemp())
    spec = mujoco.MjSpec()
    spec.compiler.degree = False
    lowest = min(s.bounding_box().min.Z for s in solids.values())
    spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[0.98, 0.98, 0.99], rgb2=[0.78, 0.81, 0.86], width=256, height=256)
    spec.add_texture(name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                     rgb1=[0.86, 0.87, 0.89], rgb2=[0.82, 0.83, 0.86], width=512, height=512)
    floor = spec.add_material(name="floor", texrepeat=[16, 16], reflectance=0.0)
    floor.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1], pos=[0, 0, lowest / 1000], material="floor")
    for i, (name, s) in enumerate(solids.items()):
        f = tmp / f"{i}.stl"
        export_stl(s, str(f), tolerance=0.15 if style == "v2" else 0.3, angular_tolerance=0.2 if style == "v2" else 0.3)
        spec.add_mesh(name=f"m{i}", file=str(f), scale=[1e-3] * 3)
        mat = spec.add_material(name=f"c{i}", rgba=color_of(name, style), specular=0.35, shininess=0.5)
        spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"m{i}", material=mat.name, contype=0, conaffinity=0)
    spec.worldbody.add_light(pos=[0.5, -0.7, 1.4], dir=[-0.35, 0.5, -1], diffuse=[0.75, 0.75, 0.75], castshadow=True)
    spec.worldbody.add_light(pos=[-0.8, 0.6, 0.9], dir=[0.6, -0.4, -0.6], diffuse=[0.3, 0.3, 0.32], castshadow=False)
    spec.visual.headlight.ambient = [0.32, 0.32, 0.34]
    spec.visual.headlight.diffuse = [0.25, 0.25, 0.25]
    spec.visual.quality.shadowsize = 4096
    spec.visual.global_.offwidth, spec.visual.global_.offheight = size
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    big = mujoco.Renderer(m, size[1], size[0])
    small = mujoco.Renderer(m, size[1], size[0] // 2)
    views = []
    for rr, (az, el, dist) in ((big, (135, -20, 0.68)), (small, (90, -5, 0.85)), (small, (180, -60, 0.9))):
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0, 0, (lowest / 1000) * 0.45]
        cam.azimuth, cam.elevation, cam.distance = az, el, dist
        rr.update_scene(d, cam)
        views.append(rr.render().copy())
    img = np.concatenate([views[0], np.concatenate([views[1], views[2]], axis=1)], axis=0)
    tag = "" if style == "v1" else "v2_"
    out_png = out_png or str(Path(__file__).parent / "out" / f"preview_{tag}{variant}_{pose}.png")
    Image.fromarray(img).save(out_png)
    print("저장:", out_png, "  바닥에서 몸 중심까지", round(-lowest, 1), "mm")
    return out_png


if __name__ == "__main__":
    args = sys.argv[1:]
    render(args[0] if args else "xl", args[1] if len(args) > 1 else "stand", args[2] if len(args) > 2 else "v1")
