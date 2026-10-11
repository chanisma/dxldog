"""DXL Dog — SpotMicro 를 DYNAMIXEL X430 용으로 다시 설계한 3D 프린트 부품 (build123d, 치수를 숫자로 바꿀 수 있음).

기준 치수(SpotMicro, mm): 엉덩이 간격 앞뒤 186 · 좌우 78, 엉덩이 링크 55, 허벅지 107.5, 정강이 130.
좌표: x 앞, y 왼쪽, z 위. 원점 = 몸통 중심. 아래 값은 앞왼쪽(FL) 다리, '설계 자세'(허벅지·정강이가 똑바로 아래) 기준.
나머지 다리는 거울 대칭(좌우·앞뒤)으로 만든다. 모터를 거울 대칭하면 '혼이 반대쪽인 같은 모터'가 되므로 실제로 조립 가능하다.

관절마다 같은 원리:
  하우징 = 앞 부품이 모터 몸통의 폭 면(±Xm, 구멍 4개씩)과 먼 끝 면을 잡는다 (모터 두께 띠 |Zm| ≤ 17 안)
  포크   = 다음 부품의 두 팔이 혼(+Zm)과 아이들러(−Zm)를 바깥에서 잡고, 축에서 24~34 mm 떨어진 '다리'로 이어진다
  → 다리 쪽 부품은 모터 몸통이 향한 방향(먼 끝 ±45°)만 피하면 되므로 관절이 넓게 돈다.
"""
import math
from dataclasses import dataclass

import numpy as np
from build123d import (Align, Box, Cylinder, Location, Plane, Polygon, Pos, Solid, Compound, extrude)

import x430 as X

MIN3 = (Align.CENTER, Align.CENTER, Align.MIN)

# ---------- 기본 치수 ----------
HIP_X, HIP_Y, HIP_LINK = 93.0, 39.0, 55.0
THIGH, SHANK = 107.5, 130.0
T_ARM = 4.0            # 포크 팔 두께
T_PLATE = 3.0          # 하우징 옆판 두께
ARM_R = 13.0           # 포크 팔의 축 둘레 원판 반지름
STRIP = 11.0           # 포크 팔 띠 반폭
BR0, BR1 = 24.0, 34.0  # 포크 '다리'가 놓이는 축으로부터의 거리 범위
FOOT_R = 12.0


@dataclass
class Build:
    hipx: X.X430 = X.XL430     # 옆으로 벌리기 모터
    hipy: X.X430 = X.XL430     # 앞뒤 흔들기 모터
    knee: X.X430 = X.XL430     # 무릎 모터
    hipy_center_deg: float = -37.5   # 허벅지 동작 범위 가운데 (아래에서 앞쪽 +): 필요 범위 −95° ~ +20°
    knee_center_deg: float = 70.0    # 무릎 굽힘 범위 가운데: 필요 범위 15° ~ 125°


def d2(a_deg):
    """x-z 평면의 방향 (아래에서 앞쪽으로 a 도)."""
    a = math.radians(a_deg)
    return np.array([math.sin(a), 0.0, -math.cos(a)])


# ---------- 평면·기본 도형 ----------
def plane_xz(y):      # 2D (x, z), y 방향으로 밀어내기: origin y=y1 에서 −y 쪽으로
    return Plane(origin=(0, y, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))


def plane_yz(x):      # 2D (y, z), +x 쪽으로 밀어내기
    return Plane(origin=(x, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))


def slab_y(pts_xz, y0, y1):
    pl = plane_xz(y1)
    return extrude(pl * Polygon(*pts_xz, align=None), amount=y1 - y0, dir=pl.z_dir)   # 방향을 명시 (점 순서에 따라 뒤집히지 않게)


def slab_x(pts_yz, x0, x1):
    pl = plane_yz(x0)
    return extrude(pl * Polygon(*pts_yz, align=None), amount=x1 - x0, dir=pl.z_dir)


def rod(pl, u, v, r, d0, d1):
    return pl * Pos(u, v, d0) * Cylinder(r, d1 - d0, align=MIN3)


def rect(u0, u1, v0, v1):
    return [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]


def motor_plane(axis_pt, zm, ym):
    zm, ym = np.asarray(zm, float), np.asarray(ym, float)
    xm = np.cross(ym, zm)
    return Plane(origin=tuple(axis_pt), x_dir=tuple(xm), z_dir=tuple(zm))


def motor_solid(m: X.X430, mp: Plane):
    case = mp * Pos(0, (X.FAR - X.NEAR) / 2, 0) * Box(X.W, X.L, X.D)
    horn = mp * Pos(0, 0, X.HALF_D) * Cylinder(m.horn_d / 2, m.horn_proud, align=MIN3)
    idler = mp * Pos(0, 0, -X.HALF_D - m.idler_proud) * Cylinder(m.horn_d / 2, m.idler_proud, align=MIN3)
    return case + horn + idler


def motor_keepout(m: X.X430, mp: Plane, clear=X.CLEAR):
    """프린트 부품이 들어오면 안 되는 모터 몸통 공간 (틈 포함, 혼·아이들러 쪽 원판 포함)."""
    case = mp * Pos(0, (X.FAR - X.NEAR) / 2, 0) * Box(X.W + 2 * clear, X.L + 2 * clear, X.D + 2 * clear)
    hub = mp * Pos(0, 0, -X.HALF_D - m.idler_proud - 0.2) * Cylinder(m.horn_d / 2 + 0.6, X.D + m.horn_proud + m.idler_proud + 0.4, align=MIN3)
    return case + hub


def housing(m: X.X430, mp: Plane, ym_from=-2.0):
    """모터 몸통의 폭 면 두 장 + 먼 끝 판. 구멍: 폭 면 4개씩(M2.5/M2.6), 먼 끝 4개."""
    x_in = X.HALF_W + X.CLEAR / 2
    y_far = X.FAR + X.CLEAR / 2
    parts = []
    for s in (1, -1):
        x0, x1 = sorted((s * x_in, s * (x_in + T_PLATE)))
        parts.append(mp * Pos((x0 + x1) / 2, (ym_from + y_far + T_PLATE) / 2, 0)
                     * Box(x1 - x0, y_far + T_PLATE - ym_from, X.D))
    parts.append(mp * Pos(0, y_far + T_PLATE / 2, 0) * Box(2 * (x_in + T_PLATE), T_PLATE, X.D))
    body = parts[0] + parts[1] + parts[2]
    side = Plane(origin=mp.origin, x_dir=mp.y_dir, z_dir=mp.x_dir)          # 2D (Ym, Zm), Xm 방향 구멍
    for (ym, zm) in X.SIDE_HOLES:
        if ym < ym_from + 2:
            continue
        body -= side * Pos(ym, zm, -30) * Cylinder(X.CASE_HOLE / 2, 60, align=MIN3)
        for s in (1, -1):
            body -= side * Pos(ym, zm, s * (x_in + T_PLATE - 1.2)) * Cylinder(X.CASE_HEAD / 2, 4, align=MIN3 if s > 0 else (Align.CENTER, Align.CENTER, Align.MAX))
    far = Plane(origin=mp.origin, x_dir=mp.x_dir, z_dir=mp.y_dir)          # 2D (Xm, Zm), Ym 방향 구멍
    for (xm, zm) in X.FAR_HOLES:
        body -= far * Pos(xm, -zm, 0) * Cylinder(X.CASE_HOLE / 2, 60, align=MIN3)
    return body


def fork_arm(pl: Plane, center_uv, dir_uv, d0, d1, outer_sign, length=BR1):
    """포크 팔 하나: 축 둘레 원판 + 다리 방향 띠 (BR1 까지). pl 은 2D(u,v) 평면, d0~d1 두께."""
    cu, cv = center_uv
    du, dv = dir_uv
    nu, nv = -dv, du
    pts = [(cu + nu * STRIP, cv + nv * STRIP), (cu + nu * STRIP + du * length, cv + nv * STRIP + dv * length),
           (cu - nu * STRIP + du * length, cv - nv * STRIP + dv * length), (cu - nu * STRIP, cv - nv * STRIP)]
    arm = extrude(pl.offset(d0) * Polygon(*pts, align=None), amount=d1 - d0, dir=pl.z_dir)
    arm += pl * Pos(cu, cv, d0) * Cylinder(ARM_R, d1 - d0, align=MIN3)
    arm -= pl * Pos(cu, cv, d0 - 1) * Cylinder(X.HUB_HOLE / 2, d1 - d0 + 2, align=MIN3)
    for k in range(8):                                                       # 16 mm 원 위 45° 간격 8개 (XL 은 4개, XM 은 8개 사용)
        a = math.radians(45 * k)
        hu, hv = cu + 8 * math.cos(a), cv + 8 * math.sin(a)
        arm -= pl * Pos(hu, hv, d0 - 1) * Cylinder(X.HORN_HOLE / 2, d1 - d0 + 2, align=MIN3)
        head_d = d1 - 1.5 if outer_sign > 0 else d0 - 0.5
        arm -= pl * Pos(hu, hv, head_d) * Cylinder(X.HORN_HEAD / 2, 2.0, align=MIN3)
    return arm


def fork_planes(m: X.X430, axis_offset):
    """포크 두 팔의 두께 구간 (모터 축 방향 좌표, 축 중심 = axis_offset). 혼 쪽(+), 아이들러 쪽(−)."""
    h0 = axis_offset + X.HALF_D + m.horn_proud + 0.2
    i1 = axis_offset - X.HALF_D - m.idler_proud - 0.2
    return (h0, h0 + T_ARM), (i1 - T_ARM, i1)


# ---------- 관절 위치 (FL, 설계 자세) ----------
def joints(b: Build):
    hx = np.array([HIP_X, HIP_Y, 0.0])                       # hip_x 축 위의 점 (모터 두께 중심)
    hy = np.array([HIP_X, HIP_Y + HIP_LINK, 0.0])            # hip_y 축 (모터 두께 중심)
    kn = hy + np.array([0, 0, -THIGH])
    ft = kn + np.array([0, 0, -SHANK])
    mp_hx = motor_plane(hx, zm=(1, 0, 0), ym=(0, -1, 0))                      # 먼 끝 → 몸 가운데
    mp_hy = motor_plane(hy, zm=(0, 1, 0), ym=-d2(b.hipy_center_deg))          # 가까운 끝 → 허벅지 범위 가운데
    mp_kn = motor_plane(kn, zm=(0, 1, 0), ym=-d2(b.knee_center_deg))          # 가까운 끝 → 정강이 범위 가운데
    return hx, hy, kn, ft, mp_hx, mp_hy, mp_kn


# ---------- 부품 ----------
def body_plates(b: Build):
    """몸통 위·아래 판: 옆으로 벌리기 모터 4개의 폭 면(위·아래)을 직접 잡는다 = 하우징 겸 몸통."""
    z_in = X.HALF_W + X.CLEAR / 2
    out = {}
    x_end = HIP_X + X.HALF_D + 1.0
    y_half = 48.0
    for name, s in (("body_top", 1), ("body_bottom", -1)):
        z0, z1 = sorted((s * z_in, s * (z_in + T_PLATE)))
        plate = Pos(0, 0, (z0 + z1) / 2) * Box(2 * x_end, 2 * y_half, z1 - z0)
        for sx in (1, -1):
            for sy in (1, -1):
                for (ym, zm) in X.SIDE_HOLES:                                  # 모터 폭 면 구멍: y = ±(39 − Ym), x = ±(93 ± 6)
                    hx_, hy_ = sx * (HIP_X + zm), sy * (HIP_Y - ym)
                    plate -= Pos(hx_, hy_, -30) * Cylinder(X.CASE_HOLE / 2, 60, align=MIN3)
                    plate -= Pos(hx_, hy_, s * (z_in + T_PLATE) - (1.2 if s > 0 else -1.2)) * Cylinder(
                        X.CASE_HEAD / 2, 3, align=MIN3 if s > 0 else (Align.CENTER, Align.CENTER, Align.MAX))
        # 어깨 포크의 몸 쪽 팔이 ±30° 돌 때 지나가는 자리 홈 (앞: 아이들러 팔, 뒤: 혼 팔)
        (fh0, fh1), (fi0, fi1) = fork_planes(b.hipx, HIP_X)
        rh0, rh1 = fh0 - 2 * HIP_X, fh1 - 2 * HIP_X
        for xa, xb in ((fi0 - 1.2, fi1 + 1.2), (rh0 - 1.2, rh1 + 1.2)):
            for sy in (1, -1):
                plate -= Pos((xa + xb) / 2, sy * (36 + 20), 0) * Box(xb - xa, 40, 60)
        for px in (-45, 45):                                                  # 위·아래 판을 잇는 기둥 자리 (M3)
            for py in (-30, 30):
                plate -= Pos(px, py, -30) * Cylinder(1.7, 60, align=MIN3)
        if s > 0:
            for px in (-29, 29):                                              # Raspberry Pi 장착 구멍 58 × 49 (M2.5)
                for py in (-24.5, 24.5):
                    plate -= Pos(px, py, -30) * Cylinder(1.4, 60, align=MIN3)
            plate -= Pos(0, 0, -30) * Box(40, 16, 60, align=MIN3)             # 케이블 구멍
        out[name] = plate
    pillar = Pos(0, 0, -z_in) * Cylinder(4.5, 2 * z_in, align=MIN3) - Pos(0, 0, -z_in - 1) * Cylinder(1.4, 2 * z_in + 2, align=MIN3)
    out["body_pillar"] = pillar                                               # 4개 인쇄 (M3 탭핑 나사로 위·아래 판 고정)
    return out


def shoulder(b: Build):
    hx, hy, kn, ft, mp_hx, mp_hy, mp_kn = joints(b)
    (h0, h1), (i0, i1) = fork_planes(b.hipx, HIP_X)
    pl = plane_yz(0)
    # 팔 띠는 다리(y ≤ 67)까지만: 더 길면 허벅지 아이들러 팔이 도는 면(y ≈ 70~74)에 들어간다
    arms = [fork_arm(pl, (HIP_Y, 0), (1, 0), d0, d1, sign, length=BR1 - 6) for (d0, d1), sign in (((h0, h1), 1), ((i0, i1), -1))]
    part = arms[0] + arms[1]
    y_br0, y_br1 = HIP_Y + BR0 - 3, HIP_Y + BR1 - 6                        # 포크 다리 (hip_x 축에서 21~28 mm)
    part += slab_x(rect(y_br0, y_br1, -STRIP, 42), i0, h1)                  # 다리 + 위로 올라가는 기둥
    # 허벅지 아이들러 팔이 지나가는 면(y ≈ 70~74)을 위쪽(z ≥ 17)으로 건너 hip_y 하우징으로
    (_, _), (ti0, ti1) = fork_planes(b.hipy, HIP_Y + HIP_LINK)
    deck_x0, deck_x1 = HIP_X - 22, HIP_X + 23
    part += slab_y(rect(deck_x0, deck_x1, 17, 42), y_br0, HIP_Y + HIP_LINK - X.HALF_D + 2)
    part += housing(b.hipy, mp_hy, ym_from=-2.0)
    # 덱과 하우징이 만나도록 하우징 바깥 덮개 (모터 두께 띠, 모터 위쪽)
    part += slab_y(rect(deck_x0, deck_x1, 30, 42), HIP_Y + HIP_LINK - X.HALF_D, HIP_Y + HIP_LINK + X.HALF_D)
    part -= motor_keepout(b.hipy, mp_hy)
    part -= rod(plane_xz(ti1 + 1.5), hy[0], hy[2], 16.0, 0, ti1 - ti0 + 3)  # 허벅지 아이들러 팔 원판이 도는 자리 (반지름 16)
    part -= motor_keepout(b.hipx, mp_hx)
    return part


def thigh(b: Build):
    hx, hy, kn, ft, mp_hx, mp_hy, mp_kn = joints(b)
    (h0, h1), (i0, i1) = fork_planes(b.hipy, hy[1])
    pl = plane_xz(0)                     # 2D (x, z), 법선 −y  →  두께 좌표 d 는 −y 방향
    part = None
    for (d0, d1), sign in (((h0, h1), 1), ((i0, i1), -1)):
        arm = fork_arm(pl, (hy[0], hy[2]), (0, -1), -d1, -d0, -sign)
        part = arm if part is None else part + arm
    band0, band1 = i1, h0                                                     # 모터 두께 띠 (팔 사이)
    part += slab_y(rect(hy[0] - STRIP, hy[0] + STRIP, -BR1, -BR0), band0, band1)   # 포크 다리
    # 허벅지 기둥: 포크 다리에서 무릎 하우징까지 (모터 두께 띠 안, 가운데를 비워 가볍게)
    part += slab_y(rect(hy[0] - 9, hy[0] + 9, kn[2] + 14, -BR0), hy[1] - X.HALF_D, hy[1] + X.HALF_D)
    part -= slab_y(rect(hy[0] - 5, hy[0] + 5, kn[2] + 40, -BR1 - 6), hy[1] - X.HALF_D + 4, hy[1] + X.HALF_D - 4)
    part += housing(b.knee, mp_kn, ym_from=-2.0)
    part -= motor_keepout(b.knee, mp_kn)
    return part


def shank(b: Build):
    hx, hy, kn, ft, mp_hx, mp_hy, mp_kn = joints(b)
    (h0, h1), (i0, i1) = fork_planes(b.knee, kn[1])
    pl = plane_xz(0)
    part = None
    for (d0, d1), sign in (((h0, h1), 1), ((i0, i1), -1)):
        arm = fork_arm(pl, (kn[0], kn[2]), (0, -1), -d1, -d0, -sign)
        part = arm if part is None else part + arm
    part += slab_y(rect(kn[0] - STRIP, kn[0] + STRIP, kn[2] - BR1, kn[2] - BR0), i1, h0)
    # 정강이: 위는 넓고 아래는 좁게 (발 쪽 가볍게)
    top_z, bot_z = kn[2] - BR0, ft[2] + 6
    prof = [(kn[0] - 9, top_z), (kn[0] + 9, top_z), (kn[0] + 6, bot_z), (kn[0] - 6, bot_z)]
    part += slab_y(prof, kn[1] - 8, kn[1] + 8)
    part += rod(Plane(origin=(0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, 0, 1)), kn[0], kn[1], 7.0, bot_z - 6, bot_z + 4)
    part -= rod(Plane(origin=(0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, 0, 1)), kn[0], kn[1], 1.4, bot_z - 8, bot_z + 10)   # 발 고정 M3 탭핑
    part -= motor_keepout(b.knee, mp_kn)
    return part


def foot(b: Build):
    hx, hy, kn, ft, *_ = joints(b)
    ball = Pos(*ft) * Solid.make_sphere(FOOT_R)
    ball -= Pos(ft[0], ft[1], ft[2] + 6) * Box(40, 40, 20, align=MIN3)        # 위를 잘라 정강이 끝에 끼움
    ball -= Pos(ft[0], ft[1], ft[2] - 2) * Cylinder(7.3, 10, align=MIN3)        # 정강이 끝 기둥 자리
    ball -= Pos(ft[0], ft[1], ft[2] - 20) * Cylinder(1.7, 40, align=MIN3)
    return ball


# ---------- 맞춤 시험 부품 (큰 부품을 뽑기 전에 먼저 인쇄) ----------
def fit_gauges(b: Build):
    pl = Plane.XY
    horn = fork_arm(pl, (0, 0), (1, 0), 0, 3, 1)                               # 혼 구멍 8개 + 가운데 구멍
    case_ring = (Box(X.W + 2 * X.CLEAR + 6, X.L + 2 * X.CLEAR + 6, 8) - Box(X.W + 2 * X.CLEAR, X.L + 2 * X.CLEAR, 10))
    side = Box(X.L, X.D, 2.5, align=(Align.MIN, Align.CENTER, Align.MIN))       # 폭 면 구멍 위치 확인판 (Y, Z)
    side = Pos(-X.NEAR, 0, 0) * side
    for (ym, zm) in X.SIDE_HOLES:
        side -= Pos(ym, zm, -1) * Cylinder(X.CASE_HOLE / 2, 5, align=MIN3)
    side -= Pos(0, 0, -1) * Cylinder(1.0, 5, align=MIN3)                         # 축 위치 표시
    return {"gauge_horn": horn, "gauge_case_ring": case_ring, "gauge_side_holes": side}


# ---------- 조립 (관절 각도 적용) ----------
def rotate_about(shape, point, axis, deg):
    from build123d import Axis
    return shape.rotate(Axis(tuple(float(v) for v in point), tuple(float(v) for v in axis)), deg)


def leg_parts(b: Build):
    hx, hy, kn, ft, mp_hx, mp_hy, mp_kn = joints(b)
    return {
        "shoulder": shoulder(b), "thigh": thigh(b), "shank": shank(b), "foot": foot(b),
        "m_hipx": motor_solid(b.hipx, mp_hx), "m_hipy": motor_solid(b.hipy, mp_hy), "m_knee": motor_solid(b.knee, mp_kn),
    }


def pose_leg(parts, b: Build, hipx_deg, thigh_deg, knee_deg):
    """FL 다리를 관절 각도로 움직인 조립 상태. thigh_deg: 허벅지가 아래에서 앞쪽으로, knee_deg: 무릎 굽힘(앞으로)."""
    hx, hy, kn, ft, *_ = joints(b)
    out = {}
    # 무릎: 정강이·발을 무릎 축(+y) 기준으로 −knee 회전 (y축 +회전 = 뒤로)
    for k in ("shank", "foot"):
        out[k] = rotate_about(parts[k], kn, (0, 1, 0), -knee_deg)
    # 허벅지: 허벅지·무릎 모터·정강이·발을 hip_y 축 기준 −thigh
    for k in ("thigh", "m_knee"):
        out[k] = parts[k]
    for k in ("thigh", "m_knee", "shank", "foot"):
        out[k] = rotate_about(out[k], hy, (0, 1, 0), -thigh_deg)
    # 옆으로 벌리기: 어깨부터 아래 전부를 hip_x 축(+x) 기준 회전
    out["shoulder"], out["m_hipy"] = parts["shoulder"], parts["m_hipy"]
    for k in ("shoulder", "m_hipy", "thigh", "m_knee", "shank", "foot"):
        out[k] = rotate_about(out[k], hx, (1, 0, 0), hipx_deg)
    out["m_hipx"] = parts["m_hipx"]
    return out
