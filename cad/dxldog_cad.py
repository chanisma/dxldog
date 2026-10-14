"""DXL Dog — SpotMicro 를 버스 서보(DYNAMIXEL X430 또는 Feetech STS3215)용으로 다시 설계한 3D 프린트 부품 (build123d, 치수를 숫자로 바꿀 수 있음).

기준 치수(SpotMicro, mm): 엉덩이 간격 앞뒤 186 · 좌우 78, 엉덩이 링크 55, 허벅지 107.5, 정강이 130.
좌표: x 앞, y 왼쪽, z 위. 원점 = 몸통 중심. 아래 값은 앞왼쪽(FL) 다리, '설계 자세'(허벅지·정강이가 똑바로 아래) 기준.
나머지 다리는 거울 대칭(좌우·앞뒤)으로 만든다. 모터를 거울 대칭하면 '혼이 반대쪽인 같은 모터'가 되므로 실제로 조립 가능하다.

관절마다 같은 원리:
  하우징 = 앞 부품이 모터 몸통의 폭 면(±Xm)과 먼 끝 면을 잡는다 (모터 두께 띠 안).
           나사: X430 은 폭 면 구멍 4개씩, STS3215 는 앞·뒤 면의 먼 쪽 구멍 2개씩(면 장착판)
  포크   = 다음 부품의 두 팔이 혼(+Zm)과 아이들러(−Zm)를 바깥에서 잡고, 축에서 24~34 mm 떨어진 '다리'로 이어진다
  → 다리 쪽 부품은 모터 몸통이 향한 방향(먼 끝 ±45°)만 피하면 되므로 관절이 넓게 돈다.
"""
import math
from dataclasses import dataclass

import numpy as np
from build123d import (Align, Axis, Box, Cylinder, Location, Plane, Polygon, Pos, Solid, Compound, extrude)

import x430 as X

MIN3 = (Align.CENTER, Align.CENTER, Align.MIN)

# ---------- 기본 치수 ----------
HIP_X, HIP_Y, HIP_LINK = 93.0, 39.0, 55.0
THIGH, SHANK = 107.5, 130.0
T_ARM = 4.0            # 포크 팔 두께
T_PLATE = 3.0          # 하우징 옆판 두께
STRIP = 11.0           # 포크 팔 띠 반폭
BR0, BR1 = 24.0, 34.0  # 포크 '다리'가 놓이는 축으로부터의 거리 범위
FOOT_R = 12.0


@dataclass
class Build:
    hipx: X.Motor = X.XL430    # 옆으로 벌리기 모터
    hipy: X.Motor = X.XL430    # 앞뒤 흔들기 모터
    knee: X.Motor = X.XL430    # 무릎 모터
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


def motor_solid(m: X.Motor, mp: Plane):
    case = mp * Pos(0, (m.FAR - m.NEAR) / 2, m.zc) * Box(m.W, m.L, m.D)
    horn = mp * Pos(0, 0, m.z_top) * Cylinder(m.horn_d / 2, m.horn_proud, align=MIN3)
    idler = mp * Pos(0, 0, m.z_bot - m.idler_proud) * Cylinder(m.horn_d / 2, m.idler_proud, align=MIN3)
    out = case + horn + idler
    if m.plug_box:                                                              # 뒤쪽 커넥터에 꽂은 케이블
        x0, x1, y0, y1, depth = m.plug_box
        out += mp * Pos((x0 + x1) / 2, (y0 + y1) / 2, m.z_bot - depth / 2) * Box(x1 - x0, y1 - y0, depth)
    return out


def motor_keepout(m: X.Motor, mp: Plane, clear=X.CLEAR):
    """프린트 부품이 들어오면 안 되는 모터 몸통 공간 (틈 포함, 혼·아이들러 쪽 원판, 커넥터·케이블 자리 포함)."""
    case = mp * Pos(0, (m.FAR - m.NEAR) / 2, m.zc) * Box(m.W + 2 * clear, m.L + 2 * clear, m.D + 2 * clear)
    hub = mp * Pos(0, 0, m.z_bot - m.idler_proud - 0.2) * Cylinder(m.horn_d / 2 + 0.6, m.D + m.horn_proud + m.idler_proud + 0.4, align=MIN3)
    out = case + hub
    if m.plug_box:
        x0, x1, y0, y1, depth = m.plug_box
        out += mp * Pos((x0 + x1) / 2, (y0 + y1) / 2, m.z_bot - depth / 2) * Box(x1 - x0 + 1, y1 - y0 + 1, depth + 1)
    return out


def housing(m: X.Motor, mp: Plane, ym_from=-2.0):
    """모터 몸통의 폭 면 두 장 + 먼 끝 판 (+ 앞·뒤 면 장착판). 구멍은 모터 종류에 따라."""
    x_in = m.HALF_W + X.CLEAR / 2
    y_far = m.FAR + X.CLEAR / 2
    band = m.D + (X.CLEAR if (m.top_holes or m.bot_holes) else 0.0)          # 면 장착판이 있으면 판과 닿도록 틈까지 (0.15 mm 떠 있던 문제)
    parts = []
    for s in (1, -1):
        x0, x1 = sorted((s * x_in, s * (x_in + T_PLATE)))
        parts.append(mp * Pos((x0 + x1) / 2, (ym_from + y_far + T_PLATE) / 2, m.zc)
                     * Box(x1 - x0, y_far + T_PLATE - ym_from, band))
    parts.append(mp * Pos(0, y_far + T_PLATE / 2, m.zc) * Box(2 * (x_in + T_PLATE), T_PLATE, band))
    body = parts[0] + parts[1] + parts[2]
    zt, zb = m.z_top + X.CLEAR / 2, m.z_bot - X.CLEAR / 2
    if m.top_holes or m.bot_holes:                                             # 앞·뒤 면 장착판 (먼 쪽만: 포크 팔·커넥터를 피함)
        y0 = m.face_tab_from
        for z0 in (zt, zb - T_PLATE):
            body += mp * Pos(0, (y0 + y_far + T_PLATE) / 2, z0 + T_PLATE / 2) * Box(2 * (x_in + T_PLATE), y_far + T_PLATE - y0, T_PLATE)
        for holes, z_face, sgn in ((m.top_holes, zt, 1), (m.bot_holes, zb, -1)):
            for (xm, ym) in holes:
                hole_align = MIN3 if sgn > 0 else (Align.CENTER, Align.CENTER, Align.MAX)
                body -= mp * Pos(xm, ym, z_face - sgn * 6) * Cylinder(m.case_hole / 2, 12 + T_PLATE, align=hole_align)
                body -= mp * Pos(xm, ym, z_face + sgn * (T_PLATE - 1.2)) * Cylinder(m.case_head / 2, 4, align=hole_align)
    side = Plane(origin=mp.origin, x_dir=mp.y_dir, z_dir=mp.x_dir)          # 2D (Ym, Zm), Xm 방향 구멍
    for (ym, zm) in m.side_holes:
        if ym < ym_from + 2:
            continue
        body -= side * Pos(ym, zm, -30) * Cylinder(m.case_hole / 2, 60, align=MIN3)
        for s in (1, -1):
            body -= side * Pos(ym, zm, s * (x_in + T_PLATE - 1.2)) * Cylinder(m.case_head / 2, 4, align=MIN3 if s > 0 else (Align.CENTER, Align.CENTER, Align.MAX))
    far = Plane(origin=mp.origin, x_dir=mp.x_dir, z_dir=mp.y_dir)          # 2D (Xm, Zm), Ym 방향 구멍
    for (xm, zm) in m.far_holes:
        body -= far * Pos(xm, -zm, 0) * Cylinder(m.case_hole / 2, 60, align=MIN3)
    return body


def fork_arm(pl: Plane, m: X.Motor, center_uv, dir_uv, d0, d1, outer_sign, length=BR1):
    """포크 팔 하나: 축 둘레 원판 + 다리 방향 띠 (BR1 까지). pl 은 2D(u,v) 평면, d0~d1 두께. 구멍은 모터 혼 모양대로."""
    cu, cv = center_uv
    du, dv = dir_uv
    nu, nv = -dv, du
    pts = [(cu + nu * STRIP, cv + nv * STRIP), (cu + nu * STRIP + du * length, cv + nv * STRIP + dv * length),
           (cu - nu * STRIP + du * length, cv - nv * STRIP + dv * length), (cu - nu * STRIP, cv - nv * STRIP)]
    arm = extrude(pl.offset(d0) * Polygon(*pts, align=None), amount=d1 - d0, dir=pl.z_dir)
    arm += pl * Pos(cu, cv, d0) * Cylinder(m.arm_r, d1 - d0, align=MIN3)
    arm -= pl * Pos(cu, cv, d0 - 1) * Cylinder(m.hub_hole / 2, d1 - d0 + 2, align=MIN3)
    for (hx_, hy_) in m.horn_holes:                                            # X430: 16 mm 원 위 8개, STS3215: 10 mm 정사각형 4개
        hu, hv = cu + hx_, cv + hy_
        arm -= pl * Pos(hu, hv, d0 - 1) * Cylinder(m.horn_hole / 2, d1 - d0 + 2, align=MIN3)
        head_d = d1 - 1.5 if outer_sign > 0 else d0 - 0.5
        arm -= pl * Pos(hu, hv, head_d) * Cylinder(m.horn_head / 2, 2.0, align=MIN3)
    return arm


def fork_planes(m: X.Motor, axis_offset):
    """포크 두 팔의 두께 구간 (모터 축 방향 좌표, 축 중심 = axis_offset). 혼 쪽(+), 아이들러 쪽(−)."""
    h0 = axis_offset + m.z_top + m.horn_proud + 0.2
    i1 = axis_offset + m.z_bot - m.idler_proud - 0.2
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
    m = b.hipx
    z_in = m.HALF_W + X.CLEAR / 2
    out = {}
    x_end = HIP_X + m.z_top + 1.0
    y_half = 48.0
    for name, s in (("body_top", 1), ("body_bottom", -1)):
        z0, z1 = sorted((s * z_in, s * (z_in + T_PLATE)))
        plate = Pos(0, 0, (z0 + z1) / 2) * Box(2 * x_end, 2 * y_half, z1 - z0)
        for sx in (1, -1):
            for sy in (1, -1):
                for (ym, zm) in m.side_holes:                                  # 모터 폭 면 구멍: y = ±(39 − Ym), x = ±(93 ± 6)
                    hx_, hy_ = sx * (HIP_X + zm), sy * (HIP_Y - ym)
                    plate -= Pos(hx_, hy_, -30) * Cylinder(m.case_hole / 2, 60, align=MIN3)
                    plate -= Pos(hx_, hy_, s * (z_in + T_PLATE) - (1.2 if s > 0 else -1.2)) * Cylinder(
                        m.case_head / 2, 3, align=MIN3 if s > 0 else (Align.CENTER, Align.CENTER, Align.MAX))
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
        if s < 0 and (m.top_holes or m.bot_holes):
            plate += hipx_walls(m, z_in)
        out[name] = plate
    pillar = Pos(0, 0, -z_in) * Cylinder(4.5, 2 * z_in, align=MIN3) - Pos(0, 0, -z_in - 1) * Cylinder(1.4, 2 * z_in + 2, align=MIN3)
    out["body_pillar"] = pillar                                               # 4개 인쇄 (M3 탭핑 나사로 위·아래 판 고정)
    return out


def hipx_walls(m: X.Motor, z_in):
    """(STS3215 처럼 앞·뒤 면으로 다는 모터) 옆으로 벌리기 모터 4개의 앞·뒤 면을 잡는 세운 벽. 아래 판과 한 몸으로 인쇄한다.
    모터 먼 끝이 몸 가운데(−y)를 향하므로 벽은 y = 39 − Ym 쪽에 선다. 나사는 바깥(±x)에서 넣는다."""
    walls = None
    y_lo, y_hi = HIP_Y - (m.FAR + X.CLEAR / 2 + T_PLATE), HIP_Y - m.face_tab_from
    for shift in (0.0, -2 * HIP_X):                                          # 뒷다리 모터는 앞 모터를 그대로 뒤로 옮긴 것 (거울 아님)
        for sy in (1, -1):
            for holes, z_face, sgn in ((m.top_holes, m.z_top + X.CLEAR / 2, 1), (m.bot_holes, m.z_bot - X.CLEAR / 2, -1)):
                xa = HIP_X + z_face if sgn > 0 else HIP_X + z_face - T_PLATE
                # 아래로 판 두께만큼 더 내려 판과 겹친다: 바깥쪽 벽은 판 끝(x_end)에 0.15–0.85 mm 만 걸쳐 있던 문제
                wall = Pos(shift + xa + T_PLATE / 2, sy * (y_lo + y_hi) / 2, -T_PLATE / 2) * Box(T_PLATE, y_hi - y_lo, 2 * z_in + T_PLATE)
                for (xm, ym) in holes:                                       # 모터 X = 세계 z, 모터 Y → 세계 y = 39 − Y
                    pl = Plane(origin=(shift + HIP_X + z_face, sy * (HIP_Y - ym), xm), x_dir=(0, 1, 0), z_dir=(sgn, 0, 0))
                    wall -= pl * Pos(0, 0, -6) * Cylinder(m.case_hole / 2, 12 + T_PLATE, align=MIN3)
                    wall -= pl * Pos(0, 0, T_PLATE - 1.2) * Cylinder(m.case_head / 2, 4, align=MIN3)
                walls = wall if walls is None else walls + wall
    return walls


def shoulder(b: Build):
    hx, hy, kn, ft, mp_hx, mp_hy, mp_kn = joints(b)
    (h0, h1), (i0, i1) = fork_planes(b.hipx, HIP_X)
    pl = plane_yz(0)
    # 팔 띠는 다리(y ≤ 67)까지만: 더 길면 허벅지 아이들러 팔이 도는 면(y ≈ 70~74)에 들어간다
    arms = [fork_arm(pl, b.hipx, (HIP_Y, 0), (1, 0), d0, d1, sign, length=BR1 - 6) for (d0, d1), sign in (((h0, h1), 1), ((i0, i1), -1))]
    part = arms[0] + arms[1]
    y_br0, y_br1 = HIP_Y + BR0 - 3, HIP_Y + BR1 - 6                        # 포크 다리 (hip_x 축에서 21~28 mm)
    part += slab_x(rect(y_br0, y_br1, -STRIP, 42), i0, h1)                  # 다리 + 위로 올라가는 기둥
    # 허벅지 아이들러 팔이 지나가는 면(y ≈ 70~74)을 위쪽(z ≥ 17)으로 건너 hip_y 하우징으로
    (_, _), (ti0, ti1) = fork_planes(b.hipy, HIP_Y + HIP_LINK)
    deck_x0, deck_x1 = HIP_X - 22, HIP_X + 23
    part += slab_y(rect(deck_x0, deck_x1, 17, 42), y_br0, HIP_Y + HIP_LINK + b.hipy.z_bot + 2)
    part += housing(b.hipy, mp_hy, ym_from=-2.0)
    # 덱과 하우징이 만나도록 하우징 바깥 덮개 (모터 두께 띠, 모터 위쪽)
    part += slab_y(rect(deck_x0, deck_x1, 30, 42), HIP_Y + HIP_LINK + b.hipy.z_bot, HIP_Y + HIP_LINK + b.hipy.z_top)
    part -= motor_keepout(b.hipy, mp_hy)
    part -= rod(plane_xz(ti1 + 1.5), hy[0], hy[2], b.hipy.arm_r + 3, 0, ti1 - ti0 + 3)  # 허벅지 아이들러 팔 원판이 도는 자리
    part -= motor_keepout(b.hipx, mp_hx)
    return part


def thigh(b: Build):
    hx, hy, kn, ft, mp_hx, mp_hy, mp_kn = joints(b)
    (h0, h1), (i0, i1) = fork_planes(b.hipy, hy[1])
    pl = plane_xz(0)                     # 2D (x, z), 법선 −y  →  두께 좌표 d 는 −y 방향
    part = None
    for (d0, d1), sign in (((h0, h1), 1), ((i0, i1), -1)):
        arm = fork_arm(pl, b.hipy, (hy[0], hy[2]), (0, -1), -d1, -d0, -sign)
        part = arm if part is None else part + arm
    band0, band1 = i1, h0                                                     # 모터 두께 띠 (팔 사이)
    part += slab_y(rect(hy[0] - STRIP, hy[0] + STRIP, -BR1, -BR0), band0, band1)   # 포크 다리
    # 허벅지 기둥: 포크 다리에서 무릎 하우징까지 (모터 두께 띠 안, 가운데를 비워 가볍게)
    part += slab_y(rect(hy[0] - 9, hy[0] + 9, kn[2] + 14, -BR0), hy[1] + b.hipy.z_bot, hy[1] + b.hipy.z_top)
    part -= slab_y(rect(hy[0] - 5, hy[0] + 5, kn[2] + 40, -BR1 - 6), hy[1] + b.hipy.z_bot + 4, hy[1] + b.hipy.z_top - 4)
    part += housing(b.knee, mp_kn, ym_from=-2.0)
    part -= motor_keepout(b.knee, mp_kn)
    return part


def shank(b: Build):
    hx, hy, kn, ft, mp_hx, mp_hy, mp_kn = joints(b)
    (h0, h1), (i0, i1) = fork_planes(b.knee, kn[1])
    pl = plane_xz(0)
    part = None
    for (d0, d1), sign in (((h0, h1), 1), ((i0, i1), -1)):
        arm = fork_arm(pl, b.knee, (kn[0], kn[2]), (0, -1), -d1, -d0, -sign)
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
SCREW_STEPS = (-0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2)   # 설계값(m.horn_hole) 기준 구멍 지름 차이


def screw_gauge(m: X.Motor, t=4.0):
    """혼 나사 구멍 크기 시험판: 왼쪽부터 지름 horn_hole−0.4 … +0.2 (0.1 mm 간격), 설계값 구멍 위에 홈 표시.
    두께 t 는 포크 팔과 같고, 위쪽에 머리 자리(지름 horn_head, 깊이 1.5)가 있다. 나사가 손으로 끝까지 들어가는
    가장 작은 구멍을 골라 그 지름을 모터 파일의 horn_hole 에 적는다."""
    pitch = m.horn_head + 3.0
    n = len(SCREW_STEPS)
    g = Box(pitch * n, pitch + 2, t, align=(Align.MIN, Align.CENTER, Align.MIN))
    for i, dd in enumerate(SCREW_STEPS):
        x = pitch * (i + 0.5)
        g -= Pos(x, 0, -1) * Cylinder((m.horn_hole + dd) / 2, t + 2, align=MIN3)
        g -= Pos(x, 0, t - 1.5) * Cylinder(m.horn_head / 2, 2.0, align=MIN3)
        if dd == 0.0:                                                          # 설계값 표시 홈
            g -= Pos(x, (pitch + 2) / 2, t - 1.0) * Box(1.2, 2.4, 2.0)
    g -= Pos(0, -(pitch + 2) / 2, 0) * Box(3.0, 3.0, 2 * t + 2).rotate(Axis.Z, 45)   # 가장 작은 구멍 쪽 모서리 깎기
    return g


def fit_gauges(b: Build):
    m = b.knee
    pl = Plane.XY
    horn = fork_arm(pl, m, (0, 0), (1, 0), 0, 3, 1)                            # 혼 구멍 + 가운데 구멍
    case_ring = (Box(m.W + 2 * X.CLEAR + 6, m.L + 2 * X.CLEAR + 6, 8) - Box(m.W + 2 * X.CLEAR, m.L + 2 * X.CLEAR, 10))
    out = {"gauge_horn": horn, "gauge_case_ring": case_ring, "gauge_horn_screw": screw_gauge(m)}
    if m.side_holes:
        side = Box(m.L, m.D, 2.5, align=(Align.MIN, Align.CENTER, Align.MIN))   # 폭 면 구멍 위치 확인판 (Y, Z)
        side = Pos(-m.NEAR, 0, 0) * side
        for (ym, zm) in m.side_holes:
            side -= Pos(ym, zm, -1) * Cylinder(m.case_hole / 2, 5, align=MIN3)
        side -= Pos(0, 0, -1) * Cylinder(1.0, 5, align=MIN3)                     # 축 위치 표시
        out["gauge_side_holes"] = side
    if m.top_holes:                                                            # 앞·뒤 면 구멍 위치 확인판 (X, Y): 원판 구멍을 혼에 끼워 대 본다
        for name, holes in (("gauge_face_top", m.top_holes), ("gauge_face_bottom", m.bot_holes)):
            g = Pos(0, (m.FAR - m.NEAR) / 2, 0) * Box(m.W, m.L, 2.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
            g -= Pos(0, 0, -1) * Cylinder(m.horn_d / 2 + 0.6, 5, align=MIN3)
            for (xm, ym) in holes:
                g -= Pos(xm, ym, -1) * Cylinder(m.case_hole / 2, 5, align=MIN3)
            out[name] = g
    return out


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
