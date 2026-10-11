"""DXL Dog v2 — Boston Dynamics Spot 을 참고해 겉모양을 다듬은 버전 (build123d).

v1(dxldog_cad.py)과 같은 것: 관절 위치·링크 길이(기구학), 모터 종류, 혼·케이스 나사 구멍, 포크 원리, 맞춤 시험 부품
  → 소프트웨어(관절 각도 규칙, 기구학)는 그대로 쓴다.
바뀐 것:
  몸통  : 위 모서리를 깎은 덮개(가운데 1 + 앞뒤 끝 2) + 앞·뒤 검은 얼굴판(카메라 창) + 위 레일 2개 (Spot 모양)
  어깨  : 앞뒤 관절(hip_y) 모터를 세워서(먼 끝이 위) 모서리가 둥근 '엉덩이 통'에 넣음 (v1 은 37.5° 기울어진 상자)
  허벅지: 무릎 모터를 허벅지 방향 가까이 눕혀(뒤로 25°) 허벅지 안에 넣음. 바깥 면은 엉덩이 원판에서 무릎까지 한 장
  정강이: 가늘고 둥근 모양, 포크 팔은 '원판 + 가는 목 + 끝 패드' 모양
좌표·관절 각도 규칙은 v1 과 같다 (x 앞, y 왼쪽, z 위, 원점 = 몸통 중심, FL 다리 설계 자세 = 다리 똑바로 아래).
"""
import math

import numpy as np
from build123d import (Align, Box, Circle, Cylinder, Kind, Plane, Polygon, Pos, RectangleRounded, Rot,
                       extrude, fillet, make_hull, offset)

import dxldog_cad as V1
import x430 as X
from dxldog_cad import (BR0, BR1, HIP_X, HIP_Y, MIN3, STRIP, T_PLATE, Build, fork_planes, housing, joints,
                        motor_keepout, plane_xz, plane_yz, rect, rod)
from sts3215 import STS3215

foot, fit_gauges, pose_leg, motor_solid = V1.foot, V1.fit_gauges, V1.pose_leg, V1.motor_solid

# ---------- v2 치수 ----------
NECK = 6.0             # 포크 팔 '목' 반폭 (원판과 끝 패드 사이)
WL = 13.0              # 허벅지 포크 다리 반폭
R_HIP = 22.0           # 허벅지 바깥 면 엉덩이 원판 반지름 (X430)
HIPY_CENTER = 0.0      # 앞뒤 관절 모터: 먼 끝이 똑바로 위
KNEE_CENTER = 25.0     # 무릎 모터: 먼 끝이 허벅지 방향에서 뒤로 25°
T_COV = 2.0            # 덮개 두께
XN, XS = 133.0, 114.0  # 덮개 앞 끝, 앞 비탈이 윗면과 만나는 x
X_MID = 64.5           # 가운데 덮개 ↔ 끝 덮개 나눔 (220 mm 프린터에 들어가게)
X_CUT0, XC = 66.0, 119.5   # 어깨가 지나가는 모서리 구멍 (x 범위)
X_NOTCH = 72.0         # 몸통 판을 좁히는 x 범위 (덮개 옆 벽·모서리 둥근 곳을 피함)
ZB, ZF, ZT = -24.0, 12.0, 44.0   # 덮개 아래 끝, 앞면 세로 부분 위, 윗면
Z_SIDE = 19.5          # 모서리 구멍 위쪽 (어깨가 ±30° 돌아도 닿지 않는 높이)
YW, CH = 48.0, 12.0    # 덮개 반폭, 위 긴 모서리 깎기
YW_IN = 45.6           # 가운데 부분 몸통 판 반폭 (덮개 옆 벽이 들어갈 자리)
COVER_SCREWS_MID = ((56.0, 0.0), (-56.0, 0.0))
COVER_SCREWS_END = ((85.0, 0.0), (105.0, 0.0))
RAIL_Y, RAIL_L = 22.0, 110.0

VARIANTS = {
    "xl": Build(hipy_center_deg=HIPY_CENTER, knee_center_deg=KNEE_CENTER),
    "xm": Build(X.XM430, X.XM430, X.XM430, HIPY_CENTER, KNEE_CENTER),
    "mixed": Build(X.XL430, X.XM430, X.XM430, HIPY_CENTER, KNEE_CENTER),
    "sts": Build(STS3215, STS3215, STS3215, HIPY_CENTER, KNEE_CENTER),
}


# ---------- 2D 도구 (평면의 u, v 좌표) ----------
def circ(c, r):
    return Pos(c[0], c[1]) * Circle(r)


def poly(pts):
    """다각형 (점 순서를 반시계로 맞춤: 방향이 섞이면 합치기·offset 이 실패한다)."""
    pts = [tuple(map(float, p)) for p in pts]
    area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]))
    return Polygon(*(pts if area > 0 else pts[::-1]), align=None)


def hull(*shapes):
    s = shapes[0]
    for t in shapes[1:]:
        s = s + t
    return make_hull(s.edges())


def round_out(s, r):
    """볼록한 모서리를 반지름 r 로 둥글게."""
    return offset(offset(s, -r, kind=Kind.ARC), r, kind=Kind.ARC)


def round_in(s, r):
    """오목한 모서리를 반지름 r 로 채워 둥글게."""
    return offset(offset(s, r, kind=Kind.ARC), -r, kind=Kind.ARC)


def slab_y(sk, y0, y1):
    """(x, z) 스케치를 y0 ~ y1 로 밀어내기."""
    return extrude(plane_xz(y1) * sk, amount=y1 - y0, dir=(0, -1, 0))


def slab_x(sk, x0, x1):
    """(y, z) 스케치를 x0 ~ x1 로 밀어내기."""
    return extrude(plane_yz(x0) * sk, amount=x1 - x0, dir=(1, 0, 0))


def slab_z(sk, z0, z1):
    """(x, y) 스케치를 z0 ~ z1 로 밀어내기."""
    return extrude(Plane.XY.offset(z0) * sk, amount=z1 - z0)


def strip2d(c, d, w, a0, a1):
    cu, cv = c
    du, dv = d
    nu, nv = -dv, du
    return poly([(cu + du * a0 + nu * w, cv + dv * a0 + nv * w), (cu + du * a1 + nu * w, cv + dv * a1 + nv * w),
                 (cu + du * a1 - nu * w, cv + dv * a1 - nv * w), (cu + du * a0 - nu * w, cv + dv * a0 - nv * w)])


def arm2d(c, d, r_hub, neck=NECK, pad=STRIP, l0=BR0, l1=BR1):
    """포크 팔 모양: 축 둘레 원판 + 가는 목(±neck) + 끝 패드(±pad, 축에서 l0−2 ~ l1). 오목한 곳은 둥글게."""
    s = circ(c, r_hub) + strip2d(c, d, neck, 0, l0) + strip2d(c, d, pad, l0 - 2, l1)
    return round_in(round_out(s, 2.0), 3.0)


def sweep2d(c, r_hub, neck, pad, k_lo, k_hi, clear, l0=BR0, l1=BR1):
    """arm2d 모양의 팔이 축 c 둘레로 k_lo ~ k_hi 도(아래에서 앞으로 +) 돌 때 쓸고 가는 영역 (+ 틈 clear). (x, z) 평면."""
    phi = np.radians(np.arange(0.0, 360.0, 0.5))
    ks = np.radians(np.arange(k_lo, k_hi + 1e-9, 1.0))
    d = phi[:, None] - ks[None, :]
    cs, sn = np.cos(d), np.abs(np.sin(d))
    w_n, w_p = neck + 3.0 + clear, pad + clear                      # 목 둘레 둥근 채움(3)까지
    with np.errstate(divide="ignore", invalid="ignore"):
        fwd = cs > 1e-9
        r_n = np.where(fwd, np.minimum((l0 + clear) / cs, np.where(sn > 1e-9, w_n / sn, np.inf)), 0.0)
        hit_p = fwd & ((l0 - 2 - clear) * sn / np.where(fwd, cs, 1) <= w_p)
        r_p = np.where(hit_p, np.minimum((l1 + clear) / cs, np.where(sn > 1e-9, w_p / sn, np.inf)), 0.0)
    rho = np.maximum(r_hub + 3.0 + clear, np.maximum(r_n, r_p).max(axis=1))
    return poly([(c[0] + r * math.sin(p), c[1] - r * math.cos(p)) for r, p in zip(rho, phi)])


# ---------- 구멍 ----------
def arm_holes(pl, m: X.Motor, c, d0, d1, outer_sign):
    """혼·아이들러 나사 구멍 (v1 fork_arm 과 같은 배치). pl 의 법선 방향 두께 d0~d1."""
    cu, cv = c
    cut = pl * Pos(cu, cv, d0 - 1) * Cylinder(m.hub_hole / 2, d1 - d0 + 2, align=MIN3)
    for (hx_, hy_) in m.horn_holes:
        cut += pl * Pos(cu + hx_, cv + hy_, d0 - 1) * Cylinder(m.horn_hole / 2, d1 - d0 + 2, align=MIN3)
        head_d = d1 - 1.5 if outer_sign > 0 else d0 - 0.5
        cut += pl * Pos(cu + hx_, cv + hy_, head_d) * Cylinder(m.horn_head / 2, 2.0, align=MIN3)
    return cut


def housing_holes(m: X.Motor, mp: Plane, ym_from=-2.0, head_len=30.0):
    """하우징 나사 구멍. v1 housing() 과 같은 자리지만 머리 자리를 바깥으로 길게 뚫어 덧붙인 살에 막히지 않게 한다."""
    x_in = m.HALF_W + X.CLEAR / 2
    zt, zb = m.z_top + X.CLEAR / 2, m.z_bot - X.CLEAR / 2
    up, dn = MIN3, (Align.CENTER, Align.CENTER, Align.MAX)
    cut = []
    for holes, z_face, sgn in ((m.top_holes, zt, 1), (m.bot_holes, zb, -1)):
        for (xm, ym) in holes:
            cut.append(mp * Pos(xm, ym, z_face - sgn * 6) * Cylinder(m.case_hole / 2, 12 + T_PLATE, align=up if sgn > 0 else dn))
            cut.append(mp * Pos(xm, ym, z_face + sgn * (T_PLATE - 1.2)) * Cylinder(m.case_head / 2, head_len, align=up if sgn > 0 else dn))
    side = Plane(origin=mp.origin, x_dir=mp.y_dir, z_dir=mp.x_dir)
    for (ym, zm) in m.side_holes:
        if ym < ym_from + 2:
            continue
        cut.append(side * Pos(ym, zm, -30) * Cylinder(m.case_hole / 2, 60, align=MIN3))
        for s in (1, -1):
            cut.append(side * Pos(ym, zm, s * (x_in + T_PLATE - 1.2)) * Cylinder(m.case_head / 2, head_len, align=up if s > 0 else dn))
    far = Plane(origin=mp.origin, x_dir=mp.x_dir, z_dir=mp.y_dir)
    for (xm, zm) in m.far_holes:
        cut.append(far * Pos(xm, -zm, 0) * Cylinder(m.case_hole / 2, 60, align=MIN3))
    out = cut[0]
    for c_ in cut[1:]:
        out += c_
    return out


# ---------- 다리 부품 ----------
def rot_sweep(pts, c, a_lo, a_hi, step=2.0):
    """(x, z) 다각형 pts(엉덩이 축 c 기준 고정)를 허벅지 좌표로 볼 때, 허벅지가 a_lo ~ a_hi 도 도는 동안 지나가는 영역."""
    out = None
    for a in np.arange(a_lo, a_hi + 1e-9, step):
        r = math.radians(-a)
        ca, sa = math.cos(r), math.sin(r)
        q = poly([(c[0] + ca * (x - c[0]) - sa * (z - c[1]), c[1] + sa * (x - c[0]) + ca * (z - c[1])) for x, z in pts])
        out = q if out is None else out + q
    return out


def pod_outer_zone(b: Build, H):
    """엉덩이 통이 허벅지 바깥 팔 면(y ≥ th0)까지 나온 부분(STS 의 바깥 면 장착판·앞판)이 허벅지 기준으로 쓸고 가는 영역."""
    mh = b.hipy
    if not mh.top_holes:
        return None
    (h0, h1), _ = fork_planes(b.hipx, HIP_X)
    xo = mh.HALF_W + X.CLEAR / 2 + T_PLATE + 2.5
    tab = [(H[0] - xo, H[1] + mh.face_tab_from - 2.5), (H[0] + xo, H[1] + mh.face_tab_from - 2.5),
           (H[0] + xo, H[1] + mh.FAR + 6), (H[0] - xo, H[1] + mh.FAR + 6)]
    plate = [(h0 - 2.5, -13.5), (h1 + 2.5, -13.5), (h1 + 2.5, H[1] + mh.FAR + 6), (h0 - 2.5, H[1] + mh.FAR + 6)]
    return rot_sweep(tab, H, -98, 23) + rot_sweep(plate, H, -98, 23)


def thigh(b: Build):
    hx, hy, kn, ft, mp_hx, mp_hy, mp_kn = joints(b)
    m, mk = b.hipy, b.knee
    (h0, h1), (i0, i1) = fork_planes(m, hy[1])
    H, K = (hy[0], hy[2]), (kn[0], kn[2])
    a = math.radians(b.knee_center_deg)
    F, N = np.array([-math.sin(a), math.cos(a)]), np.array([math.cos(a), math.sin(a)])   # 무릎 모터 먼 끝 방향, 폭 방향 (x, z)
    xo, yf = mk.HALF_W + X.CLEAR / 2 + T_PLATE, mk.FAR + X.CLEAR / 2 + T_PLATE

    def mpt(xm, ym):
        return np.array(K) + ym * F + xm * N

    house = poly([mpt(-xo, -2), mpt(xo, -2), mpt(xo, yf), mpt(-xo, yf)])
    house_far = poly([mpt(-xo, 18), mpt(xo, 18), mpt(xo, yf), mpt(-xo, yf)])
    leg = poly(rect(H[0] - WL, H[0] + WL, H[1] - BR1, H[1] - BR0))
    body2d = round_out(hull(leg, house_far), 4.0)
    # 허벅지 몸통: 포크 다리 ~ 무릎 하우징 (무릎 하우징 자리는 v1 하우징 벽만: 모터를 가까운 끝 쪽에서 밀어 넣을 수 있게)
    part = slab_y(body2d - house, i1, h0) + housing(mk, mp_kn, ym_from=-2.0)
    pocket = offset(hull(leg, house_far), -3.0, kind=Kind.ARC) - house - poly(rect(H[0] - 40, H[0] + 40, H[1] - BR1 - 3, H[1] + 50))
    part -= slab_y(round_out(pocket, 2.0), i1 - 1, h0 - 2)                     # 안쪽(−y)으로 열린 홈: 가볍게, 받침 없이 인쇄
    # 바깥 면: 엉덩이 원판 ~ 무릎 하우징까지 한 장. 정강이 바깥 팔이 도는 자리는 비움
    shank_zone = sweep2d(K, mk.arm_r, NECK, STRIP, -5, 130, 2.0)
    outer2d = hull(circ(H, R_HIP), leg, house_far) - shank_zone
    zone = pod_outer_zone(b, H)
    if zone is not None:                                                       # STS: 엉덩이 통 바깥 면 장착판을 피함
        outer2d = (outer2d - zone) + arm2d(H, (0, -1), m.arm_r, pad=WL)
    outer2d = round_out(outer2d, 2.0)
    part += slab_y(outer2d, h0, h1)
    part += slab_y(arm2d(H, (0, -1), m.arm_r, pad=WL), i0, i1)                 # 안쪽 팔
    part -= motor_keepout(mk, mp_kn)
    part -= motor_keepout(m, mp_hy)
    part -= housing_holes(mk, mp_kn, ym_from=-2.0)
    pl = plane_xz(0)
    part -= arm_holes(pl, m, H, -h1, -h0, -1) + arm_holes(pl, m, H, -i1, -i0, 1)
    return part


def shank(b: Build):
    hx, hy, kn, ft, mp_hx, mp_hy, mp_kn = joints(b)
    m = b.knee
    (h0, h1), (i0, i1) = fork_planes(m, kn[1])
    K = (kn[0], kn[2])
    arm = arm2d(K, (0, -1), m.arm_r)
    part = slab_y(arm, h0, h1) + slab_y(arm, i0, i1)
    leg = poly(rect(K[0] - STRIP, K[0] + STRIP, K[1] - BR1, K[1] - BR0))
    part += slab_y(round_out(leg, 3.0), i1, h0)
    bot_z = ft[2] + 6
    body2d = round_out(hull(leg, circ((K[0], bot_z + 2), 6.5)), 3.0)
    body = slab_y(body2d, kn[1] - 8, kn[1] + 8)
    try:                                                                       # 긴 모서리 둥글게 (Spot 의 정강이처럼)
        body = fillet(body.edges().filter_by(lambda e: abs(e.tangent_at(0.5).Y) < 0.5), 3.0)
    except Exception:
        pass
    part += body
    zpl = Plane(origin=(0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, 0, 1))
    part += rod(zpl, kn[0], kn[1], 7.0, bot_z - 6, bot_z + 4)
    part -= rod(zpl, kn[0], kn[1], 1.4, bot_z - 8, bot_z + 10)                 # 발 고정 M3 탭핑
    part -= motor_keepout(m, mp_kn)
    pl = plane_xz(0)
    part -= arm_holes(pl, m, K, -h1, -h0, -1) + arm_holes(pl, m, K, -i1, -i0, 1)
    return part


def shoulder(b: Build):
    hx, hy, kn, ft, mp_hx, mp_hy, mp_kn = joints(b)
    m, mh = b.hipx, b.hipy
    (h0, h1), (i0, i1) = fork_planes(m, HIP_X)            # 옆 관절 포크 팔 (x 범위)
    (th0, th1), (ti0, ti1) = fork_planes(mh, hy[1])       # 허벅지 팔이 도는 면 (y 범위)
    A = (HIP_Y, 0.0)                                      # 옆 관절 축 (y, z)
    yc0, yc1 = HIP_Y + BR0 - 3, ti0 - 1.5                 # 가로대 (허벅지 안쪽 팔 면 앞에서 멈춤)
    zl, zh = -11.0, 9.0
    xo = mh.HALF_W + X.CLEAR / 2 + T_PLATE
    tab = T_PLATE if mh.top_holes else 0.0
    yb0, yb1 = hy[1] + mh.z_bot - X.CLEAR / 2, hy[1] + mh.z_top + X.CLEAR / 2
    ztop = mh.FAR + X.CLEAR / 2 + T_PLATE
    y_up = max(yb0, ti1 + 1.0)                            # 이보다 바깥에서만 위로 올라감 (허벅지 안쪽 팔 면을 피함)
    bar = poly(rect(yc0, yc1, zl, zh))
    neck = circ(A, m.arm_r) + strip2d(A, (1, 0), NECK, 0, BR0 - 2)
    idler = slab_x(round_in(round_out(neck + bar, 2.0), 3.0), i0, i1)
    crossbar = slab_x(round_out(bar, 2.0), i0, h1)
    front = slab_x(round_in(round_out(neck + poly(rect(yc0, yb1 - 5, zl, zh)), 2.0), 3.0), h0, h1)
    # 엉덩이 통: 모터 두께 띠를 꽉 채운 상자(모터 자리는 아래에서 뺌) + 하우징(면 장착판) + 앞판 위쪽, 모서리를 둥글게.
    # 뒤·위 벽을 1.5 mm 두껍게 해서 둥근 모서리에도 살이 남는다.
    x0, zt2 = HIP_X - xo - 1.5, ztop + 1.5
    block = slab_y(poly(rect(x0, h1, zl, zt2)), yb0, yb1)
    upper = slab_x(poly(rect(y_up, yb1 + tab, zl, zt2)), h0, h1)
    env_xz = round_out(poly(rect(x0, h1, zl, zt2)), 4.0) & round_out(poly(rect(x0, h1, zl - 30, zt2)), 10.0)
    env = slab_y(env_xz, yb0 - tab - 0.1, yb1 + tab + 0.1) & slab_z(round_out(poly(rect(x0, h1, yb0 - tab - 0.1, yb1 + tab + 0.1)), 5.0), zl - 1, zt2 + 1)
    pod = (block + housing(mh, mp_hy, ym_from=zl) + upper) & env
    part = idler + crossbar + front + pod
    part -= motor_keepout(mh, mp_hy)
    part -= motor_keepout(m, mp_hx)
    part -= slab_y(sweep2d((hy[0], hy[2]), mh.arm_r, NECK, WL, -98, 23, 1.5), ti0 - 1, ti1 + 1)   # 허벅지 안쪽 팔이 도는 자리
    if mh.top_holes:
        part -= slab_y(sweep2d((hy[0], hy[2]), mh.arm_r, NECK, WL, -98, 23, 1.5), th0 - 1, th1 + 1)
    part -= housing_holes(mh, mp_hy, ym_from=zl)
    pl = plane_yz(0)
    part -= arm_holes(pl, m, A, i0, i1, -1) + arm_holes(pl, m, A, h0, h1, 1)
    return part


# ---------- 몸통 ----------
def z_plate(b: Build):
    return b.hipx.HALF_W + X.CLEAR / 2 + T_PLATE          # 위 판 윗면 높이


def body_plates(b: Build):
    """v1 의 위·아래 판 + 가운데 부분을 좁힘(덮개 옆 벽 자리) + 덮개 나사 구멍."""
    out = V1.body_plates(b)
    for name in ("body_top", "body_bottom"):
        p = out[name]
        for sy in (1, -1):
            p -= Pos(0, sy * (YW_IN + 15), 0) * Box(2 * X_NOTCH, 30, 80)
        for (px, py) in COVER_SCREWS_MID + COVER_SCREWS_END + tuple((-x_, y_) for x_, y_ in COVER_SCREWS_END):
            r = 1.7 if name == "body_top" else 3.5                             # 아래 판: 드라이버·나사 머리가 지나가는 구멍
            p -= Pos(px, py, -40) * Cylinder(r, 80, align=MIN3)
        out[name] = p
    return out


def _cover_env(t=0.0):
    """덮개 바깥 모양 (t > 0 이면 안쪽으로 t 만큼 줄인 모양, 아래는 뚫림)."""
    side = poly([(-XN, ZB), (XN, ZB), (XN, ZF), (XS, ZT), (-XS, ZT), (-XN, ZF)])
    front = poly([(-YW, ZB), (YW, ZB), (YW, ZT - CH), (YW - CH, ZT), (-(YW - CH), ZT), (-YW, ZT - CH)])
    top = round_out(poly(rect(-XN, XN, -YW, YW)), 14.0)
    if t:
        side = offset(side, -t, kind=Kind.INTERSECTION) + poly(rect(-XN + t + 1, XN - t - 1, ZB - 10, ZB + t + 1))
        front = offset(front, -t, kind=Kind.INTERSECTION) + poly(rect(-YW + t + 1, YW - t - 1, ZB - 10, ZB + t + 1))
        top = offset(top, -t, kind=Kind.ARC)
    return slab_y(side, -70, 70) & slab_x(front, -150, 150) & slab_z(top, ZB - 20, ZT + 20)


def _slope_plane():
    s = np.array([XS - XN, 0.0, ZT - ZF])
    L = float(np.linalg.norm(s))
    n = np.array([ZT - ZF, 0.0, XN - XS]) / L
    return Plane(origin=(XN, 0, ZF), x_dir=(0, 1, 0), z_dir=tuple(n)), L


def _face_region():
    pl, L = _slope_plane()
    return pl, Pos(0, L / 2) * RectangleRounded(64, 24, 5)


def _boss(b, px, py):
    z0 = z_plate(b)
    s = Pos(px, py, z0) * Cylinder(4.5, ZT - T_COV + 0.5 - z0, align=MIN3)
    return s - Pos(px, py, z0 - 1) * Cylinder(1.25, 13, align=MIN3)          # M3 탭핑 나사 (위 판 아래에서 올려 조임)


def covers(b: Build):
    """조립 위치의 덮개들: cover_mid, cover_end_F/R, face_F/R, rail_L/R."""
    shell = _cover_env(0) - _cover_env(T_COV)
    mid = shell & Pos(0, 0, 0) * Box(2 * X_MID, 200, 200)
    lip_out = _cover_env(T_COV + 0.2) - _cover_env(T_COV + 1.4)               # 끝 덮개 안쪽 면과 0.2 mm 틈
    lip_in = _cover_env(T_COV - 0.1) - _cover_env(T_COV + 1.4)                # 가운데 덮개와 붙는 부분
    for sx in (1, -1):                                                         # 끝 덮개 안으로 들어가는 턱
        mid += lip_out & Pos(sx * (X_MID + 3), 0, Z_SIDE + 50.5) * Box(6, 200, 100)
        mid += lip_in & Pos(sx * (X_MID - 2), 0, Z_SIDE + 50.5) * Box(4.0, 200, 100)
        mid += _boss(b, sx * 56.0, 0.0)
    for sy in (1, -1):                                                         # 레일 나사 구멍 (M3, 20 mm 간격)
        for k in range(-2, 3):
            mid -= Pos(k * 20.0, sy * RAIL_Y, ZT - 5) * Cylinder(1.25, 10, align=MIN3)
    end = shell & Pos((X_MID + XN + 5) / 2, 0, 0) * Box(XN + 5 - X_MID, 200, 200)
    end -= slab_y(round_out(poly(rect(X_CUT0, XC, ZB - 30, Z_SIDE)), 6.0), -70, 70)   # 어깨가 지나가는 모서리
    for (px, py) in COVER_SCREWS_END:
        end += _boss(b, px, py)
    pl, region = _face_region()
    end -= extrude(pl.offset(-T_COV - 1) * region, amount=T_COV + 2)          # 얼굴판 창
    face = extrude(pl.offset(-T_COV) * offset(region, -0.2, kind=Kind.ARC), amount=T_COV)
    face += extrude(pl.offset(-T_COV - 1.2) * offset(region, 2.5, kind=Kind.ARC), amount=1.2)   # 안쪽 턱 (안에서 끼움)
    _, L = _slope_plane()
    for u in (-17.0, 17.0):                                                    # Spot 처럼 두 '눈' (오목)
        face -= extrude(pl.offset(-0.8) * (Pos(u, L / 2) * RectangleRounded(20, 9, 4)), amount=2)
    face -= extrude(pl.offset(-T_COV - 2) * (Pos(0, L / 2) * Circle(4.0)), amount=5)   # 카메라 렌즈 구멍 ⚠️ 카메라 고정은 아직
    rail = Pos(0, 0, ZT) * Box(RAIL_L, 8, 5, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for k in range(-2, 3):
        rail -= Pos(k * 20.0, 0, ZT - 1) * Cylinder(1.7, 10, align=MIN3)
        rail -= Pos(k * 20.0, 0, ZT + 2.5) * Cylinder(3.0, 5, align=MIN3)
    rot = Rot(0, 0, 180)
    return {"cover_mid": mid, "cover_end_F": end, "cover_end_R": rot * end, "face_F": face, "face_R": rot * face,
            "rail_L": Pos(0, RAIL_Y, 0) * rail, "rail_R": Pos(0, -RAIL_Y, 0) * rail}


# ---------- 조립 ----------
def leg_parts(b: Build):
    hx, hy, kn, ft, mp_hx, mp_hy, mp_kn = joints(b)
    return {
        "shoulder": shoulder(b), "thigh": thigh(b), "shank": shank(b), "foot": foot(b),
        "m_hipx": motor_solid(b.hipx, mp_hx), "m_hipy": motor_solid(b.hipy, mp_hy), "m_knee": motor_solid(b.knee, mp_kn),
    }
