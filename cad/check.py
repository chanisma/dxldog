"""관절 범위 전체에서 부품·모터끼리 부딪히지 않는지 검사 (겹치는 부피 > 0.5 mm³ 이면 충돌로 표시).
실행: ../.venv/Scripts/python check.py [xl|xm|mixed|sts] [v1|v2] [fine]
  fine 을 붙이면 범위를 촘촘히 나눈 405개 자세 (기본 75개, v2 는 무릎 0° 를 더해 90개)
  앞다리(FL)와 뒷다리(RL)를 몸통·옆 관절 모터 4개와 함께 검사한다 (오른쪽 다리는 좌우 거울이라 같다). v2 는 덮개·얼굴판·레일도.
"""
import itertools
import sys
import time

import numpy as np

from render import build_of, design
from dxldog_cad import HIP_X

LIMIT = 0.5


def grid(style, fine):
    if fine:
        return (tuple(np.linspace(-30, 30, 5)), tuple(np.linspace(-95, 20, 9)), tuple(np.linspace(0 if style == "v2" else 15, 125, 9)))
    knee = (0, 15, 45, 70, 100, 125) if style == "v2" else (15, 45, 70, 100, 125)
    return (-30, 0, 30), (-95, -70, -37.5, -10, 20), knee


def overlap(a, b):
    ba, bb = a.bounding_box(), b.bounding_box()
    if (ba.max.X < bb.min.X or bb.max.X < ba.min.X or ba.max.Y < bb.min.Y or bb.max.Y < ba.min.Y
            or ba.max.Z < bb.min.Z or bb.max.Z < ba.min.Z):
        return 0.0
    inter = a & b
    return sum(s.volume for s in inter.solids()) if inter else 0.0


def main(variant="xl", style="v1", fine=False):
    D, b = design(style), build_of(variant, style)
    parts = D.leg_parts(b)
    body = D.body_plates(b)
    static = {"body_top": body["body_top"], "body_bottom": body["body_bottom"], "m_hipx": parts["m_hipx"],
              "m_hipx_R": parts["m_hipx"].translate((-2 * HIP_X, 0, 0))}
    legs = ["", "R:"]
    if style == "v2":
        import dxldog_v2
        static.update(dxldog_v2.covers(b))
    groups = [("어깨", ["shoulder", "m_hipy"]), ("허벅지", ["thigh", "m_knee"]), ("정강이", ["shank", "foot"])]
    worst = {}

    def note(key, v, pose):
        if v > worst.get(key, (0, None))[0]:
            worst[key] = (v, pose)

    for a_, c_ in itertools.combinations(static, 2):                     # 몸통 부품끼리 (자세와 무관)
        note((a_, c_), overlap(static[a_], static[c_]), None)
    t = time.perf_counter()
    n = 0
    HIPX, THIGH, KNEE = grid(style, fine)
    for hx, th, kn in itertools.product(HIPX, THIGH, KNEE):
        posed = D.pose_leg(parts, b, hx, th, kn)
        pose = (hx, th, kn)
        for (ga, na), (gb, nb) in itertools.combinations(groups, 2):     # 한 다리 안의 부품끼리
            for a in na:
                for c in nb:
                    note((a, c), overlap(posed[a], posed[c]), pose)
        for tag in legs:                                                 # 다리 ↔ 몸통 (앞다리, 뒷다리)
            for _, names in groups:
                for a in names:
                    s = posed[a] if not tag else posed[a].translate((-2 * HIP_X, 0, 0))
                    for c, sc in static.items():
                        note((tag + a, c), overlap(s, sc), pose)
        n += 1
    print(f"[{variant} {style}] 자세 {n}개 검사, {time.perf_counter() - t:.0f}s")
    bad = {k: v for k, v in worst.items() if v[0] > LIMIT}
    for (a, c), (v, pose) in sorted(bad.items(), key=lambda kv: -kv[1][0]):
        where = "고정 부품" if pose is None else f"옆 {pose[0]:g}°, 허벅지 {pose[1]:g}°, 무릎 {pose[2]:g}°"
        print(f"  충돌 {a:12s} ↔ {c:12s} 최대 {v:8.1f} mm³  ({where})")
    if not bad:
        print("  충돌 없음")
    return bad


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if a != "fine"]
    main(args[0] if args else "xl", args[1] if len(args) > 1 else "v1", "fine" in sys.argv[1:])
