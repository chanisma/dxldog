"""관절 범위 전체에서 부품·모터끼리 부딪히지 않는지 검사 (겹치는 부피 > 0.5 mm³ 이면 충돌로 표시).
실행: ../.venv/Scripts/python check.py [xl|xm|mixed]
"""
import itertools
import sys
import time

from build123d import Compound

from build_all import VARIANTS
from dxldog_cad import body_plates, leg_parts, pose_leg

HIPX = (-30, 0, 30)
THIGH = (-95, -70, -37.5, -10, 20)
KNEE = (15, 45, 70, 100, 125)
LIMIT = 0.5


def overlap(a, b):
    ba, bb = a.bounding_box(), b.bounding_box()
    if (ba.max.X < bb.min.X or bb.max.X < ba.min.X or ba.max.Y < bb.min.Y or bb.max.Y < ba.min.Y
            or ba.max.Z < bb.min.Z or bb.max.Z < ba.min.Z):
        return 0.0
    inter = a & b
    return sum(s.volume for s in inter.solids()) if inter else 0.0


def main(variant="xl"):
    b = VARIANTS[variant]
    parts = leg_parts(b)
    body = body_plates(b)
    static = {"body_top": body["body_top"], "body_bottom": body["body_bottom"], "m_hipx": parts["m_hipx"]}
    groups = [("몸통", ["body_top", "body_bottom", "m_hipx"]), ("어깨", ["shoulder", "m_hipy"]),
              ("허벅지", ["thigh", "m_knee"]), ("정강이", ["shank", "foot"])]
    worst = {}
    t = time.perf_counter()
    n = 0
    for hx, th, kn in itertools.product(HIPX, THIGH, KNEE):
        posed = pose_leg(parts, b, hx, th, kn)
        posed.update(static)
        for (ga, na), (gb, nb) in itertools.combinations(groups, 2):
            for a in na:
                for c in nb:
                    v = overlap(posed[a], posed[c])
                    key = (a, c)
                    if v > worst.get(key, (0, None))[0]:
                        worst[key] = (v, (hx, th, kn))
        n += 1
    print(f"[{variant}] 자세 {n}개 검사, {time.perf_counter() - t:.0f}s")
    bad = {k: v for k, v in worst.items() if v[0] > LIMIT}
    for (a, c), (v, pose) in sorted(bad.items(), key=lambda kv: -kv[1][0]):
        print(f"  충돌 {a:12s} ↔ {c:12s} 최대 {v:8.1f} mm³  (옆 {pose[0]}°, 허벅지 {pose[1]}°, 무릎 {pose[2]}°)")
    if not bad:
        print("  충돌 없음")
    return bad


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "xl")
