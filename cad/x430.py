"""DYNAMIXEL X430 계열(XL430 / XC430 / XM430) 치수 — ROBOTIS 공식 도면(XL-430 2018-03-24, XM430/XH430 2019-03-19)에서 읽은 값.

모터 좌표계 (단위 mm):
  원점 = 출력축 중심, 몸통 두께(34)의 가운데
  +Z  = 혼(출력) 쪽,  −Z = 아이들러(뒤) 쪽
  +Y  = 몸통 먼 끝 방향 (축에서 먼 끝까지 35.25, 가까운 끝까지 11.25)
  X   = 폭 방향 (±14.25)
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class X430:
    name: str
    horn_proud: float      # 케이스 면에서 혼 바깥면까지 (+Z)
    idler_proud: float     # 케이스 면에서 아이들러 바깥면까지 (−Z)
    horn_d: float          # 혼 지름
    screw: str             # 케이스 장착 나사
    mass_g: float


XL430 = X430("XL430/XC430", horn_proud=3.9, idler_proud=3.0, horn_d=20.5, screw="M2.6 탭핑 나사 (FHS M2.6×8)", mass_g=65)
XM430 = X430("XM430/XH430", horn_proud=2.2, idler_proud=2.0, horn_d=19.5, screw="M2.5×0.45 탭 (WB M2.5)", mass_g=82)

W, L, D = 28.5, 46.5, 34.0          # 폭(X), 길이(Y), 두께(Z)
NEAR, FAR = 11.25, 35.25            # 축 → 가까운 끝 / 먼 끝
HALF_W, HALF_D = W / 2, D / 2

PCD = 16.0                          # 혼·아이들러 나사 원 지름 (XL 4개, XM 8개 → 부품에는 45° 간격 8개를 뚫는다)
HORN_HOLE = 2.3                     # M2 나사 통과 구멍
HORN_HEAD = 4.4                     # M2 머리 자리 (카운터보어)
HUB_HOLE = 9.0                      # 가운데 (혼 고정 나사 머리 / 아이들러 베어링)

SIDE_HOLES = [(y, z) for y in (4.0, 28.0) for z in (-6.0, 6.0)]   # 폭 면(±X)의 장착 구멍 (Y, Z)
FAR_HOLES = [(x, z) for x in (-8.0, 8.0) for z in (-6.0, 6.0)]     # 먼 끝 면(+Y)의 장착 구멍 (X, Z)
CASE_HOLE = 2.8                     # M2.5 / M2.6 통과 구멍
CASE_HEAD = 5.0                     # 머리 자리

CLEAR = 0.3                         # 모터 몸통과 프린트 부품 사이 틈


def fork_gap(m: X430):
    """포크 두 팔 안쪽 면 사이 거리 = 혼 바깥면 ~ 아이들러 바깥면 (+ 약간의 틈)."""
    return D + m.horn_proud + m.idler_proud + 0.4
