"""모터 치수 — DYNAMIXEL X430 계열(XL430 / XC430 / XM430). ROBOTIS 공식 도면(XL-430 2018-03-24, XM430/XH430 2019-03-19)에서 읽은 값.
다른 모터(Feetech STS3215)는 sts3215.py. 두 파일 모두 아래 Motor 형식을 쓴다.

모터 좌표계 (단위 mm):
  원점 = 출력축 중심 (X430 은 몸통 두께의 가운데)
  +Z  = 혼(출력) 쪽,  −Z = 아이들러(뒤) 쪽
  +Y  = 몸통 먼 끝 방향 (축에서 먼 끝까지 FAR, 가까운 끝까지 NEAR)
  X   = 폭 방향 (±W/2)
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Motor:
    name: str
    horn_proud: float      # 케이스 혼 쪽 면에서 혼 바깥면까지 (+Z)
    idler_proud: float     # 케이스 뒤쪽 면에서 아이들러 바깥면까지 (−Z)
    horn_d: float          # 혼 지름
    screw: str             # 케이스 장착 나사
    mass_g: float
    W: float = 28.5        # 폭 (X)
    L: float = 46.5        # 길이 (Y)
    z_top: float = 17.0    # 혼 쪽 케이스 면 (+Z)
    z_bot: float = -17.0   # 뒤쪽 케이스 면 (−Z)
    NEAR: float = 11.25    # 축 → 가까운 끝
    FAR: float = 35.25     # 축 → 먼 끝
    horn_holes: tuple = tuple((round(8 * math.cos(math.radians(45 * k)), 4), round(8 * math.sin(math.radians(45 * k)), 4))
                              for k in range(8))   # 혼·아이들러 나사 (X, Y). X430: 16 mm 원 위 45° 간격 8개 (XL 4개, XM 8개 사용)
    horn_hole: float = 2.3      # 혼 나사 통과 구멍 (M2)
    horn_head: float = 4.4      # 혼 나사 머리 자리
    hub_hole: float = 9.0       # 가운데 (혼 고정 나사 머리 / 아이들러 가운데)
    arm_r: float = 13.0         # 포크 팔의 축 둘레 원판 반지름
    side_holes: tuple = tuple((y, z) for y in (4.0, 28.0) for z in (-6.0, 6.0))   # 폭 면(±X) 장착 구멍 (Y, Z)
    far_holes: tuple = tuple((x, z) for x in (-8.0, 8.0) for z in (-6.0, 6.0))    # 먼 끝 면(+Y) 장착 구멍 (X, Z)
    top_holes: tuple = ()       # 혼 쪽 면(+Z) 장착 구멍 (X, Y) — STS3215 처럼 앞·뒤 면으로 다는 모터
    bot_holes: tuple = ()       # 뒤쪽 면(−Z) 장착 구멍 (X, Y)
    face_tab_from: float = 19.0  # 앞·뒤 면 장착판이 시작하는 Y (포크 팔·커넥터를 피함)
    case_hole: float = 2.8      # 케이스 나사 통과 구멍
    case_head: float = 5.0      # 머리 자리
    plug_box: tuple = ()        # 뒤쪽 면 커넥터+꽂은 케이블 자리 (X0, X1, Y0, Y1, 깊이) — 부품이 들어오면 안 됨

    @property
    def D(self):
        return self.z_top - self.z_bot

    @property
    def zc(self):
        return (self.z_top + self.z_bot) / 2

    @property
    def HALF_W(self):
        return self.W / 2


XL430 = Motor("XL430/XC430", horn_proud=3.9, idler_proud=3.0, horn_d=20.5, screw="M2.6 탭핑 나사 (FHS M2.6×8)", mass_g=65)
XM430 = Motor("XM430/XH430", horn_proud=2.2, idler_proud=2.0, horn_d=19.5, screw="M2.5×0.45 탭 (WB M2.5)", mass_g=82)
X430 = Motor          # 예전 이름

# 예전 코드 호환용 (X430 공통 값)
W, L, D = XL430.W, XL430.L, XL430.D
NEAR, FAR = XL430.NEAR, XL430.FAR
HALF_W, HALF_D = W / 2, D / 2
PCD = 16.0
HORN_HOLE, HORN_HEAD, HUB_HOLE = XL430.horn_hole, XL430.horn_head, XL430.hub_hole
SIDE_HOLES, FAR_HOLES = list(XL430.side_holes), list(XL430.far_holes)
CASE_HOLE, CASE_HEAD = XL430.case_hole, XL430.case_head

CLEAR = 0.3                         # 모터 몸통과 프린트 부품 사이 틈


def fork_gap(m: Motor):
    """포크 두 팔 안쪽 면 사이 거리 = 혼 바깥면 ~ 아이들러 바깥면 (+ 약간의 틈)."""
    return m.D + m.horn_proud + m.idler_proud + 0.4
