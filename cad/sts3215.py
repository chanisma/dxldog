"""Feetech STS3215 (LeRobot SO-100/SO-101 의 모터) 치수.

⚠️ Feetech 의 장착 도면을 구하지 못해서, SO-ARM100 저장소의 3D 모델(Simulation/SO101/assets/sts3215_03a_v1.stl)을
잰 값이다. 판매처 사양 45.2 × 24.7 × 35 mm, 55 g 과 맞는다. 큰 부품을 뽑기 전에 맞춤 시험 부품(gauge_*)으로 꼭 확인한다.

모터 좌표계는 x430.py 와 같다: 원점 = 출력축 중심, +Z 혼 쪽, +Y 먼 끝 쪽.
  케이스 혼 쪽 면 +17.0, 뒤쪽 면 −17.7 (두께 34.7, 축이 0.35 mm 혼 쪽으로 치우침)
  혼(앞)·아이들러(뒤) 모두 지름 20 원판, 나사 구멍 4개가 한 변 10 mm 정사각형 (축 양쪽으로 도는 '두 축' 모터)
  장착 구멍은 옆면이 아니라 앞·뒤 면에 있다: 앞면 (±10.4, 8.3), (±10.4, 29.2) / 뒷면 (±10.4, 8.3), (±10.4, 32.9)
  뒷면에 커넥터 2개(축에서 Y 11.5–17 mm) — 케이블을 꽂으면 아래로 튀어나온다
"""
from x430 import Motor

STS3215 = Motor(
    "STS3215", horn_proud=1.7, idler_proud=0.6, horn_d=20.0, screw="M2 ⚠️ (케이스 구멍 지름 약 2 mm)", mass_g=55,
    W=24.8, L=45.4, z_top=17.0, z_bot=-17.7, NEAR=10.2, FAR=35.2,
    horn_holes=tuple((x, y) for x in (-5.0, 5.0) for y in (-5.0, 5.0)),
    horn_hole=2.3, horn_head=4.4, hub_hole=8.0, arm_r=11.0,
    side_holes=(), far_holes=(),
    top_holes=((-10.4, 29.2), (10.4, 29.2)), bot_holes=((-10.4, 32.9), (10.4, 32.9)),
    face_tab_from=19.0, case_hole=2.2, case_head=4.2,
    plug_box=(-9.8, 9.8, 11.5, 17.0, 12.0),   # 커넥터 핀이 축에서 Y 14.1 mm, X ±3–8 mm (3핀 × 2)
)
