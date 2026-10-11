"""4. 다리 하나 들기 — 무게중심과 지지 다각형.

다리를 하나 들면 남은 세 발이 만드는 삼각형(지지 다각형) 안에 무게중심이 있어야 넘어지지 않는다.
앞왼쪽(FL) 다리를 들면 남는 발은 FR·RL·RR → 삼각형의 빗변이 FR 과 RL 을 잇는 대각선이다.
서기 자세에서 무게중심은 이 대각선 바로 위에 있다 → 그냥 들면 넘어진다.
"""
from _setup import LEGS, Dog, inverse_kinematics

LIFT_FL = {"FL": (0, -80, 120)}            # 앞왼쪽 다리를 접어 올린 자세 (hip_x, hip_y, knee)

dog = Dog()
dog.reset()

# 실험 A: 그냥 든다
if dog.mode == "sim":                      # ⚠️ 실물로는 하지 않는다 (넘어짐)
    dog.move_to("stand", 1.5)
    print("서기 자세 발 위치 (x, z):", [round(v, 3) for v in dog.get_foot("FL")])
    dog.move_to(LIFT_FL, 0.6)
    dog.wait(1.0)
    print("A. 그냥 들기          roll %5.1f°, pitch %5.1f°  ← 넘어짐!" % dog.get_tilt())
    dog.reset()

# 실험 B: 무게중심을 뒤로 옮긴 다음 든다
dog.move_to("stand", 1.5)
hip_y, knee = inverse_kinematics(0.0, -0.155)          # 네 발을 몸 기준 3.5 cm 앞으로 → 몸(무게중심)이 3.5 cm 뒤로
dog.move_to({leg: (0, hip_y, knee) for leg in LEGS}, 1.0)
dog.move_to(LIFT_FL, 0.6)
dog.wait(1.0)
print("B. 몸을 뒤로 옮기고 들기 roll %5.1f°, pitch %5.1f°  ← 버팀!" % dog.get_tilt())
dog.move_to("stand", 1.0)
dog.move_to("lie", 1.5)

# 생각해 보기: 왜 '뒤로' 옮겼을까? 앞왼쪽 다리를 들면 남은 삼각형은 로봇의 뒤·오른쪽에 있다.
# 도전: 0.0 대신 −0.02, −0.01 로 바꿔 보자. 몇 cm 부터 버틸까?
# 실물: IMU 가 없으면 get_tilt() 가 (0, 0) — 기울어지는지 눈으로 보자.
