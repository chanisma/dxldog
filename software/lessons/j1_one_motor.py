"""1. 모터 하나 움직이기 — 목표 각도를 주면 모터가 그 각도로 간다.

로봇은 엎드린 상태에서 시작한다. 앞왼쪽(FL) 다리의 관절 3개를 하나씩 움직여 보자.
"""
from _setup import Dog
from dxldog import deg_to_tick, motor_info

dog = Dog()
dog.reset()       # 엎드린 처음 자세에서 시작 (실물은 2초 동안 천천히 엎드림)

for joint, goal in [("hip_x", 20), ("hip_y", -30), ("knee", 100)]:
    before = dog.get_angle("FL", joint)
    dog.set_angle("FL", joint, goal)       # 목표 각도 주기
    dog.wait(0.6)
    after = dog.get_angle("FL", joint)     # 현재 각도 읽기 (모터 센서)
    dxl_id, model = motor_info("FL", joint)
    print(f"FL {joint:5s}: {before:6.1f}° → 목표 {goal}° → 실제 {after:6.1f}°   "
          f"(모터 ID {dxl_id} {model}, Goal Position = {deg_to_tick('FL', joint, goal)})")

# 생각해 보기: 목표와 실제 각도가 조금 다른 이유는? (중력, 모터의 힘)
# 생각해 보기: Goal Position 값은 0~4095. 2048 이 가운데(다리 똑바로 아래)일 때 1° 는 몇 칸일까? (4096 ÷ 360)
# 도전: 가동 범위 밖의 값(예: knee 200°)을 주면 어떻게 될까?
