"""2. 자세 — 관절 12개를 한꺼번에 부드럽게 움직이기.

move_to(자세, 초) 는 지금 각도에서 목표 각도까지 정해진 시간 동안 조금씩 옮긴다.
자세 이름: "lie"(엎드리기), "stand"(서기), "sit"(앉기), "zero"(다리 똑바로)
"""
from dxldog import Dog

dog = Dog()

for pose in ["stand", "sit", "stand", "lie"]:
    dog.move_to(pose, 1.5)
    dog.wait(0.5)
    print(f"{pose:5s} → 몸 높이 {dog.get_height() * 100:.1f} cm")

# 직접 만든 자세: (hip_x, hip_y, knee) 하나를 주면 네 다리 모두 같은 각도
dog.move_to((0, -40, 80), 1.0)
print("내 자세:", dog.get_angles()["FL"])

# 도전: {"FL": (...), "FR": (...), ...} 처럼 다리마다 다른 각도를 줘서 인사하는 자세를 만들어 보자.
