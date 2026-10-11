"""3. 각도 ↔ 발 위치 — 정기구학과 역기구학.

다리 = 허벅지(0.1075 m) + 정강이(0.130 m) 두 막대.
  정기구학: 각도를 알면 발 위치를 계산할 수 있다 (삼각함수)
  역기구학: 발을 놓고 싶은 위치에서 필요한 각도를 거꾸로 계산한다 (코사인 법칙)
"""
from _setup import LEGS, Dog, forward_kinematics, inverse_kinematics

print("서기 자세 각도 → 발 위치:", [round(v, 3) for v in forward_kinematics(-67.1, 96.6)])
print("발 위치 (0, -0.17) → 각도:", [round(v, 1) for v in inverse_kinematics(0.0, -0.17)])

dog = Dog()
dog.reset()
dog.move_to("stand", 1.5)

# 팔굽혀펴기: 발끝 높이만 바꾸면 몸이 오르내린다
for z in [-0.13, -0.19, -0.13, -0.19]:
    dog.set_feet({leg: (-0.035, z) for leg in LEGS}, seconds=0.8)   # x = −0.035: 서기 자세의 발 위치
    print(f"발 높이 {z} m → 몸 높이 {dog.get_height():.3f} m")

# 인사하기: 앞발은 낮추고(z 를 0 쪽으로) 뒷발은 높이면 몸이 앞으로 숙여진다
dog.set_feet({"FL": (-0.035, -0.12), "FR": (-0.035, -0.12), "RL": (-0.035, -0.19), "RR": (-0.035, -0.19)}, 1.0)
print("인사  pitch = %.1f°" % dog.get_tilt()[1])
dog.move_to("stand", 1.0)
dog.move_to("lie", 1.5)
