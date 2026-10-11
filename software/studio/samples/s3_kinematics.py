"""3. 각도 ↔ 발 위치 (기구학)

정기구학: 관절 각도 → 발끝 위치   forward_kinematics(hip_y, knee) → (x, z)
역기구학: 발끝 위치 → 관절 각도   inverse_kinematics(x, z) → (hip_y, knee)
x 는 앞(+)·뒤(−), z 는 엉덩이 관절 아래 깊이(음수), 단위 m. 허벅지 10.75 cm + 정강이 13 cm.
"""
from dxldog import Dog, forward_kinematics, inverse_kinematics

dog = Dog()
dog.move_to("stand", 1.5)

x, z = forward_kinematics(-67.1, 96.6)
print(f"서기 자세의 발끝: x = {x * 100:.1f} cm, z = {z * 100:.1f} cm")

hy, kn = inverse_kinematics(0.0, -0.17)
print(f"발을 17 cm 아래에 두려면: hip_y = {hy:.1f}°, knee = {kn:.1f}°")

for depth in [0.15, 0.18, 0.13, 0.155]:      # 네 발을 같이 내리고 올리기 → 몸이 오르내림
    dog.set_feet({leg: (-0.035, -depth) for leg in ["FL", "FR", "RL", "RR"]}, seconds=0.8)
    dog.wait(0.3)
    print(f"발 깊이 {depth * 100:.1f} cm → 몸 높이 {dog.get_height() * 100:.1f} cm")

# 서기 자세의 발은 엉덩이보다 3.5 cm 뒤(x = −0.035)에 있다: 무릎이 모두 뒤로 굽어 무게중심이 뒤쪽이기 때문
# 도전: 앞발만 앞으로(x = +0.01) 내밀면 몸이 어떻게 될까?
