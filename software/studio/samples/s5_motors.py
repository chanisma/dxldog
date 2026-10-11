"""5. 모터 번호·위치값·온도 — Dog 함수 뒤에서 실제 모터(DYNAMIXEL 또는 STS3215)에 무엇이 가는지 보기.

모터마다 ID(1~12)가 있고, '목표 위치(Goal Position)'에 0~4095 사이 숫자를 쓰면 그 위치로 돈다.
(주소: DYNAMIXEL 은 116, STS3215 는 42 — 회사마다 표가 다르지만 위치값 규칙은 같다)
2048 이 가운데(다리 똑바로 아래), 한 바퀴 = 4096.
"""
from dxldog import Dog, deg_to_tick, motor_info

dog = Dog()
dog.move_to("stand", 1.5)

for leg in ["FL", "FR"]:
    for joint in ["hip_x", "hip_y", "knee"]:
        motor_id, model = motor_info(leg, joint)
        angle = dog.get_angle(leg, joint)
        print(f"{leg} {joint:5s}: ID {motor_id:2d} ({model}) 각도 {angle:6.1f}° → 위치값 {deg_to_tick(leg, joint, angle)}")

# 왼쪽(FL)과 오른쪽(FR) 다리는 모터가 거울처럼 달려 있어서, 같은 각도라도 위치값이 2048 을 기준으로 반대쪽이다.

temps = dog.get_temperatures()
hottest = max((t, leg, i) for leg, ts in temps.items() for i, t in enumerate(ts))
print(f"가장 뜨거운 모터: {hottest[1]} {['hip_x', 'hip_y', 'knee'][hottest[2]]} {hottest[0]:.1f}°C")

# 도전: 오래 서 있으면 온도가 어떻게 바뀔까? dog.wait(30) 뒤에 다시 재 보자. (엎드리면 식는다)
