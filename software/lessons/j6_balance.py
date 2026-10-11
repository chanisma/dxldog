"""6. 센서로 균형 잡기 — 기울기 센서(IMU)와 자이로.

J5 의 트롯에서 DUTY 를 0.5 로 하면(두 발로만 서는 시간이 길다) 몸이 좌우로 크게 흔들리고 방향이 틀어진다.
센서로 몸의 상태를 재서 다리를 고쳐 주면(피드백) 더 안정하게 걸을 수 있다.
  기울기 get_tilt()  → (roll 옆, pitch 앞뒤) [도]       몸이 얼마나 기울었나
  자이로 get_gyro()  → (roll, pitch, yaw 회전 속도) [도/초]  몸이 얼마나 빨리 기우나
  방향   get_heading() [도]                            몸이 어느 쪽을 보나

균형 잡기: 몸이 기운 쪽 다리는 늘이고, 반대쪽 다리는 줄인다.
  고치는 양 [cm] = GAIN × 기울기 + DAMP × 기우는 속도
방향 잡기: 왼쪽으로 돌아가 있으면 왼발 보폭을 늘여 오른쪽으로 돌린다.
"""
from _setup import Dog, trot_feet

T = 0.5            # 한 주기 (초)
DUTY = 0.5         # 딛는 시간 비율 — 일부러 흔들리는 값
GAIN = 0.6         # 1° 기울면 다리를 0.6 cm 고친다
DAMP = 0.007       # 1 °/s 로 기울고 있으면 0.007 cm 더 고친다 (자이로: 미리 막기)
TURN_GAIN = 0.2    # 방향이 1° 틀어지면 왼발·오른발 보폭을 0.2 cm 다르게


def balance(feet):
    """발끝 위치 {다리: (x, z)} 를 센서 값으로 고쳐서 돌려준다."""
    roll, pitch = dog.get_tilt()
    roll_rate, pitch_rate, _ = dog.get_gyro()
    fix_pitch = (GAIN * pitch + DAMP * pitch_rate) / 100     # cm → m
    fix_roll = (GAIN * roll + DAMP * roll_rate) / 100
    new = {}
    for leg, (x, z) in feet.items():
        front = 1 if leg[0] == "F" else -1                   # 앞다리 +1, 뒷다리 −1
        left = 1 if leg[1] == "L" else -1                    # 왼다리 +1, 오른다리 −1
        # 앞이 숙여지면(pitch +) 앞다리를 늘이고(z 를 더 아래로), 왼쪽이 들리면(roll +) 왼다리를 줄인다
        new[leg] = (x, z - front * fix_pitch + left * fix_roll)
    return new


def walk(name, use_balance, use_heading, cycles=10):
    dog.reset()
    dog.move_to("stand", 1.5)
    x0, y0 = dog.get_position()
    h0 = dog.get_heading()
    worst = 0.0
    t = 0.0
    while t < cycles * T:
        turn = 0.0
        if use_heading:
            error = dog.get_heading() - h0                     # + 면 왼쪽으로 돌아가 있음
            turn = max(-0.02, min(0.02, -TURN_GAIN * error / 100))
        feet = trot_feet(t, T, duty=DUTY, turn=turn)
        if use_balance:
            feet = balance(feet)
        dog.set_feet(feet, seconds=0)
        dog.wait(0.01)
        t += 0.01
        worst = max(worst, max(abs(a) for a in dog.get_tilt()))
    x1, y1 = dog.get_position()
    print(f"{name}: 앞으로 {x1 - x0:+.2f} m, 방향 {dog.get_heading() - h0:+4.0f}°, 가장 크게 기운 각도 {worst:4.1f}°")


dog = Dog()
if not dog.has_imu:
    print("이 예제는 기울기 센서(IMU)가 필요해요. 시뮬레이터로 해 보세요: python j6_balance.py")
    raise SystemExit

walk("A. 센서 없이        ", use_balance=False, use_heading=False)
walk("B. 균형 잡기        ", use_balance=True, use_heading=False)
walk("C. 균형 + 방향 잡기 ", use_balance=True, use_heading=True, cycles=20)
dog.move_to("lie", 1.5)

# 생각해 보기: B 에서 기울기가 줄었는데 왜 더 멀리 갔을까? (흔들리면 발이 미끄러져 힘이 헛돈다)
# 실험 1: GAIN 을 0.1 로 줄이면? 2.0 으로 키우면? (너무 세게 고치면 오히려 뒤로 가거나 앞뒤로 출렁인다)
# 실험 2: DAMP 를 0 으로 하면? (자이로 없이 기울기만 쓰기)
# 실험 3: TURN_GAIN 을 0.05, 0.5 로 바꿔 C 의 방향이 어떻게 변하는지 보자.
# 쉬운 방법: 직접 계산하는 대신 dog.set_balance(True) 를 한 번 부르면 set_feet 가 알아서 고친다 (같은 계산).
