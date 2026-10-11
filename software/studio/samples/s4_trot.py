"""4. 걸음새 직접 만들기 — 트롯(trot): 대각선 두 다리씩 번갈아 딛기.

한 다리의 한 주기(T초) = 딛기(stance, 발을 땅에 대고 뒤로 밀기) + 옮기기(swing, 발을 들어 앞으로)
FL·RR 이 한 짝, FR·RL 이 한 짝이고, 두 짝은 반 주기 차이로 움직인다.
⚠️ 실물: 처음에는 로봇을 받침대에 올려 발이 바닥에 닿지 않게 하고 시험한다.
DUTY = 딛는 시간의 비율. 0.5 보다 크면 네 발이 모두 땅에 닿는 순간이 생겨 덜 흔들린다.
"""
import math

from dxldog import Dog

T = 0.5          # 한 주기 시간 (초)
STEP = 0.04      # 한 번 딛는 동안 발이 미는 거리 (m)
LIFT = 0.03      # 발 드는 높이 (m)
HEIGHT = -0.155  # 딛는 발의 높이 (m)
FOOT_X = -0.035  # 발의 앞뒤 기준 위치 (m) — 서기 자세와 같음. 무게중심이 엉덩이보다 뒤에 있어서 발도 뒤에 둔다
DUTY = 0.65      # 딛는 시간 비율
PHASE = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}   # 대각선 짝


def foot_at(p):
    """주기 안의 위치 p(0~1) → 발끝 (x, z)"""
    if p < DUTY:                                # 딛기: 앞(+STEP/2) → 뒤(−STEP/2), 일정한 속도
        return FOOT_X + STEP / 2 - STEP * (p / DUTY), HEIGHT
    s = (p - DUTY) / (1 - DUTY)                 # 옮기기: 0 → 1
    x = -STEP / 2 + STEP * (1 - math.cos(math.pi * s)) / 2      # 천천히 출발해서 천천히 도착
    z = HEIGHT + LIFT * math.sin(math.pi * s)                   # 반원 모양으로 들었다 내림
    return FOOT_X + x, z


dog = Dog()
dog.move_to("stand", 1.5)
start = dog.get_position() if dog.mode == "sim" else None

t = 0.0
while t < 10 * T:                               # 10 주기
    feet = {leg: foot_at((t / T + phase) % 1.0) for leg, phase in PHASE.items()}
    dog.set_feet(feet, seconds=0)               # 매 순간 목표 위치 지정
    dog.wait(0.01)
    t += 0.01

dog.move_to("stand", 0.5)
if start:
    x1, y1 = dog.get_position()
    print(f"10 주기 동안 이동: {x1 - start[0]:+.2f} m (어림값 {10 * STEP / DUTY:.2f} m), 방향 {dog.get_heading():+.0f}°")
    print("기울기(roll, pitch):", [round(a, 1) for a in dog.get_tilt()])
print("모터 온도:", dog.get_temperatures())
dog.move_to("lie", 1.5)

# 실험 1: DUTY 를 0.5 로 바꾸면? (발이 미끄러져 절반밖에 못 간다 — 두 발로만 서는 시간이 길다)
# 실험 2: FOOT_X 를 0 으로 바꾸면? (앞발이 땅을 잘 못 딛는다 — 무게중심이 뒤에 있어서)
# 실험 3: T 를 0.3 으로 줄이면? 모터가 따라가지 못한다 (모터에는 최대 속도가 있다: XM430 약 46 rpm)
# 도전: 오른쪽 다리 STEP 만 크게 하면? → 왼쪽으로 돈다! (dxldog.trot_feet 의 turn 참고)
