"""2. 자세 = 관절 12개 각도의 묶음 — 엎드리기, 서기, 앉기."""
from _setup import Dog

dog = Dog()
dog.reset()
print("엎드린 자세:", dog.get_angles())

dog.move_to("stand", seconds=1.5)          # 1.5초 동안 부드럽게
print("서기  높이 %.3f m" % dog.get_height())

# 강아지처럼 앉기: 앞다리는 조금 굽히고, 뒷다리는 접는다 (다리마다 (hip_x, hip_y, knee))
sit = {"FL": (0, -20, 45), "FR": (0, -20, 45), "RL": (0, -75, 125), "RR": (0, -75, 125)}
dog.move_to(sit, seconds=1.5)
print("앉기  기울기(roll, pitch) =", [round(a, 1) for a in dog.get_tilt()])

dog.move_to("stand", seconds=1.5)
if dog.mode == "sim":                      # ⚠️ 실물에서는 하지 않는다 (모터·부품이 상함)
    dog.move_to("lie", seconds=0.1)        # 너무 빨리 움직이면?
    print("0.1초 만에 엎드리기 → 높이 %.3f m (쾅!)" % dog.get_height())
else:
    dog.move_to("lie", seconds=1.5)

print("모터 온도:", dog.get_temperatures())
# 생각해 보기: seconds 를 0.1 로 하면 실제 로봇에는 어떤 일이 생길까?
