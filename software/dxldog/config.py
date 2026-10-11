"""DXL Dog 설정: 다리·관절 이름, 모터 ID·종류·방향, 관절 범위, DYNAMIXEL 컨트롤 테이블.

학생 각도(도) 규칙 — robotdog 의 JointDog(Lite3) 와 같은 뜻:
  hip_x  옆으로 벌리기: + 이면 발이 로봇의 오른쪽으로 (네 다리 모두)
  hip_y  앞뒤로 흔들기: + 이면 허벅지가 앞으로 (아래 0°, 서 있을 때 약 −67°)
  knee   무릎 굽히기: 값이 클수록 많이 굽힘 (0° = 똑바로, 서 있을 때 약 97°)
모델 관절각(rad) = −radians(학생 각도)   (세 관절 모두. 축: hip_x = +x, hip_y·knee = +y)
모터 위치값 = 2048 + 방향 × 모델 관절각 × 4096/2π + 보정값   (조립: 모든 모터 2048 에서 다리를 똑바로 아래로)
"""
import math

LEGS = ["FL", "FR", "RL", "RR"]
LEG_NAMES = {"FL": "앞왼쪽", "FR": "앞오른쪽", "RL": "뒤왼쪽", "RR": "뒤오른쪽"}
JOINTS = ["hip_x", "hip_y", "knee"]
JOINT_INFO = {
    "hip_x": ("옆으로 벌리기", "+ 값이면 발이 로봇의 오른쪽으로, − 값이면 왼쪽으로 (네 다리 모두 같은 방향)"),
    "hip_y": ("앞뒤로 흔들기", "+ 값이면 허벅지가 앞으로, − 값이면 뒤로 (서 있을 때 약 −67°)"),
    "knee": ("무릎 굽히기", "값이 클수록 무릎을 많이 굽혀 발이 몸 쪽으로 올라옴 (서 있을 때 약 97°)"),
}
JOINT_RANGE_DEG = {"hip_x": (-30.0, 30.0), "hip_y": (-95.0, 20.0), "knee": (0.0, 125.0)}   # CAD 충돌 검사로 확인한 범위

THIGH, SHANK = 0.1075, 0.130          # m (SpotMicro)
HIP_LINK = 0.055

POSES = {   # 다리 하나의 (hip_x, hip_y, knee) [도] — 네 다리 공통 또는 {다리: (...)}
    "zero":  (0.0, 0.0, 0.0),            # 조립 자세: 다리 똑바로 아래 (모든 모터 2048)
    "lie":   (0.0, -69.0, 120.0),        # 쉬기 (몸 높이 약 13 cm)
    "stand": (0.0, -67.1, 96.6),         # 서기: 몸 높이 15.5 cm, 발은 엉덩이보다 3.5 cm 뒤 (무게중심이 뒤쪽이라 균형)
    "sit":   {"FL": (0, -20, 45), "FR": (0, -20, 45), "RL": (0, -75, 125), "RR": (0, -75, 125)},
}

# ---------- 모터 ----------
MOTOR_IDS = {(leg, j): 1 + 3 * li + ji for li, leg in enumerate(LEGS) for ji, j in enumerate(JOINTS)}   # FL 1·2·3 … RR 10·11·12
MOTOR_MODEL = {"hip_x": "XL430-W250", "hip_y": "XM430-W350", "knee": "XM430-W350"}      # 권장(혼합) 구성
MODEL_NUMBER = {"XL430-W250": 1060, "XM430-W350": 1020, "XM430-W210": 1030, "XC430-W240": 1070}
STALL_TORQUE = {"XL430-W250": 1.5, "XM430-W350": 4.1, "XM430-W210": 3.0, "XC430-W240": 1.9}  # Nm @12 V
NO_LOAD_RPM = {"XL430-W250": 57.0, "XM430-W350": 46.0, "XM430-W210": 77.0, "XC430-W240": 70.0}  # rpm @12 V (공식 사양)
# 방향: 모터 + 회전(혼 쪽에서 봐서 반시계) 이 모델 관절 축의 + 와 같으면 +1.
#   hip_x: 네 다리 모두 혼이 +x 쪽 → +1.   hip_y·knee: 왼쪽 다리 혼 +y → +1, 오른쪽(거울) 혼 −y → −1
DIRECTION = {(leg, j): (1 if j == "hip_x" or leg[1] == "L" else -1) for leg in LEGS for j in JOINTS}

TICKS_PER_REV = 4096
CENTER = 2048
RAD_TO_TICK = TICKS_PER_REV / (2 * math.pi)

# ---------- DYNAMIXEL Protocol 2.0 컨트롤 테이블 (X 시리즈) ----------
BAUD = 1_000_000
ADDR = {
    "model_number": (0, 2), "id": (7, 1), "baud_rate": (8, 1), "return_delay_time": (9, 1), "operating_mode": (11, 1),
    "temperature_limit": (31, 1), "max_position_limit": (48, 4), "min_position_limit": (52, 4),
    "torque_enable": (64, 1), "led": (65, 1), "hardware_error": (70, 1),
    "position_p_gain": (84, 2), "profile_acceleration": (108, 4), "profile_velocity": (112, 4),
    "goal_position": (116, 4), "present_current": (126, 2), "present_position": (132, 4),
    "present_input_voltage": (144, 2), "present_temperature": (146, 1),
}
POSITION_MODE = 3
BAUD_CODE = {9600: 0, 57600: 1, 115200: 2, 1_000_000: 3, 2_000_000: 4}   # baud_rate 레지스터 값
FACTORY_BAUD = 57600                    # 새 모터 공장 설정 (ID 1, 57600 bps)
