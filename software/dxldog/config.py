"""DXL Dog 설정: 다리·관절 이름, 모터 ID·종류·방향, 관절 범위, 로봇 종류별 모터와 컨트롤 테이블.

학생 각도(도) 규칙 — robotdog 의 JointDog(Lite3) 와 같은 뜻:
  hip_x  옆으로 벌리기: + 이면 발이 로봇의 오른쪽으로 (네 다리 모두)
  hip_y  앞뒤로 흔들기: + 이면 허벅지가 앞으로 (아래 0°, 서 있을 때 약 −67°)
  knee   무릎 굽히기: 값이 클수록 많이 굽힘 (0° = 똑바로, 서 있을 때 약 97°)
모델 관절각(rad) = −radians(학생 각도)   (세 관절 모두. 축: hip_x = +x, hip_y·knee = +y)
모터 위치값 = 2048 + 방향 × 모델 관절각 × 4096/2π + 보정값   (조립: 모든 모터 2048 에서 다리를 똑바로 아래로)

로봇 종류 (같은 몸, 다른 모터) — 학생 코드는 똑같고 종류만 고른다:
  "mixed"  DYNAMIXEL: hip_x XL430-W250, hip_y·knee XM430-W350 (U2D2, Protocol 2.0)
  "sts"    Feetech STS3215 12개 (LeRobot SO-101 의 모터, URT-1 또는 Waveshare 버스 서보 어댑터)
처음 종류: 환경 변수 DXLDOG_ROBOT → ~/.dxldog/robot.json {"robot": ...} → "mixed".
바꾸기: config.use("sts") 또는 Dog(robot="sts"), 기본값 저장: python -m dxldog.tools use sts
다른 모듈은 `from . import config as C` 로 불러 C.MOTOR_MODEL, C.ADDR 처럼 '쓸 때' 읽는다 → use() 가 바로 반영된다.
"""
import json
import math
import os
from pathlib import Path

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

# ---------- 모터 사양 (모든 종류) ----------
# 번호: 모델 번호 레지스터 값.  stall: 정지 토크 Nm.  rpm: 무부하 속도.  mass: kg.  temp: 기본 온도 한계 °C
MOTORS = {
    "XL430-W250": {"number": 1060, "stall": 1.5, "rpm": 57.0, "mass": 0.065, "temp": 72.0},   # @12 V (공식 사양)
    "XM430-W350": {"number": 1020, "stall": 4.1, "rpm": 46.0, "mass": 0.082, "temp": 80.0},
    "XM430-W210": {"number": 1030, "stall": 3.0, "rpm": 77.0, "mass": 0.082, "temp": 80.0},
    "XC430-W240": {"number": 1070, "stall": 1.9, "rpm": 70.0, "mass": 0.065, "temp": 80.0},
    # STS3215 12 V 판: 30 kg·cm = 2.94 Nm @12 V, 0.222 s/60° ≈ 45 rpm, 55 g. 모델 번호 777 은 LeRobot 표의 값
    # ⚠️ 온도 한계 70 °C 는 Feetech 기본값으로 알려진 값 (실물에서 Max Temperature 레지스터를 읽어 확인)
    "STS3215-12V": {"number": 777, "stall": 2.94, "rpm": 45.0, "mass": 0.055, "temp": 70.0},
    # 7.4 V 판 (참고용): 19.5 kg·cm ≈ 1.91 Nm, 약 52 rpm ⚠️ 판매처 값. 모델 번호가 12 V 판과 같은지 ⚠️ 미확인
    "STS3215-7.4V": {"number": 777, "stall": 1.91, "rpm": 52.0, "mass": 0.055, "temp": 70.0},
}
MODEL_NUMBER = {k: v["number"] for k, v in MOTORS.items()}
STALL_TORQUE = {k: v["stall"] for k, v in MOTORS.items()}
NO_LOAD_RPM = {k: v["rpm"] for k, v in MOTORS.items()}
MOTOR_MASS = {k: v["mass"] for k, v in MOTORS.items()}
TEMP_LIMIT = {k: v["temp"] for k, v in MOTORS.items()}

# ---------- 컨트롤 테이블: (주소, 바이트 수) ----------
# DYNAMIXEL Protocol 2.0 (X 시리즈). 음수는 2의 보수.
DXL_ADDR = {
    "model_number": (0, 2), "id": (7, 1), "baud_rate": (8, 1), "return_delay_time": (9, 1), "operating_mode": (11, 1),
    "temperature_limit": (31, 1), "max_position_limit": (48, 4), "min_position_limit": (52, 4),
    "torque_enable": (64, 1), "led": (65, 1), "hardware_error": (70, 1),
    "position_p_gain": (84, 2), "profile_acceleration": (108, 4), "profile_velocity": (112, 4),
    "goal_position": (116, 4), "present_current": (126, 2), "present_position": (132, 4),
    "present_input_voltage": (144, 2), "present_temperature": (146, 1),
}
# Feetech STS (SMS_STS) 시리즈. 0–39 EEPROM, 40– RAM. 2바이트 값은 리틀 엔디언(protocol_end 0).
# 주소는 Feetech SCServo 라이브러리(SMS_STS.h)·LeRobot feetech 표와 같다.
STS_ADDR = {
    "model_number": (3, 2), "id": (5, 1), "baud_rate": (6, 1), "return_delay_time": (7, 1),
    "min_position_limit": (9, 2), "max_position_limit": (11, 2), "temperature_limit": (13, 1),
    "p_gain": (21, 1), "d_gain": (22, 1), "i_gain": (23, 1),
    "homing_offset": (31, 2), "operating_mode": (33, 1),
    "torque_enable": (40, 1), "acceleration": (41, 1), "goal_position": (42, 2), "goal_time": (44, 2),
    "goal_speed": (46, 2), "torque_limit": (48, 2), "lock": (55, 1),
    "present_position": (56, 2), "present_speed": (58, 2), "present_load": (60, 2),
    "present_input_voltage": (62, 1), "present_temperature": (63, 1), "moving": (66, 1), "present_current": (69, 2),
}
# STS 는 음수를 '부호 비트 + 크기'로 적는다: {이름: 부호 비트 번호}. SCServo 라이브러리의 ReadPos/WritePosEx 등과 같음.
# ⚠️ present_load(비트 10)·present_current(비트 15) 는 라이브러리 코드 기준, 실물 미확인
STS_SIGN_BIT = {"homing_offset": 11, "goal_position": 15, "present_position": 15, "goal_speed": 15,
                "present_speed": 15, "present_load": 10, "present_current": 15}

# ---------- 로봇 종류 ----------
ROBOTS = {
    "mixed": {
        "name": "DYNAMIXEL X430 (혼합)", "brand": "DYNAMIXEL", "bus": "dynamixel",
        "adapter": "U2D2", "ports": "Windows COM3 등, Raspberry Pi /dev/ttyUSB0",
        "motor_model": {"hip_x": "XL430-W250", "hip_y": "XM430-W350", "knee": "XM430-W350"},
        "addr": DXL_ADDR, "sign_bit": {}, "eeprom_end": 64,          # 64 미만은 EEPROM (토크를 끈 뒤에만 쓰기)
        "position_mode": 3,
        "baud": 1_000_000, "factory_baud": 57600, "factory_id": 1,
        "baud_code": {9600: 0, 57600: 1, 115200: 2, 1_000_000: 3, 2_000_000: 4},
        "speed_item": "profile_velocity", "speed_unit_rpm": 0.229,   # 0 = 최대 속도
        "volt_unit": 0.1,
        "dir_sign": {"hip_x": 1, "hip_y": 1, "knee": 1},
    },
    "sts": {
        "name": "Feetech STS3215", "brand": "STS3215", "bus": "feetech",
        "adapter": "Feetech URT-1 또는 Waveshare 버스 서보 어댑터",
        "ports": "Windows COM5 등, Raspberry Pi /dev/ttyACM0 또는 /dev/ttyUSB0",
        "motor_model": {"hip_x": "STS3215-12V", "hip_y": "STS3215-12V", "knee": "STS3215-12V"},
        "addr": STS_ADDR, "sign_bit": STS_SIGN_BIT, "eeprom_end": 40,  # 40 미만은 EEPROM (Lock 을 0 으로 풀고 쓴다)
        "position_mode": 0,
        "baud": 1_000_000, "factory_baud": 1_000_000, "factory_id": 1,
        # SCServo 라이브러리 SMS_STS.h 값. ⚠️ LeRobot 표는 5=57600, 6=38400, 7=19200 으로 다르다 (1 Mbps=0 은 같음)
        "baud_code": {1_000_000: 0, 500_000: 1, 250_000: 2, 128_000: 3, 115_200: 4, 76_800: 5, 57_600: 6, 38_400: 7},
        "speed_item": "goal_speed", "speed_unit_rpm": 60 / 4096,     # ⚠️ 단위 = 걸음/초 (1 걸음 = 1/4096 바퀴), 0 = 최대 속도 ⚠️
        "volt_unit": 0.1,
        # ⚠️ STS 의 + 회전 방향이 DYNAMIXEL 과 같은지 미확인 → 조립 후 `tools check --robot sts` 로 확인하고 고친다
        "dir_sign": {"hip_x": 1, "hip_y": 1, "knee": 1},
    },
}

MOTOR_IDS = {(leg, j): 1 + 3 * li + ji for li, leg in enumerate(LEGS) for ji, j in enumerate(JOINTS)}   # FL 1·2·3 … RR 10·11·12

TICKS_PER_REV = 4096
CENTER = 2048
RAD_TO_TICK = TICKS_PER_REV / (2 * math.pi)

ROBOT_FILE = Path.home() / ".dxldog" / "robot.json"

# 아래 이름들은 use() 가 지금 로봇 종류에 맞게 '그 자리에서' 바꾼다 (다른 모듈이 같은 dict 를 계속 쓸 수 있게)
ROBOT = None
ROBOT_INFO = {}
MOTOR_MODEL = {}
ADDR = {}
SIGN_BIT = {}
DIRECTION = {}
BAUD_CODE = {}
BUS = BRAND = ADAPTER = SPEED_ITEM = None
BAUD = FACTORY_BAUD = FACTORY_ID = POSITION_MODE = EEPROM_END = None
SPEED_UNIT_RPM = VOLT_UNIT = None


def use(name):
    """지금 쓰는 로봇 종류를 바꾼다 ("mixed" 또는 "sts"). 모터를 연결하거나 시뮬레이터를 만들기 '전에' 부른다."""
    global ROBOT, BUS, BRAND, ADAPTER, SPEED_ITEM, BAUD, FACTORY_BAUD, FACTORY_ID, POSITION_MODE, EEPROM_END
    global SPEED_UNIT_RPM, VOLT_UNIT
    if name not in ROBOTS:
        raise ValueError(f"로봇 종류 '{name}' 은(는) 없어요. 쓸 수 있는 종류: {', '.join(ROBOTS)}")
    r = ROBOTS[name]
    ROBOT = name
    ROBOT_INFO.clear(); ROBOT_INFO.update(r)
    MOTOR_MODEL.clear(); MOTOR_MODEL.update(r["motor_model"])
    ADDR.clear(); ADDR.update(r["addr"])
    SIGN_BIT.clear(); SIGN_BIT.update(r["sign_bit"])
    BAUD_CODE.clear(); BAUD_CODE.update(r["baud_code"])
    # 방향: 모터 + 회전(혼 쪽에서 봐서 반시계) 이 모델 관절 축의 + 와 같으면 +1.
    #   hip_x: 네 다리 모두 혼이 +x 쪽 → +1.   hip_y·knee: 왼쪽 다리 혼 +y → +1, 오른쪽(거울) 혼 −y → −1
    #   × 로봇 종류별 dir_sign (모터 회사마다 + 회전 방향이 다를 때)
    DIRECTION.clear()
    DIRECTION.update({(leg, j): (1 if j == "hip_x" or leg[1] == "L" else -1) * r["dir_sign"][j] for leg in LEGS for j in JOINTS})
    BUS, BRAND, ADAPTER, SPEED_ITEM = r["bus"], r["brand"], r["adapter"], r["speed_item"]
    BAUD, FACTORY_BAUD, FACTORY_ID = r["baud"], r["factory_baud"], r["factory_id"]
    POSITION_MODE, EEPROM_END = r["position_mode"], r["eeprom_end"]
    SPEED_UNIT_RPM, VOLT_UNIT = r["speed_unit_rpm"], r["volt_unit"]
    return name


def saved_robot():
    """~/.dxldog/robot.json 에 저장된 기본 로봇 종류 (없으면 None)."""
    try:
        return json.loads(ROBOT_FILE.read_text(encoding="utf-8")).get("robot")
    except (OSError, ValueError):
        return None


def save_default(name):
    """기본 로봇 종류를 ~/.dxldog/robot.json 에 저장하고 지금 종류도 바꾼다."""
    use(name)
    ROBOT_FILE.parent.mkdir(parents=True, exist_ok=True)
    ROBOT_FILE.write_text(json.dumps({"robot": name}), encoding="utf-8")
    return ROBOT_FILE


def model_name(number):
    """모델 번호 → 이름. 지금 로봇 종류의 모터를 먼저 고른다 (STS3215 12 V·7.4 V 는 번호가 같을 수 있음)."""
    for m in list(MOTOR_MODEL.values()) + list(MOTORS):
        if MODEL_NUMBER[m] == number:
            return m
    return str(number)


use(os.environ.get("DXLDOG_ROBOT") or saved_robot() or "mixed")
