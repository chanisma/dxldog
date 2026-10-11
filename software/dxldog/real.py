"""실제 DXL Dog 백엔드 — 모터 12개를 움직인다. 노트북·Raspberry Pi 모두 같은 코드, 모터 종류도 둘 다.

  로봇 종류 "mixed": DYNAMIXEL (dynamixel_sdk, Protocol 2.0, U2D2)          → DxlBus
  로봇 종류 "sts":   Feetech STS3215 (scservo_sdk, URT-1·Waveshare 어댑터)   → StsBus
  하드웨어 없이 시험: port="fake"  → FakeBus: 지금 로봇 종류의 모터를 흉내 내는 가짜 버스

  노트북(Windows):   DxlReal(port="COM3")
  Raspberry Pi:      DxlReal(port="/dev/ttyUSB0")   (STS 어댑터는 /dev/ttyACM0 일 수 있음)

관절각·목표는 '모델 rad' (config.py 규칙). 위치값(틱) = 2048 + 방향 × rad × 4096/2π + 보정값.
버스는 모두 같은 함수(ping, read, write, sync_read, sync_write, write_eeprom, close)를 가지고
레지스터는 config.ADDR 의 이름("goal_position" 등)으로 부른다. 주소·크기·부호 규칙은 버스가 알아서 처리한다.
"""
import json
import time
from pathlib import Path

import numpy as np

from . import config as C

CAL_DIR = Path.home() / ".dxldog"
CAL_FILE = CAL_DIR / "calibration.json"                      # "mixed" 보정값 (예전 이름 그대로)
TEMP_LIMIT = C.TEMP_LIMIT                                    # 모터 모델별 기본 온도 한계 (°C)
TEMP_READ_SEC = 0.5                                          # 온도는 0.5초마다 한 번만 읽는다 (통신 아끼기)


def cal_file(robot=None):
    """로봇 종류별 보정값 파일. mixed → calibration.json, sts → calibration_sts.json"""
    robot = robot or C.ROBOT
    return CAL_FILE if robot == "mixed" else CAL_DIR / f"calibration_{robot}.json"


def sts_encode(value, sign_bit, size):
    """STS 부호 규칙: 음수는 '부호 비트 + 크기'. 예) −5 (비트 15) → 0x8005"""
    value = int(value)
    if sign_bit is not None and value < 0:
        value = (-value) | (1 << sign_bit)
    return value & ((1 << (8 * size)) - 1)


def sts_decode(raw, sign_bit):
    if sign_bit is not None and raw & (1 << sign_bit):
        return -(raw & ~(1 << sign_bit))
    return raw


def _open_port(PortHandler, port, baud):
    ph = PortHandler(port)
    try:
        ok = ph.openPort() and ph.setBaudRate(baud)
    except Exception as e:                                   # 포트가 없거나 다른 프로그램이 쓰는 중
        raise RuntimeError(f"포트 {port} 를 열 수 없어요 ({e}). {C.ADAPTER} 연결과 포트 이름을 확인하세요 "
                           f"({C.ROBOT_INFO['ports']}). 포트 목록: python -m dxldog.tools ports") from None
    if not ok:
        raise RuntimeError(f"포트 {port} 를 {baud} bps 로 열 수 없어요. {C.ADAPTER} 연결과 포트 이름을 확인하세요.")
    return ph


class _Bus:
    """세 버스가 함께 쓰는 기능."""

    def write_eeprom(self, mid, items):
        """EEPROM 값(ID, 위치 한계, 운전 모드 등) 쓰기. 토크를 끈 뒤에 부른다.
        STS 는 Lock(55) 을 0 으로 풀고 쓴 다음 다시 1 로 잠근다. ID 를 바꾸면 새 ID 를 돌려준다."""
        locked = "lock" in C.ADDR
        if locked:
            self.write(mid, "lock", 0)
        try:
            for item, v in items.items():
                if item != "id":
                    self.write(mid, item, v)
            new = int(items.get("id", mid))
            if new != mid:
                try:
                    self.write(mid, "id", new)
                except RuntimeError:                         # 응답이 새 ID 로 오면 못 받을 수 있다 → 새 ID 로 확인
                    if self.ping(new) is None:
                        raise
                mid = new
        finally:
            if locked:
                self.write(mid, "lock", 1)
        return mid


class DxlBus(_Bus):
    """dynamixel_sdk 를 감싼 버스: 같은 주소에 여러 모터 동시 쓰기·읽기. 음수는 2의 보수."""

    def __init__(self, port, baud=None):
        from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler
        self._ok = COMM_SUCCESS
        self._GSW, self._GSR = GroupSyncWrite, GroupSyncRead
        self.port = _open_port(PortHandler, port, baud or C.BAUD)
        self.pk = PacketHandler(2.0)

    def ping(self, dxl_id):
        model, res, err = self.pk.ping(self.port, dxl_id)
        return model if res == self._ok else None

    def write(self, dxl_id, item, value):
        addr, size = C.ADDR[item]
        fn = {1: self.pk.write1ByteTxRx, 2: self.pk.write2ByteTxRx, 4: self.pk.write4ByteTxRx}[size]
        res, err = fn(self.port, dxl_id, addr, int(value) & ((1 << (8 * size)) - 1))
        if res != self._ok:
            raise RuntimeError(f"ID {dxl_id} {item} 쓰기 실패: {self.pk.getTxRxResult(res)}")

    def read(self, dxl_id, item):
        addr, size = C.ADDR[item]
        fn = {1: self.pk.read1ByteTxRx, 2: self.pk.read2ByteTxRx, 4: self.pk.read4ByteTxRx}[size]
        val, res, err = fn(self.port, dxl_id, addr)
        if res != self._ok:
            raise RuntimeError(f"ID {dxl_id} {item} 읽기 실패: {self.pk.getTxRxResult(res)}")
        return self._signed(val, size)

    @staticmethod
    def _signed(v, size):
        if size == 4 and v >= 2 ** 31:
            return v - 2 ** 32
        if size == 2 and v >= 2 ** 15:
            return v - 2 ** 16
        return v

    def sync_write(self, item, values):
        addr, size = C.ADDR[item]
        g = self._GSW(self.port, self.pk, addr, size)
        for dxl_id, v in values.items():
            v = int(v) & 0xFFFFFFFF
            g.addParam(dxl_id, list(int(v).to_bytes(4, "little"))[:size])
        g.txPacket()

    def sync_read(self, item, ids):
        addr, size = C.ADDR[item]
        g = self._GSR(self.port, self.pk, addr, size)
        for dxl_id in ids:
            g.addParam(dxl_id)
        res = g.txRxPacket()
        if res != self._ok:
            raise RuntimeError(f"{item} 동시 읽기 실패: {self.pk.getTxRxResult(res)}")
        return {dxl_id: self._signed(g.getData(dxl_id, addr, size), size) for dxl_id in ids}

    def close(self):
        self.port.closePort()


class StsBus(_Bus):
    """scservo_sdk(feetech-servo-sdk) 를 감싼 Feetech STS 버스. DxlBus 와 같은 함수.
    2바이트 값은 리틀 엔디언(protocol_end 0), 음수는 '부호 비트 + 크기' (config.STS_SIGN_BIT)."""

    def __init__(self, port, baud=None, port_handler=None):
        import scservo_sdk as scs
        self._ok = scs.COMM_SUCCESS
        self._GSW, self._GSR = scs.GroupSyncWrite, scs.GroupSyncRead
        self.port = port_handler or _open_port(scs.PortHandler, port, baud or C.BAUD)   # port_handler: 시험용
        self.pk = scs.PacketHandler(0)                       # 0 = STS/SMS (리틀 엔디언)

    def _encode(self, item, value):
        return sts_encode(value, C.SIGN_BIT.get(item), C.ADDR[item][1])

    def _decode(self, item, raw):
        return sts_decode(raw, C.SIGN_BIT.get(item))

    def ping(self, sid):
        model, res, err = self.pk.ping(self.port, sid)
        return model if res == self._ok else None

    def write(self, sid, item, value):
        addr, size = C.ADDR[item]
        fn = {1: self.pk.write1ByteTxRx, 2: self.pk.write2ByteTxRx}[size]
        res, err = fn(self.port, sid, addr, self._encode(item, value))
        if res != self._ok:
            raise RuntimeError(f"ID {sid} {item} 쓰기 실패: {self.pk.getTxRxResult(res)}")

    def read(self, sid, item):
        addr, size = C.ADDR[item]
        fn = {1: self.pk.read1ByteTxRx, 2: self.pk.read2ByteTxRx}[size]
        val, res, err = fn(self.port, sid, addr)
        if res != self._ok:
            raise RuntimeError(f"ID {sid} {item} 읽기 실패: {self.pk.getTxRxResult(res)}")
        return self._decode(item, val)

    def sync_write(self, item, values):
        addr, size = C.ADDR[item]
        g = self._GSW(self.port, self.pk, addr, size)
        for sid, v in values.items():
            g.addParam(sid, list(self._encode(item, v).to_bytes(size, "little")))
        g.txPacket()

    def sync_read(self, item, ids):
        addr, size = C.ADDR[item]
        g = self._GSR(self.port, self.pk, addr, size)
        for sid in ids:
            g.addParam(sid)
        res = g.txRxPacket()
        if res != self._ok:
            raise RuntimeError(f"{item} 동시 읽기 실패: {self.pk.getTxRxResult(res)}")
        return {sid: self._decode(item, g.getData(sid, addr, size)) for sid in ids}

    def close(self):
        self.port.closePort()


class FakeBus(_Bus):
    """모터 12개를 흉내 내는 가짜 버스 (하드웨어 없이 실물 코드 경로 시험용). 지금 로봇 종류의 레지스터를 가진다.
    목표 위치를 향해 속도 레지스터(0 이면 모델의 무부하 속도)만큼 움직이고, 온도는 25 °C 고정.
    실물처럼 EEPROM 은 DYNAMIXEL 은 토크가 꺼졌을 때만, STS 는 Lock 을 풀었을 때만 쓸 수 있다."""

    def __init__(self, *_):
        self.reg = {}
        for (leg, j), i in C.MOTOR_IDS.items():
            model = C.MOTOR_MODEL[j]
            r = {k: 0 for k in C.ADDR}
            r.update(model_number=C.MODEL_NUMBER[model], id=i, operating_mode=C.POSITION_MODE, baud_rate=C.BAUD_CODE[C.BAUD],
                     goal_position=C.CENTER, present_position=C.CENTER, present_temperature=25,
                     present_input_voltage=round(12.0 / C.VOLT_UNIT), max_position_limit=4095,
                     temperature_limit=int(C.TEMP_LIMIT[model]))
            if "lock" in r:
                r["lock"] = 1                                   # ⚠️ STS 공장 설정의 Lock 값은 미확인 — 여기서는 잠김으로 둔다
            self.reg[i] = r
        self.t = time.perf_counter()
        self.log = []

    def _update(self):
        now = time.perf_counter()
        dt, self.t = now - self.t, now
        for r in self.reg.values():
            if not r["torque_enable"]:
                continue
            rpm = r[C.SPEED_ITEM] * C.SPEED_UNIT_RPM or C.NO_LOAD_RPM[C.model_name(r["model_number"])]   # 0 = 최대 속도
            step = rpm / 60 * C.TICKS_PER_REV * dt
            err = r["goal_position"] - r["present_position"]
            r["present_position"] += int(np.clip(err, -step - 1, step + 1))

    def _check(self, mid, item):
        if item not in C.ADDR:
            raise RuntimeError(f"{C.BRAND} 모터에는 '{item}' 레지스터가 없어요")
        if mid not in self.reg:
            raise RuntimeError(f"ID {mid} {item}: 응답 없음")

    def ping(self, mid):
        return self.reg[mid]["model_number"] if mid in self.reg else None

    def write(self, mid, item, value):
        self._check(mid, item)
        r = self.reg[mid]
        if C.ADDR[item][0] < C.EEPROM_END:
            if C.BUS == "dynamixel" and r["torque_enable"]:
                raise RuntimeError(f"ID {mid}: 토크가 켜진 상태에서는 EEPROM({item})을 바꿀 수 없어요")
            if C.BUS == "feetech" and r["lock"]:
                raise RuntimeError(f"ID {mid}: Lock 이 걸린 상태에서는 EEPROM({item})이 저장되지 않아요 (먼저 lock 0)")
        self.log.append(("write", mid, item, value))
        r[item] = int(value)
        if item == "id" and int(value) != mid:
            self.reg[int(value)] = self.reg.pop(mid)

    def read(self, mid, item):
        self._check(mid, item)
        self._update()
        return self.reg[mid][item]

    def sync_write(self, item, values):
        self.log.append(("sync_write", item, dict(values)))
        for i, v in values.items():
            self._check(i, item)
            self.reg[i][item] = int(v)

    def sync_read(self, item, ids):
        self._update()
        for i in ids:
            self._check(i, item)
        return {i: self.reg[i][item] for i in ids}

    def close(self):
        pass


def open_bus(port, baud=None):
    """지금 로봇 종류에 맞는 버스를 연다. port="fake" 면 가짜 모터."""
    if port == "fake":
        return FakeBus()
    return StsBus(port, baud) if C.BUS == "feetech" else DxlBus(port, baud)


class DxlReal:
    """DXL Dog 실물 백엔드 (DYNAMIXEL·STS 모두). DxlSim 과 같은 이름의 속성·함수를 가진다 (Dog 가 둘을 똑같이 쓴다).
    speed_rpm: 모터 최대 속도 제한 (0 = 제한 없음). robot: 로봇 종류를 바꿔서 연결 ("mixed", "sts")."""

    kind = "real"
    has_imu = False                                              # IMU(BNO055 등)를 달면 True 로 바꾸고 rpy·gyro 를 채운다

    def __init__(self, port="fake", speed_rpm=0, calibration=None, robot=None):
        if robot:
            C.use(robot)
        self.robot = C.ROBOT
        self.bus = open_bus(port)
        self.port_name = port
        self.keys = [(l, j) for l in C.LEGS for j in C.JOINTS]
        self.ids = [C.MOTOR_IDS[k] for k in self.keys]
        self.dir = np.array([C.DIRECTION[k] for k in self.keys], float)
        self.offset = np.zeros(12)
        cf = cal_file()
        cal = calibration or (json.loads(cf.read_text(encoding="utf-8")) if cf.exists() else {})
        for n, k in enumerate(self.keys):
            self.offset[n] = cal.get(f"{k[0]}_{k[1]}", 0)
        lo = np.array([-np.radians(C.JOINT_RANGE_DEG[j][1]) for _, j in self.keys])
        hi = np.array([-np.radians(C.JOINT_RANGE_DEG[j][0]) for _, j in self.keys])
        self.lower, self.upper = np.minimum(lo, hi), np.maximum(lo, hi)
        self.temp_limit = np.array([C.TEMP_LIMIT[C.MOTOR_MODEL[j]] for _, j in self.keys])
        self._temp, self._temp_time = np.full(12, 25.0), 0.0
        self._check_motors()
        speed = int(round(speed_rpm / C.SPEED_UNIT_RPM))
        for i in self.ids:                                     # 위치 제어 모드, 속도 제한, 토크 켜기
            self.bus.write(i, "torque_enable", 0)
            if self.bus.read(i, "operating_mode") != C.POSITION_MODE:     # EEPROM 은 바뀔 때만 쓴다
                self.bus.write_eeprom(i, {"operating_mode": C.POSITION_MODE})
            self.bus.write(i, C.SPEED_ITEM, speed)
        self._target = self.joint_pos                          # 지금 자세를 목표로 잡고 나서 토크를 켠다 (튀지 않게)
        self.set_joint_targets(self._target)
        for i in self.ids:
            self.bus.write(i, "torque_enable", 1)
        self._clock = time.perf_counter()

    def _check_motors(self):
        missing, wrong = [], []
        for (leg, j), i in zip(self.keys, self.ids):
            model = self.bus.ping(i)
            if model is None:
                missing.append(f"{leg}_{j}(ID {i})")
            elif model != C.MODEL_NUMBER[C.MOTOR_MODEL[j]]:
                wrong.append(f"{leg}_{j}(ID {i}: {C.model_name(model)})")
        if missing or wrong:
            raise RuntimeError(f"모터 확인 실패 ({C.BRAND}) — 응답 없음: " + ", ".join(missing or ["없음"])
                               + " / 종류 다름: " + ", ".join(wrong or ["없음"])
                               + f"\n로봇 종류가 맞는지 확인하세요 (지금: {C.ROBOT}). 바꾸기: --robot mixed 또는 --robot sts")

    # 변환
    def rad_to_tick(self, q):
        return np.round(C.CENTER + self.dir * np.asarray(q) * C.RAD_TO_TICK + self.offset).astype(int)

    def tick_to_rad(self, t):
        return (np.asarray(t, float) - C.CENTER - self.offset) / C.RAD_TO_TICK * self.dir

    # 상태
    @property
    def joint_pos(self):
        pos = self.bus.sync_read("present_position", self.ids)
        return self.tick_to_rad([pos[i] for i in self.ids])

    @property
    def q_target(self):
        return self._target.copy()

    @property
    def temperature(self):
        now = time.perf_counter()
        if now - self._temp_time > TEMP_READ_SEC:
            t = self.bus.sync_read("present_temperature", self.ids)
            self._temp, self._temp_time = np.array([t[i] for i in self.ids], float), now
        return self._temp.copy()

    def rpy(self):
        return np.zeros(3)                                     # IMU 를 붙이면 여기서 읽는다

    def gyro(self):
        return np.zeros(3)

    def fallen(self):
        return False

    # 명령
    def set_joint_targets(self, q):
        self._target = np.clip(np.asarray(q, float), self.lower, self.upper)
        ticks = self.rad_to_tick(self._target)
        self.bus.sync_write("goal_position", dict(zip(self.ids, ticks)))

    def step(self, n=1):
        """실물은 실제 시간이 흐른다 (1 스텝 = 1 ms). 정해진 시각까지 기다려서 박자를 일정하게 유지."""
        self._clock += n * 0.001
        lag = self._clock - time.perf_counter()
        if lag > 0:
            time.sleep(lag)
        elif lag < -0.1:                                        # 많이 밀렸으면 (통신 지연 등) 박자를 다시 맞춘다
            self._clock = time.perf_counter()

    def torque_off(self):
        for i in self.ids:
            self.bus.write(i, "torque_enable", 0)

    def close(self, torque_off=True):
        if torque_off:
            self.torque_off()
        self.bus.close()
