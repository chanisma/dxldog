"""실제 DXL Dog 백엔드 — DYNAMIXEL SDK(Protocol 2.0)로 모터 12개를 움직인다. 노트북·Raspberry Pi 모두 같은 코드.

  노트북(Windows):   DxlReal(port="COM3")
  Raspberry Pi:      DxlReal(port="/dev/ttyUSB0")
  하드웨어 없이 시험: DxlReal(port="fake")      ← FakeBus: 모터를 흉내 내는 가짜 버스

관절각·목표는 '모델 rad' (config.py 규칙). 위치값(틱) = 2048 + 방향 × rad × 4096/2π + 보정값.
"""
import json
import time
from pathlib import Path

import numpy as np

from . import config as C

CAL_FILE = Path.home() / ".dxldog" / "calibration.json"     # 보정값 (calibrate.py 가 만든다)
TEMP_LIMIT = {"XL430-W250": 72.0, "XM430-W350": 80.0}        # DYNAMIXEL 기본 Temperature Limit (°C)
TEMP_READ_SEC = 0.5                                          # 온도는 0.5초마다 한 번만 읽는다 (통신 아끼기)


class DxlBus:
    """dynamixel_sdk 를 감싼 버스: 같은 주소에 여러 모터 동시 쓰기·읽기."""

    def __init__(self, port, baud=C.BAUD):
        from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler
        self._ok = COMM_SUCCESS
        self._GSW, self._GSR = GroupSyncWrite, GroupSyncRead
        self.port = PortHandler(port)
        if not self.port.openPort():
            raise RuntimeError(f"포트 {port} 를 열 수 없어요. U2D2 연결과 포트 이름을 확인하세요 (Windows: COM?, Pi: /dev/ttyUSB0)")
        if not self.port.setBaudRate(baud):
            raise RuntimeError(f"통신 속도 {baud} 설정 실패")
        self.pk = PacketHandler(2.0)

    def ping(self, dxl_id):
        model, res, err = self.pk.ping(self.port, dxl_id)
        return model if res == self._ok else None

    def write(self, dxl_id, item, value):
        addr, size = C.ADDR[item]
        fn = {1: self.pk.write1ByteTxRx, 2: self.pk.write2ByteTxRx, 4: self.pk.write4ByteTxRx}[size]
        res, err = fn(self.port, dxl_id, addr, int(value))
        if res != self._ok:
            raise RuntimeError(f"ID {dxl_id} {item} 쓰기 실패: {self.pk.getTxRxResult(res)}")

    def read(self, dxl_id, item):
        addr, size = C.ADDR[item]
        fn = {1: self.pk.read1ByteTxRx, 2: self.pk.read2ByteTxRx, 4: self.pk.read4ByteTxRx}[size]
        val, res, err = fn(self.port, dxl_id, addr)
        if res != self._ok:
            raise RuntimeError(f"ID {dxl_id} {item} 읽기 실패: {self.pk.getTxRxResult(res)}")
        return val

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
        out = {}
        for dxl_id in ids:
            v = g.getData(dxl_id, addr, size)
            if size == 4 and v >= 2 ** 31:
                v -= 2 ** 32
            elif size == 2 and v >= 2 ** 15:
                v -= 2 ** 16
            out[dxl_id] = v
        return out

    def close(self):
        self.port.closePort()


class FakeBus:
    """모터 12개를 흉내 내는 가짜 버스 (하드웨어 없이 실물 코드 경로 시험용).
    목표 위치를 향해 Profile Velocity 만큼의 속도로 움직이고, 온도는 25 °C 고정."""

    def __init__(self, *_):
        ids = list(C.MOTOR_IDS.values())
        self.reg = {i: {"model_number": C.MODEL_NUMBER[C.MOTOR_MODEL[j]], "torque_enable": 0, "operating_mode": 3,
                        "goal_position": C.CENTER, "present_position": C.CENTER, "profile_velocity": 0,
                        "present_temperature": 25, "present_input_voltage": 120, "hardware_error": 0,
                        "profile_acceleration": 0, "position_p_gain": 800, "led": 0, "present_current": 0}
                    for (leg, j), i in C.MOTOR_IDS.items()}
        self.t = time.perf_counter()
        self.log = []

    def _update(self):
        now = time.perf_counter()
        dt, self.t = now - self.t, now
        for r in self.reg.values():
            if not r["torque_enable"]:
                continue
            vel = r["profile_velocity"] or 1000                 # 0 = 최대 속도
            step = vel * 0.229 / 60 * 4096 * dt                 # 0.229 rpm 단위
            err = r["goal_position"] - r["present_position"]
            r["present_position"] += int(np.clip(err, -step - 1, step + 1))

    def ping(self, dxl_id):
        return self.reg[dxl_id]["model_number"] if dxl_id in self.reg else None

    def write(self, dxl_id, item, value):
        self.log.append(("write", dxl_id, item, value))
        if item == "operating_mode" and self.reg[dxl_id]["torque_enable"]:
            raise RuntimeError("토크가 켜진 상태에서는 운전 모드를 바꿀 수 없어요")
        self.reg[dxl_id][item] = int(value)

    def read(self, dxl_id, item):
        self._update()
        return self.reg[dxl_id][item]

    def sync_write(self, item, values):
        self.log.append(("sync_write", item, dict(values)))
        for i, v in values.items():
            self.reg[i][item] = int(v)

    def sync_read(self, item, ids):
        self._update()
        return {i: self.reg[i][item] for i in ids}

    def close(self):
        pass


class DxlReal:
    """실물 백엔드. DxlSim 과 같은 이름의 속성·함수를 가진다 (Dog 가 둘을 똑같이 쓴다)."""

    kind = "real"

    def __init__(self, port="fake", profile_velocity=0, calibration=None):
        self.bus = FakeBus() if port == "fake" else DxlBus(port)
        self.port_name = port
        self.keys = [(l, j) for l in C.LEGS for j in C.JOINTS]
        self.ids = [C.MOTOR_IDS[k] for k in self.keys]
        self.dir = np.array([C.DIRECTION[k] for k in self.keys], float)
        self.offset = np.zeros(12)
        cal = calibration or (json.loads(CAL_FILE.read_text(encoding="utf-8")) if CAL_FILE.exists() else {})
        for n, k in enumerate(self.keys):
            self.offset[n] = cal.get(f"{k[0]}_{k[1]}", 0)
        lo = np.array([-np.radians(C.JOINT_RANGE_DEG[j][1]) for _, j in self.keys])
        hi = np.array([-np.radians(C.JOINT_RANGE_DEG[j][0]) for _, j in self.keys])
        self.lower, self.upper = np.minimum(lo, hi), np.maximum(lo, hi)
        self.temp_limit = np.array([TEMP_LIMIT[C.MOTOR_MODEL[j]] for _, j in self.keys])
        self._temp, self._temp_time = np.full(12, 25.0), 0.0
        self._check_motors()
        for i in self.ids:                                     # 위치 제어 모드, 속도 제한, 토크 켜기
            self.bus.write(i, "torque_enable", 0)
            self.bus.write(i, "operating_mode", C.POSITION_MODE)
            self.bus.write(i, "profile_velocity", profile_velocity)
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
                wrong.append(f"{leg}_{j}(ID {i}: 모델 {model})")
        if missing or wrong:
            raise RuntimeError("모터 확인 실패 — 응답 없음: " + ", ".join(missing or ["없음"]) + " / 종류 다름: " + ", ".join(wrong or ["없음"]))

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
