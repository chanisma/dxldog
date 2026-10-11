"""StsBus·FakeBus 시험 (하드웨어 없이). 실행: python tests/test_buses.py   (pytest 로도 돈다)

MockStsPort 는 scservo_sdk 의 PortHandler 자리에 끼우는 '가짜 시리얼 포트'다. 보낸 바이트를 해석해서
STS 모터처럼 상태 패킷을 돌려준다 → StsBus → scservo_sdk → 바이트 까지 실제 코드 경로를 시험한다.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dxldog import config as C  # noqa: E402
from dxldog import real  # noqa: E402

PING, READ, WRITE, SYNC_READ, SYNC_WRITE = 1, 2, 3, 0x82, 0x83


class MockStsPort:
    """STS 모터 여러 개가 달린 버스 흉내. motors: {id: bytearray(256) 레지스터}."""

    def __init__(self, ids=(1,), model=777):
        self.motors = {}
        for i in ids:
            r = bytearray(256)
            r[3:5] = model.to_bytes(2, "little")
            r[5] = i
            r[55] = 1                                         # Lock
            r[56:58] = (2048).to_bytes(2, "little")
            r[63] = 25
            self.motors[i] = r
        self.is_using = False
        self.rx = bytearray()
        self.sent = []                                        # 보낸 패킷 (바이트)
        self.closed = False

    # PortHandler 와 같은 이름
    def clearPort(self):
        pass

    def setPacketTimeout(self, n):
        pass

    def setPacketTimeoutMillis(self, ms):
        pass

    def isPacketTimeout(self):
        return True                                           # 받을 바이트가 더 없으면 바로 시간 초과

    def readPort(self, n):
        out, self.rx = bytes(self.rx[:n]), self.rx[n:]
        return out

    def closePort(self):
        self.closed = True

    def _status(self, sid, params=b"", err=0):
        body = bytes([sid, len(params) + 2, err]) + bytes(params)
        self.rx += b"\xff\xff" + body + bytes([~sum(body) & 0xFF])

    def writePort(self, packet):
        pkt = bytes(packet)
        self.sent.append(pkt)
        assert pkt[:2] == b"\xff\xff"
        sid, length, inst = pkt[2], pkt[3], pkt[4]
        assert len(pkt) == length + 4
        assert pkt[-1] == ~sum(pkt[2:-1]) & 0xFF, "체크섬"
        p = pkt[5:-1]
        if inst == PING and sid in self.motors:
            self._status(sid)
        elif inst == READ and sid in self.motors:
            self._status(sid, self.motors[sid][p[0]:p[0] + p[1]])
        elif inst == WRITE and sid in self.motors:
            r = self.motors[sid]
            r[p[0]:p[0] + len(p) - 1] = p[1:]
            self._status(sid)                                 # 응답은 예전 ID 로
            if p[0] == 5 and p[1] != sid:                      # ID 바꾸기
                self.motors[p[1]] = self.motors.pop(sid)
        elif inst == SYNC_WRITE:
            addr, n = p[0], p[1]
            for k in range(2, len(p), n + 1):
                if p[k] in self.motors:
                    self.motors[p[k]][addr:addr + n] = p[k + 1:k + 1 + n]
        elif inst == SYNC_READ:
            addr, n = p[0], p[1]
            for i in p[2:]:
                if i in self.motors:
                    self._status(i, self.motors[i][addr:addr + n])
        return len(pkt)


def sts_bus(port):
    C.use("sts")
    return real.StsBus("mock", port_handler=port)


def test_sign_encoding():
    assert real.sts_encode(2048, 15, 2) == 2048
    assert real.sts_encode(-5, 15, 2) == 0x8005
    assert real.sts_decode(0x8005, 15) == -5
    assert real.sts_encode(-100, 11, 2) == 100 | (1 << 11)            # homing_offset: 비트 11
    assert real.sts_decode(100 | (1 << 11), 11) == -100
    assert real.sts_decode(0x8005, None) == 0x8005                    # 부호 없는 레지스터
    for v in (-4095, -1, 0, 1, 4095):
        assert real.sts_decode(real.sts_encode(v, 15, 2), 15) == v


def test_sync_write_bytes():
    port = MockStsPort(ids=(1, 2))
    bus = sts_bus(port)
    bus.sync_write("goal_position", {1: 2048, 2: -5})
    pkt = port.sent[-1]
    # FF FF FE 길이 83 | 주소 42 길이 2 | ID 1: 00 08 | ID 2: 05 80 | 체크섬
    assert pkt[:5] == bytes([0xFF, 0xFF, 0xFE, 2 * 3 + 4, 0x83])
    assert pkt[5:-1] == bytes([42, 2, 1, 0x00, 0x08, 2, 0x05, 0x80])
    assert port.motors[1][42:44] == bytes([0x00, 0x08])


def test_read_write_ping_sync_read():
    port = MockStsPort(ids=(1, 2, 3))
    bus = sts_bus(port)
    assert bus.ping(2) == 777 and bus.ping(9) is None
    bus.write(2, "goal_speed", 478)
    assert port.motors[2][46:48] == (478).to_bytes(2, "little")
    assert bus.read(2, "goal_speed") == 478
    port.motors[3][56:58] = (0x8000 | 7).to_bytes(2, "little")      # 현재 위치 −7 (부호 비트)
    pos = bus.sync_read("present_position", [1, 2, 3])
    assert pos == {1: 2048, 2: 2048, 3: -7}
    assert bus.sync_read("present_temperature", [1, 3]) == {1: 25, 3: 25}
    assert bus.read(1, "present_temperature") == 25
    try:
        bus.read(9, "present_position")
        raise AssertionError("없는 ID 는 오류여야 함")
    except RuntimeError:
        pass


def test_eeprom_lock_and_id():
    port = MockStsPort(ids=(1,))
    bus = sts_bus(port)
    new = bus.write_eeprom(1, {"return_delay_time": 0, "min_position_limit": 1000, "max_position_limit": 3000, "id": 7})
    assert new == 7 and 7 in port.motors and 1 not in port.motors
    r = port.motors[7]
    assert r[9:11] == (1000).to_bytes(2, "little") and r[11:13] == (3000).to_bytes(2, "little")
    assert r[55] == 1                                                  # 다시 잠김
    writes = [(p[2], p[5], p[6]) for p in port.sent if p[4] == WRITE]  # (ID, 주소, 첫 값)
    assert writes[0] == (1, 55, 0)                                     # 먼저 Lock 풀기
    assert writes[-2] == (1, 5, 7)                                     # ID 는 마지막에
    assert writes[-1] == (7, 55, 1)                                    # 새 ID 로 잠그기


def test_setup_flow_sts():
    """tools setup FL knee (STS): 공장 모터(ID 1) 하나 → ID 3, 가운데 2048, 토크 켬."""
    from dxldog import tools
    port = MockStsPort(ids=(1,))
    old_bus, old_sleep = tools._bus, tools.time.sleep
    tools._bus = lambda p, baud=None: real.StsBus("mock", port_handler=port)
    tools.time.sleep = lambda s: None
    try:
        tools.main(["setup", "FL", "knee", "--port", "MOCK", "--robot", "sts"])
    finally:
        tools._bus, tools.time.sleep = old_bus, old_sleep
    r = port.motors[3]
    assert list(port.motors) == [3]
    assert r[7] == 0 and r[33] == 0 and r[55] == 1 and r[40] == 1
    assert int.from_bytes(r[42:44], "little") == 2048


def test_fake_bus_rules():
    for robot in ("mixed", "sts"):
        C.use(robot)
        bus = real.FakeBus()
        assert bus.ping(3) == C.MODEL_NUMBER[C.MOTOR_MODEL["knee"]]
        try:                                                           # DYNAMIXEL: 토크 켠 채 EEPROM / STS: 잠긴 채 EEPROM
            if robot == "mixed":
                bus.write(1, "torque_enable", 1)
            bus.write(1, "min_position_limit", 100)
            raise AssertionError("EEPROM 보호가 동작해야 함")
        except RuntimeError:
            pass
        bus.write(1, "torque_enable", 0)
        bus.write_eeprom(1, {"min_position_limit": 100})
        assert bus.reg[1]["min_position_limit"] == 100
        try:
            bus.write(1, "lock" if robot == "mixed" else "profile_velocity", 0)   # 다른 회사 레지스터 이름
            raise AssertionError("없는 레지스터는 오류여야 함")
        except RuntimeError:
            pass


def test_fake_real_backend_ticks():
    import numpy as np
    for robot in ("mixed", "sts"):
        hw = real.DxlReal("fake", robot=robot)
        hw.set_joint_targets(-np.radians([0, -67.1, 96.6] * 4))
        goal = hw.bus.log[-1][2]
        assert goal[3] == 949 and goal[6] == 3147, goal                 # 앞왼쪽·앞오른쪽 무릎 (docs/software.md 의 예)
        assert hw.bus.reg[1][C.SPEED_ITEM] == 0
        hw.close()


if __name__ == "__main__":
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            try:
                fn()
                print("✓", name)
            except Exception as e:                                     # noqa: BLE001
                fails += 1
                print("✗", name, type(e).__name__, e)
    C.use("mixed")
    sys.exit(1 if fails else 0)
