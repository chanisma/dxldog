"""실물 준비 도구 — 모터 ID 정하기, 연결 확인, 영점 보정, 방향 확인. (노트북·Raspberry Pi 같은 명령, DYNAMIXEL·STS 모두)

  python -m dxldog.tools ports                          연결된 시리얼 포트 목록 (어댑터 찾기)
  python -m dxldog.tools use sts                        기본 로봇 종류 저장 (mixed 또는 sts → ~/.dxldog/robot.json)
  python -m dxldog.tools setup  --port COM3 FL knee     모터 '하나만' 연결하고 → ID·통신 속도 설정 (12번 반복)
  python -m dxldog.tools scan   --port COM3             12개 모터 응답·종류·온도·전압 확인
  python -m dxldog.tools calibrate --port COM3          조립 자세(다리 똑바로 아래)에서 영점 보정값 저장
  python -m dxldog.tools check  --port COM3             관절을 하나씩 +10° 움직여 방향 확인
  python -m dxldog.tools relax  --port COM3             모든 모터 힘 빼기

모든 명령에 --robot mixed / --robot sts 를 붙일 수 있다 (생략하면 기본 종류).
포트 이름: Windows 는 COM3 같은 이름, Raspberry Pi 는 /dev/ttyUSB0 (STS 어댑터는 /dev/ttyACM0 일 수 있음).
--port fake 는 가짜 모터(연습용).
"""
import argparse
import json
import sys
import time

import numpy as np

from . import config as C
from .real import DxlReal, cal_file, open_bus

KEYS = [(l, j) for l in C.LEGS for j in C.JOINTS]


def _bus(port, baud=None):
    return open_bus(port, baud)


def cmd_ports(_):
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        print(f"시리얼 포트가 없어요. {C.ADAPTER} 의 USB 케이블을 확인하세요.")
    for p in ports:
        print(f"  {p.device:15s} {p.description}")


def cmd_use(a):
    path = C.save_default(a.robot_name)
    print(f"기본 로봇 종류: {a.robot_name} ({C.ROBOT_INFO['name']}) — 저장: {path}")


def _find_one_dxl(port):
    """DYNAMIXEL: 공장 속도와 1 Mbps 에서 broadcast ping → (bus, baud, id, model)."""
    for baud in (C.FACTORY_BAUD, C.BAUD):
        bus = _bus(port, baud)
        data, res = bus.pk.broadcastPing(bus.port)
        if res == bus._ok and data:
            if len(data) > 1:
                bus.close()
                sys.exit(f"모터가 {len(data)}개 응답했어요 (ID {sorted(data)}). 설정할 모터 하나만 연결해 주세요.")
            mid = next(iter(data))
            return bus, baud, mid, data[mid][0]
        bus.close()
    return None


def _find_one_sts(port):
    """STS: broadcast ping 이 없어서 ID 0–252 를 하나씩 ping (1 Mbps, 약 10초). 못 찾으면 다른 속도에서 ID 1–12 만."""
    print(f"모터를 찾는 중… ({C.FACTORY_BAUD} bps, ID 0~252, 약 10초)")
    bus = _bus(port, C.FACTORY_BAUD)
    found = {i: m for i in range(0, 253) if (m := bus.ping(i)) is not None}
    if found:
        if len(found) > 1:
            bus.close()
            sys.exit(f"모터가 {len(found)}개 응답했어요 (ID {sorted(found)}). 설정할 모터 하나만 연결해 주세요.")
        mid = next(iter(found))
        return bus, C.FACTORY_BAUD, mid, found[mid]
    bus.close()
    for baud in (500_000, 115_200, 57_600):
        bus = _bus(port, baud)
        for i in [1] + list(range(2, 13)):
            m = bus.ping(i)
            if m is not None:
                return bus, baud, i, m
        bus.close()
    return None


def cmd_setup(a):
    """새 모터를 하나만 연결한 상태에서 ID·통신 속도·응답 지연을 정한다.
    DYNAMIXEL 공장 설정: ID 1, 57600 bps.  STS3215 공장 설정: ID 1, 1 Mbps."""
    if a.port == "fake":
        sys.exit("setup 은 진짜 모터 하나를 연결해서 써요 (가짜 모터는 이미 ID 가 정해져 있어요).")
    key = (a.leg, a.joint)
    new_id, want = C.MOTOR_IDS[key], C.MOTOR_MODEL[a.joint]
    found = _find_one_sts(a.port) if C.BUS == "feetech" else _find_one_dxl(a.port)
    if not found:
        sys.exit(f"응답하는 모터가 없어요. 전원(12 V)과 케이블, 포트 이름, 로봇 종류(지금 {C.ROBOT})를 확인하세요.")
    bus, baud, old_id, model = found
    print(f"찾음: ID {old_id}, {C.model_name(model)}, {baud} bps")
    if model != C.MODEL_NUMBER[want]:
        print(f"⚠️ {a.leg} {a.joint} 자리는 {want} 인데 연결된 모터는 {C.model_name(model)} 예요. 그래도 계속합니다.")
    bus.write(old_id, "torque_enable", 0)
    eeprom = {"return_delay_time": 0}                 # 응답을 바로 보내게 (기본 500 µs → 0)
    if C.BUS == "feetech":
        eeprom["operating_mode"] = C.POSITION_MODE    # 위치 제어
        eeprom["homing_offset"] = 0                   # ⚠️ 다른 프로그램(LeRobot 등)이 바꿔 둔 영점 이동을 지운다
    eeprom["id"] = new_id
    mid = bus.write_eeprom(old_id, eeprom)            # STS: Lock 풀기 → 쓰기 → 잠그기
    if baud != C.BAUD:
        try:
            if "lock" in C.ADDR:
                bus.write(mid, "lock", 0)
            bus.write(mid, "baud_rate", C.BAUD_CODE[C.BAUD])
        except RuntimeError:                           # 속도가 바로 바뀌어 응답을 못 받을 수 있다 → 아래에서 ping 으로 확인
            pass
    bus.close()
    time.sleep(0.2)
    bus = _bus(a.port, C.BAUD)
    ok = bus.ping(new_id) is not None
    if ok:                                             # 조립 준비: 가운데(2048)로 돌려 놓고 힘을 준 채로 둔다
        if "lock" in C.ADDR:
            bus.write(new_id, "lock", 1)
        if C.BUS == "dynamixel":
            bus.write(new_id, "operating_mode", C.POSITION_MODE)
        bus.write(new_id, C.SPEED_ITEM, int(round(7 / C.SPEED_UNIT_RPM)))      # 천천히 (약 7 rpm)
        bus.write(new_id, "goal_position", C.CENTER)
        bus.write(new_id, "torque_enable", 1)
        time.sleep(2.0)
    bus.close()
    print(f"{'완료' if ok else '확인 실패'}: {a.leg} {a.joint} → ID {new_id}, {C.BAUD} bps. 모터에 'ID {new_id}' 스티커를 붙여 두세요.")
    if ok:
        print("모터가 가운데(2048)에서 힘을 주고 있어요. 이 상태에서 혼에 부품을 '다리 똑바로 아래' 방향으로 끼우면 됩니다.")


def cmd_scan(a):
    bus = _bus(a.port)
    print(f" {C.ROBOT_INFO['name']} — 포트 {a.port}")
    print(" 다리 관절    ID  모델          위치  온도  전압")
    missing = 0
    for leg, j in KEYS:
        i = C.MOTOR_IDS[(leg, j)]
        model = bus.ping(i)
        if model is None:
            print(f" {leg:3s} {j:6s} {i:3d}  ✗ 응답 없음")
            missing += 1
            continue
        pos, temp, volt = bus.read(i, "present_position"), bus.read(i, "present_temperature"), bus.read(i, "present_input_voltage")
        mark = "" if model == C.MODEL_NUMBER[C.MOTOR_MODEL[j]] else f"  ⚠️ {C.MOTOR_MODEL[j]} 이어야 함"
        print(f" {leg:3s} {j:6s} {i:3d}  {C.model_name(model):12s} {pos:5d} {temp:4d}°C {volt * C.VOLT_UNIT:4.1f} V{mark}")
    bus.close()
    print("모두 응답함 ✓" if not missing else f"{missing}개가 응답하지 않아요. 케이블 연결(데이지 체인)과 ID, 로봇 종류를 확인하세요.")


def limits_ticks(offsets):
    """관절 가동 범위 → 모터 위치 한계(틱). 모터 자체가 범위 밖으로 못 가게 EEPROM 에 적는다."""
    out = {}
    for n, (leg, j) in enumerate(KEYS):
        lo, hi = C.JOINT_RANGE_DEG[j]
        t = [C.CENTER + C.DIRECTION[(leg, j)] * -np.radians(d) * C.RAD_TO_TICK + offsets[n] for d in (lo, hi)]
        out[(leg, j)] = (int(np.floor(min(t))) - 10, int(np.ceil(max(t))) + 10)      # 10틱(약 1°) 여유
    return out


def cmd_calibrate(a):
    bus = _bus(a.port)
    ids = [C.MOTOR_IDS[k] for k in KEYS]
    for i in ids:
        bus.write(i, "torque_enable", 0)
    print("모든 모터의 힘을 뺐어요.\n"
          "1) 로봇 몸통을 받침대 위에 올려 다리가 바닥에 닿지 않게 하세요.\n"
          "2) 네 다리를 '똑바로 아래로' 맞추세요: 어깨는 수직, 허벅지와 정강이는 일직선 (조립 자세).\n"
          "   직각자나 3D 프린트 지그를 대면 더 정확해요.")
    if a.port != "fake":
        input("맞췄으면 Enter ▶ ")
    pos = bus.sync_read("present_position", ids)
    offsets = np.array([pos[i] - C.CENTER for i in ids], float)
    cal = {f"{l}_{j}": int(o) for (l, j), o in zip(KEYS, offsets)}
    bad = [f"{k}({v:+d}틱 = {v / C.RAD_TO_TICK * 57.3:+.1f}°)" for k, v in cal.items() if abs(v) > 100]
    if bad:
        print("⚠️ 영점에서 9° 넘게 벗어난 모터:", ", ".join(bad))
        print("   혼(바퀴)을 2048 위치에서 끼웠는지 확인하세요. 혼 표시(점)와 모터 몸통 표시가 맞아야 해요.")
    lim = limits_ticks(offsets)
    for (k, (lo, hi)), i in zip(lim.items(), ids):             # 토크가 꺼진 지금 EEPROM 에 위치 한계 쓰기 (STS 는 Lock 풀고)
        bus.write_eeprom(i, {"min_position_limit": max(0, lo), "max_position_limit": min(4095, hi)})
    bus.close()
    if a.port == "fake":
        print("(가짜 모터라 파일은 저장하지 않아요)", cal)
        return
    path = cal_file()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(cal, indent=1), encoding="utf-8")
    print(f"저장: {path}\n모터 위치 한계도 관절 범위에 맞게 설정했어요. "
          f"다음 단계: python -m dxldog.tools check --port {a.port} --robot {C.ROBOT}")


def cmd_check(a):
    expect = {"hip_x": "발이 로봇의 오른쪽으로", "hip_y": "허벅지가 앞으로", "knee": "무릎이 더 굽혀짐(발이 몸 쪽으로)"}
    hw = DxlReal(a.port, speed_rpm=7)                          # 천천히 (약 7 rpm)
    hw.set_joint_targets(np.zeros(12))
    time.sleep(2.0)
    wrong = []
    for n, (leg, j) in enumerate(KEYS):
        q = np.zeros(12)
        q[n] = -np.radians(10 if j != "knee" else 20)              # 학생 각도 +10° (무릎 +20°)
        hw.set_joint_targets(q)
        time.sleep(1.2)
        if a.port == "fake":
            ans = "y"
        else:
            ans = input(f"{leg} {j}: {expect[j]} 움직였나요? (y/n) ").strip().lower()
        if ans.startswith("n"):
            wrong.append(f"{leg}_{j}")
        hw.set_joint_targets(np.zeros(12))
        time.sleep(1.0)
    hw.close(torque_off=True)
    if wrong:
        print("방향이 반대인 관절:", ", ".join(wrong),
              "\n→ 모터를 거꾸로 달았을 수 있어요. 조립 그림과 비교하거나, config.py 의 dir_sign(로봇 종류별 방향)을 바꾸세요.")
        if len(wrong) == 12:
            print(f"   12개 모두 반대라면 {C.BRAND} 모터의 + 회전 방향이 반대예요 → config.ROBOTS['{C.ROBOT}']['dir_sign'] 를 모두 −1 로.")
    else:
        print(f"12개 관절 방향 모두 맞음 ✓  이제 예제를 실물로 돌려 보세요: python lessons/j1_one_motor.py --real {a.port} --robot {C.ROBOT}")


def cmd_relax(a):
    bus = _bus(a.port)
    for k in KEYS:
        bus.write(C.MOTOR_IDS[k], "torque_enable", 0)
    bus.close()
    print("모든 모터의 힘을 뺐어요.")


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m dxldog.tools", description="DXL Dog 실물 준비 도구")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ports", help="시리얼 포트 목록").set_defaults(fn=cmd_ports)
    u = sub.add_parser("use", help="기본 로봇 종류 저장")
    u.add_argument("robot_name", choices=list(C.ROBOTS))
    u.set_defaults(fn=cmd_use)
    s = sub.add_parser("setup", help="모터 하나의 ID·통신 속도 설정")
    s.add_argument("leg", choices=C.LEGS)
    s.add_argument("joint", choices=C.JOINTS)
    s.set_defaults(fn=cmd_setup)
    for name, fn, h in (("scan", cmd_scan, "12개 모터 확인"), ("calibrate", cmd_calibrate, "영점 보정"),
                        ("check", cmd_check, "방향 확인"), ("relax", cmd_relax, "힘 빼기")):
        sub.add_parser(name, help=h).set_defaults(fn=fn)
    for name, p in sub.choices.items():
        if name == "use":
            continue
        p.add_argument("--robot", choices=list(C.ROBOTS), default=None, help=f"로봇 종류 (기본: {C.ROBOT})")
        if name != "ports":
            p.add_argument("--port", required=True, help="COM3, /dev/ttyUSB0, /dev/ttyACM0, fake")
    a = ap.parse_args(argv)
    if getattr(a, "robot", None):
        C.use(a.robot)
    a.fn(a)


if __name__ == "__main__":
    main()
