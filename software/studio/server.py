"""DXL Dog Studio 서버 — 로봇(시뮬레이터 또는 실물)을 켜 둔 채 스튜디오가 보내는 코드를 실행한다.

실행(보통은 studio_web.py 가 띄운다):
  python server.py                                   시뮬레이터 (실제 시간 속도)
  python server.py --backend real --port COM3        실물 (노트북 + U2D2 / URT-1 등 어댑터)
  python server.py --robot sts ...                   로봇 종류: mixed(DYNAMIXEL) 또는 sts(STS3215). 생략하면 기본 종류
  python server.py --backend real --port /dev/ttyUSB0   실물 (Raspberry Pi)
  python server.py --backend real --port fake        가짜 모터 (하드웨어 없이 실물 코드 경로 시험)
  python server.py --fast                            시험용: 시뮬레이터를 최대 속도로

stdin  (한 줄에 JSON 하나): {"cmd": "run", "code": "..."} | {"cmd": "stop"} | {"cmd": "reset"} | {"cmd": "quit"}
       조작 패널: {"cmd": "panel", "op": "joints", "deg": [12개]} | {"op": "pose", "name": "stand"}
                 | {"op": "gait", "on": true, "T": 0.5, "step": 0.04, "lift": 0.03, "height": -0.155, "duty": 0.65, "turn": 0}
stdout: 학생 코드의 print 출력, 그리고 EVENT_PREFIX 로 시작하는 상태 알림 줄
        ready(mode, port, robot) / fatal / run_start / run_end(ok|error|stopped) / reset_done / busy / notice
        frame(30 Hz: 3D 화면용 바디 자세) / state(10 Hz: 관절각, 목표, 온도, 높이, 기울기, 모터 위치값)
학생 코드는 별도 스레드에서 실행하고, 시간 진행(물리 계산 또는 실물 박자 맞추기)은 메인 스레드가 맡는다(dxldog/live.py 참고).
실물 모드의 3D 화면은 '디지털 트윈': 모터에서 읽은 관절각을 시뮬레이터 모델에 넣어 모양만 그린다(물리 계산 없음).
"""
import argparse
import ctypes
import json
import math
import queue
import sys
import threading
import time
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))      # software/ → dxldog 패키지

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from dxldog import config as C  # noqa: E402
from dxldog import live  # noqa: E402
from dxldog.dog import inverse_kinematics, trot_feet  # noqa: E402
from dxldog.sim import DxlSim, pose_degrees  # noqa: E402

EVENT_PREFIX = "\x01EVT "
CODE_NAME = "내 코드"
CHUNK = {"sim": 12, "real": 20}       # 학생 코드가 없을 때 한 번에 진행하는 ms (실물은 50 Hz 로 명령)
PANEL_SPEED = math.radians(90)        # 조작 패널: 관절이 움직이는 최대 속도 (90°/s) → 갑자기 튀지 않음
SAFE_STAND_HEIGHT = 0.12              # 이보다 낮거나 넘어진 상태에서 서기·앉기는 먼저 엎드린 자세를 거친다
GAIT_MIN_HEIGHT = 0.14                # 이보다 낮으면 '먼저 서기' 안내 (엎드림 약 13 cm, 서기 약 16 cm)
STATE_EVERY = 0.1                     # 상태 알림 주기 (초)
FRAME_EVERY = 1 / 30                  # 웹 3D 화면용 자세 전송 주기 (초)
TWIN_MAX_TILT = math.radians(40)


def emit(event, **data):
    sys.stdout.write(EVENT_PREFIX + json.dumps({"event": event, **data}, ensure_ascii=False) + "\n")
    sys.stdout.flush()


def pose_rad(name):
    """자세 이름 → 모델 관절각 rad 12개."""
    return -np.radians(np.array(pose_degrees(name), float))


def quat_tilt(n):
    """몸통 좌표의 바닥 법선 n 을 세계 z 축으로 돌리는 회전 (w, x, y, z)."""
    axis = np.cross(n, [0.0, 0.0, 1.0])
    s = np.linalg.norm(axis)
    q = np.array([1.0, 0, 0, 0])
    if s > 1e-9:
        mujoco.mju_axisAngle2Quat(q, axis / s, math.acos(max(-1.0, min(1.0, n[2]))))
    return q


class Twin:
    """실물용 디지털 트윈: 관절각만 받아 MuJoCo 모델의 모양을 계산한다 (mj_kinematics, 물리 없음).
    바닥이 평평하다고 보고, 네 발 중 바닥에 닿는 세 발이 만드는 면이 수평이 되게 몸통을 기울인 뒤 바닥에 내려 놓는다."""

    def __init__(self):
        self.sim = DxlSim()
        self.height = 0.0

    def update(self, q):
        s = self.sim
        m, d = s.model, s.data
        d.qpos[s.qadr] = q
        d.qpos[0:3] = [0, 0, 0.3]
        d.qpos[3:7] = [1, 0, 0, 0]
        mujoco.mj_kinematics(m, d)
        feet = s.feet_world() - d.qpos[0:3]               # 몸통 좌표 (지금은 기울지 않았으므로 같은 방향)
        best = None
        for drop in range(4):                             # 발 세 개로 만든 면 중, 남은 발이 그 아래로 내려가지 않는 면
            a, b, c = feet[[i for i in range(4) if i != drop]]
            n = np.cross(b - a, c - a)
            if np.linalg.norm(n) < 1e-9:
                continue
            n = n / np.linalg.norm(n)
            n = n if n[2] > 0 else -n
            if np.dot(feet[drop] - a, n) < -1e-4:
                continue
            tilt = math.acos(min(1.0, n[2]))
            if best is None or tilt < best[0]:
                best = (tilt, n)
        if best and best[0] < TWIN_MAX_TILT:
            d.qpos[3:7] = quat_tilt(best[1])
            mujoco.mj_kinematics(m, d)
        d.qpos[2] -= s._lowest_point() - 0.0005           # 가장 낮은 곳(발 또는 몸통)이 바닥에 닿게
        mujoco.mj_kinematics(m, d)
        self.height = float(d.qpos[2])


class Server:
    def __init__(self, backend="sim", port=None, fast=False):
        self.kind = backend
        self.port = port
        self.fast = fast
        if backend == "sim":
            self.hw = DxlSim()
            self.twin = None
        else:
            from dxldog.real import DxlReal
            self.hw = DxlReal(port)
            self.twin = Twin()
        self.chunk = CHUNK[self.kind]
        self.ctx = live.LiveContext(self.hw)
        live.CONTEXT = self.ctx
        self.commands = queue.Queue()
        self.worker = None
        self.code_running = False          # 학생 코드 실행 중 (run_end 를 알리기 직전에 False 가 됨)
        self.stop_requested_at = None
        self.injected = False
        self.panel_goal = None             # 조작 패널이 정한 목표각(모델 rad 12개)
        self.panel_waypoints = []          # 다음에 갈 목표각들 (서기 전에 엎드리기 등)
        self.gait = None                   # 조작 패널 걸음새 설정
        self.gait_t = 0.0
        self.last_state = 0.0
        self.last_frame = 0.0
        self.last_error = 0.0
        self.q = self.hw.joint_pos         # 실물은 읽을 때마다 통신하므로 한 번 읽어 두고 쓴다
        if self.twin:
            self.twin.update(self.q)

    # ---------- 명령 수신 ----------
    def _read_stdin(self):
        for line in sys.stdin:
            line = line.strip()
            if line:
                self.commands.put(json.loads(line))
        self.commands.put({"cmd": "quit"})

    def _handle(self, msg):
        cmd = msg.get("cmd")
        busy = self.code_running
        if cmd == "run":
            if busy:
                emit("busy")
            else:
                self.panel_goal, self.gait = None, None     # 이제 학생 코드가 로봇을 맡는다
                self.code_running = True
                self.worker = threading.Thread(target=self._run_code, args=(msg["code"],), daemon=True)
                self.worker.start()
        elif cmd == "stop" and busy:
            self.ctx.stop_flag = True                    # 다음 API 호출에서 StopRun 으로 멈춤
            self.stop_requested_at = time.perf_counter()
            self.injected = False
        elif cmd == "reset":
            if busy:
                emit("busy")
            else:
                with self.ctx.lock:
                    self.gait, self.panel_waypoints = None, []
                    if self.kind == "sim":
                        self.hw.reset("lie")
                        self.panel_goal = None
                    else:                                # 실물은 순간이동이 없으니 천천히 엎드린다
                        self.panel_goal = self._clip(pose_rad("lie"))
                        emit("notice", text="실물 로봇이 천천히 엎드려요.")
                emit("reset_done")
        elif cmd == "panel":
            if busy:
                emit("busy")
            else:
                with self.ctx.lock:
                    self._panel(msg)
        elif cmd == "quit":
            return False
        return True

    # ---------- 조작 패널 ----------
    def _clip(self, q):
        return np.clip(q, self.hw.lower, self.hw.upper)

    def _height(self):
        return self.twin.height if self.twin else float(self.hw.base_pos[2])

    def _panel(self, msg):
        op = msg.get("op")
        current = self.hw.q_target
        self.panel_waypoints = []
        if op == "joints":
            self.panel_goal = self._clip(-np.radians(np.asarray(msg["deg"], float)))
            self.gait = None
        elif op == "pose":
            name = msg.get("name")
            if name not in C.POSES:
                return
            self.gait = None
            goal = self._clip(pose_rad(name))
            if name in ("stand", "sit") and (self._height() < SAFE_STAND_HEIGHT or self.hw.fallen()):
                # 이상하게 접힌 자세에서 바로 다리를 펴면 넘어지기 쉬우므로 먼저 엎드렸다가 일어선다
                self.panel_goal = self._clip(pose_rad("lie"))
                self.panel_waypoints = [goal]
            else:
                self.panel_goal = goal
        elif op == "gait":
            if msg.get("on") and self.hw.fallen():
                emit("notice", text="로봇이 넘어져 있어요. ↺ 처음 자세를 누른 뒤 서기 → 걷기 순서로 해 보세요.")
            elif msg.get("on") and self._height() < GAIT_MIN_HEIGHT:
                emit("notice", text="먼저 '서기'를 눌러 로봇을 세운 뒤 걸어 보세요.")
            elif msg.get("on"):
                if self.gait is None:
                    self.gait_t = 0.0                    # 걷는 중 값만 바꿀 때는 박자를 이어 간다
                    if self.kind == "real":
                        emit("notice", text="실물 걷기: 처음에는 로봇을 받침대에 올려 발이 바닥에 닿지 않게 하고 시험하세요.")
                self.gait = {k: float(msg[k]) for k in ("T", "step", "lift", "height")}
                self.gait.update(duty=float(msg.get("duty", 0.65)), turn=float(msg.get("turn", 0.0)))
            else:
                self.gait = None
                self.panel_goal = current.copy()     # 멈춘 자리에서 그 자세 유지

    def _panel_tick(self):
        """학생 코드가 없을 때 chunk 마다: 패널 목표를 향해 천천히 이동하거나 걸음새 진행."""
        if self.panel_goal is None and self.gait is None:
            return
        current = self.hw.q_target
        dt = self.chunk * 0.001
        if self.gait:
            g = self.gait
            q = -np.degrees(current).reshape(4, 3)           # 학생 각도
            try:
                ramp = min(1.0, self.gait_t / g["T"])          # 첫 한 주기 동안 보폭을 0 에서 천천히 키움 (갑자기 튀지 않게)
                feet = trot_feet(self.gait_t, g["T"], g["step"] * ramp, g["lift"], g["height"],
                                 duty=g["duty"], turn=g["turn"] * ramp)
                for k, leg in enumerate(C.LEGS):
                    q[k, 1:] = inverse_kinematics(*feet[leg])
                target = -np.radians(q.reshape(-1))
            except ValueError:
                target = current
            self.gait_t += dt
        else:
            rate = PANEL_SPEED * dt
            target = current + np.clip(self.panel_goal - current, -rate, rate)
            if self.panel_waypoints and np.max(np.abs(self.panel_goal - target)) < 1e-3 \
                    and np.max(np.abs(self.hw.joint_pos - target)) < np.radians(6):
                self.panel_goal = self.panel_waypoints.pop(0)     # 이번 자세에 도착 → 다음 자세로
        target = self._clip(target)
        if np.max(np.abs(target - current)) < 1e-7:
            return                                       # 같은 목표는 다시 보내지 않음 (실물 통신 아끼기)
        self.hw.set_joint_targets(target)

    # ---------- 화면·상태 알림 ----------
    def _sense(self):
        self.q = self.hw.joint_pos
        if self.twin:
            self.twin.update(self.q)

    def _emit_frame(self):
        """웹 3D 화면용: 바디 14개의 위치(xpos)와 회전(xquat, w x y z). 실물은 트윈의 값."""
        d = self.twin.sim.data if self.twin else self.hw.data
        emit("frame", p=np.round(d.xpos, 4).reshape(-1).tolist(), q=np.round(d.xquat, 4).reshape(-1).tolist())

    def _ticks(self, q):
        if self.kind == "real":
            return [int(t) for t in self.hw.rad_to_tick(q)]
        dirs = np.array([C.DIRECTION[(leg, j)] for leg in C.LEGS for j in C.JOINTS])
        return [int(t) for t in np.round(C.CENTER + dirs * np.asarray(q) * C.RAD_TO_TICK)]

    def _emit_state(self):
        """조작 패널 표시용 상태. 각도는 학생 각도(도)."""
        tgt = self.hw.q_target
        rpy = np.degrees(self.hw.rpy())
        emit("state",
             q=[round(float(a), 1) for a in -np.degrees(self.q)],
             target=[round(float(a), 1) for a in -np.degrees(tgt)],
             temp=[round(float(t), 1) for t in self.hw.temperature],
             tlim=[float(t) for t in self.hw.temp_limit],
             ticks=self._ticks(tgt),                     # Goal Position (DYNAMIXEL 주소 116, STS 42)
             pticks=self._ticks(self.q),                 # Present Position (DYNAMIXEL 주소 132, STS 56)
             offset=[int(o) for o in getattr(self.hw, "offset", np.zeros(12))],
             rpy=[round(float(a), 1) for a in rpy],
             height=round(self._height(), 3),
             fallen=bool(self.hw.fallen()),
             mode=self.kind, port=self.port, robot=C.ROBOT,
             gait=self.gait is not None)

    # ---------- 학생 코드 실행 (작업 스레드) ----------
    def _run_code(self, code):
        self.ctx.lock.acquire()
        self.ctx.stop_flag = False
        self.stop_requested_at = None
        emit("run_start")
        status = "ok"
        try:
            exec(compile(code, CODE_NAME, "exec"), {"__name__": "__main__"})
        except live.StopRun:
            status = "stopped"
        except SystemExit:
            pass
        except BaseException as e:
            status = "error"
            # 서버 내부(exec 호출) 줄은 빼고 학생 코드부터 보여 준다
            traceback.print_exception(type(e), e, e.__traceback__.tb_next, file=sys.stdout)
        finally:
            self.ctx.stop_flag = False
            self.code_running = False
            self.ctx.lock.release()
            sys.stdout.flush()
            emit("run_end", status=status)

    def _force_stop_if_stuck(self):
        """정지를 눌렀는데 0.5초가 지나도 API 를 부르지 않는 계산(while True: pass 등)에 갇혀 있으면
        작업 스레드에 StopRun 예외를 한 번 주입한다. (API 대기 중일 때는 주입하지 않음)"""
        if (self.stop_requested_at is None or self.injected or self.worker is None
                or not self.code_running):
            return
        if time.perf_counter() - self.stop_requested_at > 0.5 and self.ctx.remaining == 0 \
                and self.ctx.stop_flag:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_ulong(self.worker.ident),
                                                       ctypes.py_object(live.StopRun))
            self.injected = True

    # ---------- 메인 루프: 시간 진행 + 화면 갱신 ----------
    def loop(self, max_seconds=None):
        threading.Thread(target=self._read_stdin, daemon=True).start()
        emit("ready", mode=self.kind, port=self.port, robot=C.ROBOT)
        sim = self.kind == "sim"
        wall0, sim0 = time.perf_counter(), (self.hw.data.time if sim else 0.0)
        started = time.perf_counter()
        while True:
            if max_seconds and time.perf_counter() - started > max_seconds:
                break
            try:
                while True:
                    if not self._handle(self.commands.get_nowait()):
                        return
            except queue.Empty:
                pass

            self._force_stop_if_stuck()
            if not self.ctx.lock.acquire(timeout=0.005):
                continue                                 # 학생 코드가 계산 중 → 이번 차례는 건너뜀
            n = 0
            try:
                busy = self.code_running
                if not busy:
                    self._panel_tick()
                n = min(self.chunk, self.ctx.remaining) if busy else self.chunk
                if n:
                    self.hw.step(n)                      # 시뮬: 물리 n ms / 실물: n ms 기다림
                if busy and n:
                    self.ctx.remaining -= n
                    if self.ctx.remaining == 0:
                        self.ctx.done.set()
                now = time.perf_counter()
                if now - self.last_frame > FRAME_EVERY:
                    self.last_frame = now
                    self._sense()
                    self._emit_frame()
                if now - self.last_state > STATE_EVERY:
                    self.last_state = now
                    self._emit_state()
            except RuntimeError as e:                    # 실물 통신 오류 등: 알리고 계속
                if time.perf_counter() - self.last_error > 2:
                    self.last_error = time.perf_counter()
                    emit("notice", text=f"모터 통신 오류: {e}")
            finally:
                self.ctx.lock.release()

            if sim and not self.fast:
                lag = (self.hw.data.time - sim0) - (time.perf_counter() - wall0)
                if lag > 0:
                    time.sleep(lag)                      # 실제 시간 속도에 맞추기
                elif lag < -0.2:
                    wall0, sim0 = time.perf_counter(), self.hw.data.time   # 느린 PC: 밀린 시간은 버림
            if n == 0:
                time.sleep(0.001)

    def close(self):
        if self.kind == "real":
            self.hw.close(torque_off=False)              # 모터는 마지막 자세를 계속 잡는다


def main():
    ap = argparse.ArgumentParser(description="DXL Dog Studio 서버")
    ap.add_argument("--backend", choices=["sim", "real"], default="sim")
    ap.add_argument("--port", default=None, help="실물 포트: COM3, /dev/ttyUSB0, fake(가짜 모터)")
    ap.add_argument("--robot", choices=list(C.ROBOTS), default=None, help=f"로봇 종류 (기본: {C.ROBOT})")
    ap.add_argument("--fast", action="store_true", help="시험용: 시뮬레이터를 최대 속도로")
    ap.add_argument("--stream", action="store_true", help="(호환용, 무시)")
    args = ap.parse_args()
    if args.backend == "real" and not args.port:
        args.port = "fake"
    if args.robot:
        C.use(args.robot)                                # 시뮬·실물·트윈 모두 이 로봇 종류로
    try:
        server = Server(args.backend, args.port, args.fast)
    except Exception as e:                               # 포트를 못 열거나 모터가 없음 → 알리고 대기 (재시작 반복 방지)
        emit("fatal", text=str(e), mode=args.backend, port=args.port, robot=C.ROBOT)
        for line in sys.stdin:
            if '"quit"' in line:
                break
        return
    try:
        server.loop()
    finally:
        server.close()


if __name__ == "__main__":
    main()
