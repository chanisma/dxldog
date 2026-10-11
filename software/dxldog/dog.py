"""학생용 DXL Dog API — 시뮬레이터와 실물에서 **같은 코드**가 돈다.

    from dxldog import Dog
    dog = Dog()                          # 시뮬레이터 (화면 창이 뜸)
    dog = Dog("real", port="COM3")       # 노트북 + 어댑터(U2D2 / URT-1 등) 로 실물
    dog = Dog("real", port="/dev/ttyUSB0")   # Raspberry Pi 위에서 실물
    dog = Dog("real", port="fake")       # 하드웨어 없이 실물 코드 경로 시험 (가짜 모터)
    dog = Dog(robot="sts")               # 로봇 종류 고르기: "mixed"(DYNAMIXEL) 또는 "sts"(STS3215). 생략하면 기본 종류

함수 이름·각도 규칙은 robotdog 의 JointDog(Lite3) 와 같다 → Lite3 에서 배운 코드를 거의 그대로 쓸 수 있다.
"""
import atexit
import time

import numpy as np

from . import config as C
from . import live

CTRL_DT = 0.02                    # 명령 주기 50 Hz (모터 12개 동시 쓰기에 충분히 여유 있음)
TEMP_WARN_MARGIN = 8.0            # 온도 한계보다 이만큼 낮으면 경고


def _index(leg, joint):
    if leg not in C.LEGS:
        raise ValueError(f"다리 이름 '{leg}' 은(는) 없어요. 쓸 수 있는 이름: {', '.join(C.LEGS)}")
    if joint not in C.JOINTS:
        raise ValueError(f"관절 이름 '{joint}' 은(는) 없어요. 쓸 수 있는 이름: {', '.join(C.JOINTS)}")
    return C.LEGS.index(leg) * 3 + C.JOINTS.index(joint)


def motor_info(leg, joint):
    """학생용 이름 → 모터 ID 와 모델. 예: ("FL", "knee") → (3, "XM430-W350"), STS 로봇은 (3, "STS3215-12V")"""
    _index(leg, joint)
    return C.MOTOR_IDS[(leg, joint)], C.MOTOR_MODEL[joint]


def deg_to_tick(leg, joint, deg, offset=0):
    """학생 각도(도) → 모터에 보내는 Goal Position 값(0~4095, 가운데 2048). DYNAMIXEL·STS 같은 식."""
    _index(leg, joint)
    return int(round(C.CENTER + C.DIRECTION[(leg, joint)] * -np.radians(deg) * C.RAD_TO_TICK + offset))


# ---------- 기구학 (다리 한 개, 옆에서 본 2차원 — JointDog 와 같은 식) ----------
def forward_kinematics(hip_y_deg, knee_deg):
    """관절 각도 → 발끝 위치 (x: 앞+, z: 위+, 엉덩이 관절 기준, m)."""
    q1, q2 = np.radians(hip_y_deg), np.radians(knee_deg)
    x = C.THIGH * np.sin(q1) + C.SHANK * np.sin(q1 + q2)
    z = -(C.THIGH * np.cos(q1) + C.SHANK * np.cos(q1 + q2))
    return float(x), float(z)


def inverse_kinematics(x, z):
    """발끝 위치 → 관절 각도 (hip_y, knee) [도]. 무릎은 뒤로 접히는 쪽 해를 고른다."""
    r2 = x * x + z * z
    c = (r2 - C.THIGH ** 2 - C.SHANK ** 2) / (2 * C.THIGH * C.SHANK)
    if abs(c) > 1:
        raise ValueError(f"다리가 닿을 수 없는 위치예요: 거리 {np.sqrt(r2):.3f} m (최대 {C.THIGH + C.SHANK:.3f} m)")
    q2 = np.arccos(c)
    q1 = np.arctan2(x, -z) - np.arctan2(C.SHANK * np.sin(q2), C.THIGH + C.SHANK * np.cos(q2))
    return float(np.degrees(q1)), float(np.degrees(q2))


# ---------- 트롯 걸음새 (J5 와 같은 규칙) ----------
TROT_PHASE = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}   # 대각선 짝
FOOT_X = -0.035     # 발을 엉덩이보다 3.5 cm 뒤에 둔다: 무릎이 모두 뒤로 굽어 무게중심이 뒤쪽에 있기 때문 (C.POSES["stand"] 와 같음)


def trot_feet(t, T=0.5, step=0.04, lift=0.03, height=-0.155, x_offset=FOOT_X, duty=0.65, turn=0.0):
    """시각 t 에서 네 발끝 위치 {다리: (x, z)}.
    한 주기(T초) 중 duty 비율은 딛기(발을 뒤로 밀기), 나머지는 옮기기(발을 들어 앞으로).
    duty 가 0.5 보다 크면 네 발이 모두 땅에 닿는 순간이 생겨 덜 흔들린다.
    step: 한 번 딛는 동안 발이 미는 거리 (m). 몸은 한 주기에 약 step / duty 만큼 나아간다.
    turn: + 이면 왼쪽으로, − 이면 오른쪽으로 돈다 (오른발 보폭 = step + turn, 왼발 = step − turn)."""
    feet = {}
    for leg, phase in TROT_PHASE.items():
        st = step + turn if leg[1] == "R" else step - turn
        p = (t / T + phase) % 1.0
        if p < duty:                                        # 딛기: 앞(+st/2) → 뒤(−st/2), 일정한 속도
            x, z = st / 2 - st * p / duty, height
        else:                                               # 옮기기: 뒤 → 앞, 부드럽게 들었다 내림
            s = (p - duty) / (1 - duty)
            x = -st / 2 + st * (1 - np.cos(np.pi * s)) / 2
            z = height + lift * np.sin(np.pi * s)
        feet[leg] = (x + x_offset, z)
    return feet


class Dog:
    def __init__(self, mode="sim", port=None, viewer=True, robot=None):
        if mode not in ("sim", "real"):
            raise ValueError('mode 는 "sim"(시뮬레이터) 또는 "real"(실물) 이에요')
        self.viewer = None
        self._live = live.CONTEXT
        if self._live:                                   # 스튜디오에 이미 켜진 시뮬/실물에 연결
            if robot and robot != C.ROBOT:
                print(f"(스튜디오에 켜진 로봇 종류 '{C.ROBOT}' 을 그대로 씁니다)")
            self.hw = self._live.backend
            self.robot = C.ROBOT
            self.mode = self.hw.kind
            self._warned = False
            self._bal = None
            return
        if robot:
            C.use(robot)                                 # 로봇 종류: "mixed" 또는 "sts"
        self.robot = C.ROBOT
        self.mode = mode
        if mode == "sim":
            from .sim import DxlSim
            self.hw = DxlSim()
            if viewer:
                import mujoco.viewer
                self.viewer = mujoco.viewer.launch_passive(self.hw.model, self.hw.data)
                self.viewer.cam.distance, self.viewer.cam.elevation, self.viewer.cam.azimuth = 0.9, -20, 135
                atexit.register(self._keep_open)
        else:
            if port is None:
                raise ValueError(f'실물은 port 가 필요해요. 예: Dog("real", port="COM3") ({C.ROBOT_INFO["ports"]})')
            from .real import DxlReal
            self.hw = DxlReal(port)
            atexit.register(self._goodbye)
            print(f"{C.BRAND} 모터 12개 연결 완료 ({port}). 토크 켜짐 — 지금 자세를 유지합니다.")
        self._warned = False
        self._bal = None                                  # 균형 잡기 (gain, damping) — set_balance 참고
        self._wall_start, self._sim_time = time.perf_counter(), 0.0
        self._hold(0.3)

    # ---------- 관절 하나 ----------
    def set_angle(self, leg, joint, deg):
        """관절 하나에 목표 각도를 준다(즉시). 모터는 그 각도를 향해 힘을 낸다."""
        q = self._targets()
        q[_index(leg, joint)] = deg
        self._send(q)

    def get_angle(self, leg, joint):
        """관절 하나의 현재 각도(센서 값, 도)."""
        return float(self._angles()[_index(leg, joint)])

    # ---------- 여러 관절 ----------
    def get_angles(self):
        """{다리: (hip_x, hip_y, knee)} 현재 각도."""
        q = self._angles().reshape(4, 3)
        return {leg: tuple(round(float(a), 1) for a in q[i]) for i, leg in enumerate(C.LEGS)}

    def move_to(self, pose, seconds=1.0):
        """여러 관절을 seconds 동안 부드럽게 목표 각도로 옮긴다(선형 보간).
        pose: "stand" 같은 이름, (hip_x, hip_y, knee) 하나(네 다리 공통), 또는 {다리: (…)}."""
        goal = self._pose_to_array(pose)
        if seconds <= 0:
            self._send(goal)
            return
        start = self._targets()
        n = max(1, int(round(seconds / CTRL_DT)))
        for i in range(1, n + 1):
            self._send(start + (goal - start) * i / n)
            self._advance(int(CTRL_DT * 1000))
        self._hold(0.2)

    def wait(self, seconds):
        self._hold(seconds)

    def reset(self):
        """처음 상태(엎드린 자세)로. 시뮬레이터는 바로 되돌리고, 실물은 2초 동안 천천히 엎드린다."""
        if self.mode == "sim":
            self.hw.reset("lie")
            if not self._live:
                self._wall_start, self._sim_time = time.perf_counter(), 0.0
            self._hold(0.3)
        else:
            self.move_to("lie", 2.0)

    # ---------- 발 위치 ----------
    def get_foot(self, leg):
        """발끝 위치 (x, z) [m] — 정기구학."""
        _, hy, kn = self.get_angles()[leg]
        return forward_kinematics(hy, kn)

    def set_feet(self, feet, seconds=0.5):
        """발끝 위치 {다리: (x, z)} 로 이동 — 역기구학. seconds=0 이면 즉시.
        set_balance 를 켜 두면 seconds=0 일 때 기울기 센서로 발 높이를 고쳐서 보낸다."""
        if self._bal and seconds <= 0:
            feet = self._balanced(feet)
        q = self._targets().reshape(4, 3)
        pose = {}
        for k, leg in enumerate(C.LEGS):
            hy, kn = inverse_kinematics(*feet[leg]) if leg in feet else q[k, 1:]
            pose[leg] = (q[k, 0], hy, kn)
        self.move_to(pose, seconds)

    # ---------- 몸 상태 ----------
    def get_height(self):
        """몸통(엉덩이 관절) 높이 [m]. 실물은 네 다리 관절 각도로 계산한 값(바닥이 평평하다고 가정)."""
        if self.mode == "sim":
            return float(self.hw.base_pos[2])
        return float(np.mean([-self.get_foot(leg)[1] for leg in C.LEGS]) + 0.012)

    def set_balance(self, on=True, gain=0.6, damping=0.007):
        """균형 잡기 켜기/끄기. 켜면 set_feet(..., seconds=0) 로 보내는 발 높이를 센서로 고친다:
        몸이 기운 쪽 다리는 늘이고 반대쪽은 줄인다.
          고치는 양 [cm] = gain × 기울기[°] + damping × 기우는 속도[°/s]
        (J6 예제가 같은 계산을 직접 해 본다.) 실물은 IMU 를 달아야 쓸 수 있다."""
        if on and not self.hw.has_imu:
            raise RuntimeError("균형 잡기에는 기울기 센서(IMU)가 필요해요. 지금 실물에는 센서가 없어요.")
        self._bal = (float(gain), float(damping)) if on else None

    @property
    def has_imu(self):
        """기울기 센서(IMU)가 있으면 True. 시뮬레이터는 항상 True."""
        return bool(self.hw.has_imu)

    def get_gyro(self):
        """(roll, pitch, yaw) 방향으로 몸이 도는 속도 [°/s] — 자이로 센서. 실물은 IMU 를 달기 전까지 (0, 0, 0)."""
        return tuple(float(v) for v in np.degrees(self.hw.gyro()))

    def _balanced(self, feet):
        gain, damping = self._bal
        roll, pitch = self.get_tilt()
        roll_rate, pitch_rate, _ = self.get_gyro()
        fix_pitch = (gain * pitch + damping * pitch_rate) / 100      # cm → m
        fix_roll = (gain * roll + damping * roll_rate) / 100
        fix_pitch, fix_roll = np.clip([fix_pitch, fix_roll], -0.03, 0.03)    # 한 번에 3 cm 까지만 (다리가 닿는 범위 안)
        out = {}
        for leg, (x, z) in feet.items():
            front = 1 if leg[0] == "F" else -1                       # 앞다리 +1, 뒷다리 −1
            left = 1 if leg[1] == "L" else -1                        # 왼다리 +1, 오른다리 −1
            out[leg] = (x, z - front * fix_pitch + left * fix_roll)  # 앞이 숙여지면(pitch +) 앞다리를 늘임, 왼쪽이 들리면(roll +) 왼다리를 줄임
        return out

    def get_tilt(self):
        """(옆 기울기 roll, 앞뒤 기울기 pitch) [도]. 실물은 IMU 를 달기 전까지 (0, 0)."""
        r, p, _ = np.degrees(self.hw.rpy())
        return float(r), float(p)

    def get_position(self):
        """(시뮬레이터 전용) 바닥 위 위치 (x, y) [m]."""
        if self.mode != "sim":
            raise RuntimeError("실물은 위치를 잴 센서가 없어요 (시뮬레이터 전용 함수)")
        x, y, _ = self.hw.base_pos
        return float(x), float(y)

    def get_heading(self):
        """(시뮬레이터 전용) 바라보는 방향 [도]."""
        if self.mode != "sim":
            raise RuntimeError("실물은 방향을 잴 센서가 없어요 (시뮬레이터 전용 함수)")
        return float(np.degrees(self.hw.rpy()[2]))

    def get_temperatures(self):
        """{다리: (hip_x, hip_y, knee)} 모터 온도 [°C]. 시뮬레이터 값은 교육용 어림값."""
        t = np.asarray(self.hw.temperature, float).reshape(4, 3)
        return {leg: tuple(round(float(v), 1) for v in t[i]) for i, leg in enumerate(C.LEGS)}

    def relax(self):
        """(실물) 모터 힘을 뺀다. 서 있으면 주저앉으니 먼저 엎드리게 한 다음 쓴다."""
        if self.mode == "real":
            self.hw.torque_off()

    # ---------- 내부 ----------
    def _angles(self):
        return -np.degrees(self.hw.joint_pos)

    def _targets(self):
        return -np.degrees(self.hw.q_target)

    def _send(self, q_deg):
        self.hw.set_joint_targets(-np.radians(np.asarray(q_deg, float)))   # 범위 밖 값은 백엔드가 잘라낸다

    def _pose_to_array(self, pose):
        if isinstance(pose, str):
            if pose not in C.POSES:
                raise ValueError(f"자세 이름 '{pose}' 은(는) 없어요. 쓸 수 있는 이름: {', '.join(C.POSES)}")
            pose = C.POSES[pose]
        if not isinstance(pose, dict):
            pose = {leg: pose for leg in C.LEGS}
        q = self._targets()
        for leg, angles in pose.items():
            i = _index(leg, "hip_x")
            q[i:i + 3] = angles
        return q

    def _hold(self, seconds):
        steps = int(round(seconds * 1000))
        while steps > 0:
            n = min(steps, int(CTRL_DT * 1000))
            self._advance(n)
            steps -= n

    def _advance(self, steps):
        if self._live:
            self._live.advance(steps)
        else:
            self.hw.step(steps)
            if self.mode == "sim":
                self._sim_time += steps * 0.001
                if self.viewer:
                    if not self.viewer.is_running():
                        raise SystemExit("화면을 닫아서 프로그램을 종료합니다.")
                    self.viewer.sync()
                    lag = self._sim_time - (time.perf_counter() - self._wall_start)
                    if lag > 0:
                        time.sleep(lag)
        self._check_temperature()

    def _check_temperature(self):
        t = np.asarray(self.hw.temperature, float)
        limit = np.array([self.hw.temp_limit[i] for i in range(12)]) if hasattr(self.hw, "temp_limit") else np.full(12, 70.0)
        if np.any(t >= limit):
            i = int(np.argmax(t - limit))
            name = f"{C.LEGS[i // 3]} {C.JOINTS[i % 3]}"
            raise RuntimeError(f"{name} 모터 온도 {t[i]:.0f}°C — 한계를 넘었어요. 엎드려서 식혀 주세요.")
        if not self._warned and np.any(t >= limit - TEMP_WARN_MARGIN):
            i = int(np.argmax(t - limit))
            print(f"⚠️ {C.LEGS[i // 3]} {C.JOINTS[i % 3]} 모터가 뜨거워요 ({t[i]:.0f}°C). 오래 서 있지 말고 쉬게 해 주세요.")
            self._warned = True

    def _keep_open(self):
        if not self.viewer.is_running():
            return
        print("프로그램 끝! 창을 닫으면 종료됩니다.")
        try:
            while True:
                self._advance(int(CTRL_DT * 1000))
        except SystemExit:
            pass

    def _goodbye(self):
        self.hw.close(torque_off=False)
        print("연결을 닫았어요. 모터는 마지막 자세를 계속 잡고 있어요 (힘을 빼려면 dog.relax()).")
