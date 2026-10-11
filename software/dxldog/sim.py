"""DXL Dog MuJoCo 시뮬레이터 — CAD 에서 뽑은 부품 모양·질량(assets/<로봇 종류>/)과 모터별 최대 힘으로 만든다.

모델 관절각 규칙은 config.py 참고 (학생 각도 = −모델 각도).
모터 = 위치 제어(P·D) + 토크-속도 한계(정지 토크에서 무부하 속도까지 직선). 빨리 돌수록 낼 수 있는 힘이 줄어든다.
모터 온도는 '교육용 어림 모델'이다: dT/dt = 0.5·(토크/정지토크)² − (T−25)/300  ⚠️ 실제 모터와 수치가 다를 수 있음.
"""
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from . import config as C

ASSETS_ROOT = Path(__file__).resolve().parent / "assets"           # assets/mixed, assets/sts … (cad/export_sim_assets.py 가 만듦)
SIM_DT = 0.001
EXTRA = {"battery": 0.15, "electronics": 0.10, "wiring": 0.10}      # kg — 3S 1300 mAh, Raspberry Pi·어댑터·허브, 케이블·나사
# 위치 제어 P·D (Nm/rad, Nm·s/rad) — 모델별. ⚠️ STS3215 값은 정지 토크 비율로 고른 어림값
KP = {"XL430-W250": 25.0, "XM430-W350": 40.0, "XM430-W210": 30.0, "XC430-W240": 25.0,
      "STS3215-12V": 30.0, "STS3215-7.4V": 20.0}
KV = {"XL430-W250": 0.5, "XM430-W350": 0.8, "XM430-W210": 0.6, "XC430-W240": 0.5,
      "STS3215-12V": 0.6, "STS3215-7.4V": 0.4}
T_AMB, HEAT_GAIN, COOL_TAU = 25.0, 0.5, 300.0
TEMP_LIMIT = C.TEMP_LIMIT                                          # 모터 기본 온도 한계 (°C, config.py)


def assets_dir(robot=None):
    """로봇 종류의 자산 폴더. 아직 없으면 (경고하고) mixed 폴더를 쓴다 → (폴더, 대신 쓰는지)."""
    robot = robot or C.ROBOT
    d = ASSETS_ROOT / robot
    if (d / "frames.json").exists():
        return d, False
    print(f"⚠️ assets/{robot}/ 가 아직 없어서 mixed 로봇의 부품 모양을 대신 씁니다 (모터 무게·힘은 {robot} 값).")
    return ASSETS_ROOT / "mixed", True


def _student_to_model(deg):
    return -np.radians(deg)


def build_model(robot=None):
    """MuJoCo 모델 만들기. robot: 로봇 종류 (None = 지금 종류, 주면 그 종류로 바꾼다)."""
    if robot and robot != C.ROBOT:
        C.use(robot)                                               # 모터 힘·무게도 같은 종류로 맞춘다
    assets, fallback = assets_dir()
    info = json.loads((assets / "frames.json").read_text(encoding="utf-8"))
    if fallback:                                                   # 대신 쓰는 메시: 모터 무게만 이 로봇 모터로
        info["motor_mass"] = {j: C.MOTOR_MASS[C.MOTOR_MODEL[j]] for j in C.JOINTS}
    pm = info["part_mass"]
    spec = mujoco.MjSpec()
    spec.compiler.degree = False
    spec.option.timestep = SIM_DT
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.visual.global_.offwidth, spec.visual.global_.offheight = 1280, 960
    spec.visual.headlight.diffuse = [0.7, 0.7, 0.7]
    spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
    for name in ("body_top", "body_bottom", "shoulder_L", "shoulder_R", "thigh_L", "thigh_R", "shank_L", "shank_R", "foot"):
        spec.add_mesh(name=name, file=str(assets / f"{name}.stl"), scale=[1e-3] * 3)
    wb = spec.worldbody
    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.1], rgba=[0.85, 0.87, 0.9, 1],
                friction=[1.0, 0.005, 0.0001])
    torso = wb.add_body(name="torso", pos=[0, 0, 0.25])
    torso.add_freejoint(name="root")
    torso.add_site(name="imu")
    orange, white, dark = [0.96, 0.62, 0.16, 1], [0.93, 0.93, 0.95, 1], [0.16, 0.17, 0.19, 1]
    for name in ("body_top", "body_bottom"):
        torso.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=name, mass=pm[name], rgba=white, contype=0, conaffinity=0)
    torso.add_geom(name="body_collision", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.111, 0.048, 0.0174], group=3, mass=0.0001)
    torso.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.037, 0.017, 0.010], mass=EXTRA["battery"], rgba=[0.2, 0.3, 0.8, 1],
                   contype=0, conaffinity=0)
    torso.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.043, 0.030, 0.010], pos=[0, 0, 0.028], mass=EXTRA["electronics"],
                   rgba=[0.1, 0.5, 0.25, 1], contype=0, conaffinity=0)
    torso.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.05, 0.02, 0.005], mass=EXTRA["wiring"], rgba=dark, contype=0, conaffinity=0)

    hx0, hy0, kn0 = (np.array(info[k]) for k in ("hip_x_axis", "hip_y_axis", "knee_axis"))
    rng = {j: tuple(sorted(_student_to_model(np.array(C.JOINT_RANGE_DEG[j])))) for j in C.JOINTS}
    half = np.array(info["motor_box"]) / 2
    cy = info["motor_box_center_y"]

    def motor_box(body, joint, side, origin_body):
        f = info["motor_frames_FL"][joint]
        o, x, y = np.array(f["origin"]), np.array(f["x"]), np.array(f["y"])
        if side == "R":                                  # 거울: y 성분 뒤집고, 오른손 좌표 유지하려 x 축도 뒤집음
            o, x, y = o * [1, -1, 1], -(x * [1, -1, 1]), y * [1, -1, 1]
        center = o + y * cy - origin_body
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=list(half), pos=list(center),
                      xyaxes=list(x) + list(y), mass=info["motor_mass"][joint], rgba=dark, contype=0, conaffinity=0)

    for leg in C.LEGS:
        side = leg[1]
        sy = 1 if side == "L" else -1
        shift = np.array([info["rear_shift"] if leg[0] == "R" else 0.0, 0, 0])
        mir = np.array([1, sy, 1])
        p_hx, p_hy, p_kn = hx0 * mir, hy0 * mir, kn0 * mir
        motor_box(torso, "hip_x", side, -shift)            # 몸통 안 hip_x 모터 (몸통 좌표 = 조립 좌표 + 뒤 이동)
        sh = torso.add_body(name=f"{leg}_shoulder", pos=list(p_hx + shift))
        sh.add_joint(name=f"{leg}_hip_x", axis=[1, 0, 0], range=list(rng["hip_x"]), damping=0.05, armature=0.01)
        sh.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"shoulder_{side}", pos=list(-p_hx), mass=pm[f"shoulder_{side}"],
                    rgba=orange, contype=0, conaffinity=0)
        motor_box(sh, "hip_y", side, p_hx)
        th = sh.add_body(name=f"{leg}_thigh", pos=list(p_hy - p_hx))
        th.add_joint(name=f"{leg}_hip_y", axis=[0, 1, 0], range=list(rng["hip_y"]), damping=0.05, armature=0.01)
        th.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"thigh_{side}", pos=list(-p_hy), mass=pm[f"thigh_{side}"],
                    rgba=orange, contype=0, conaffinity=0)
        motor_box(th, "knee", side, p_hy)
        sk = th.add_body(name=f"{leg}_shank", pos=list(p_kn - p_hy))
        sk.add_joint(name=f"{leg}_knee", axis=[0, 1, 0], range=list(rng["knee"]), damping=0.05, armature=0.01)
        sk.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"shank_{side}", pos=list(-p_kn), mass=pm[f"shank_{side}"],
                    rgba=orange, contype=0, conaffinity=0)
        foot_shift = np.array([0, 0 if side == "L" else -2 * hy0[1], 0])     # 발 메시는 왼쪽 좌표 → 오른쪽은 y 로 옮김
        sk.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="foot", pos=list(-p_kn + foot_shift), mass=pm["foot"],
                    rgba=[0.1, 0.1, 0.1, 1], contype=0, conaffinity=0)
        sk.add_geom(name=f"{leg}_foot", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.012, 0, 0], pos=[0, 0, -C.SHANK],
                    mass=0.0001, friction=[1.0, 0.005, 0.0001], group=3)
    for leg in C.LEGS:
        for j in C.JOINTS:
            model = C.MOTOR_MODEL[j]
            a = spec.add_actuator(name=f"{leg}_{j}", target=f"{leg}_{j}", trntype=mujoco.mjtTrn.mjTRN_JOINT)
            a.ctrllimited = True                                  # 토크 명령 (위치 제어는 DxlSim.step 에서 계산)
            a.ctrlrange = [-C.STALL_TORQUE[model], C.STALL_TORQUE[model]]
    return spec.compile()


class DxlSim:
    """시뮬레이터 백엔드. 관절각·목표는 '모델 rad' (학생 각도 변환은 Dog 가 한다)."""

    kind = "sim"
    has_imu = True                                               # 시뮬레이터는 기울기·자이로를 정확히 안다

    def __init__(self, robot=None):
        if robot:
            C.use(robot)                                             # 로봇 종류 바꾸기 (모터 사양·자산 폴더)
        self.robot = C.ROBOT
        self.model = build_model()
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.qadr = np.array([m.jnt_qposadr[m.joint(f"{l}_{j}").id] for l in C.LEGS for j in C.JOINTS])
        self.dadr = np.array([m.jnt_dofadr[m.joint(f"{l}_{j}").id] for l in C.LEGS for j in C.JOINTS])
        self.stall = np.array([C.STALL_TORQUE[C.MOTOR_MODEL[j]] for _ in C.LEGS for j in C.JOINTS])
        self.temp_limit = np.array([C.TEMP_LIMIT[C.MOTOR_MODEL[j]] for _ in C.LEGS for j in C.JOINTS])
        self.w0 = np.array([C.NO_LOAD_RPM[C.MOTOR_MODEL[j]] * 2 * math.pi / 60 for _ in C.LEGS for j in C.JOINTS])
        self.kp = np.array([KP[C.MOTOR_MODEL[j]] for _ in C.LEGS for j in C.JOINTS])
        self.kv = np.array([KV[C.MOTOR_MODEL[j]] for _ in C.LEGS for j in C.JOINTS])
        jid = [m.joint(f"{l}_{j}").id for l in C.LEGS for j in C.JOINTS]
        self.lower, self.upper = m.jnt_range[jid, 0].copy(), m.jnt_range[jid, 1].copy()
        self.reset()

    def reset(self, pose="lie"):
        mujoco.mj_resetData(self.model, self.data)
        q = _student_to_model(np.array(pose_degrees(pose)))
        self.data.qpos[self.qadr] = q
        self.q_des = q.copy()
        self.data.qpos[2] = 0.30
        self.temperature = np.full(12, T_AMB)
        mujoco.mj_forward(self.model, self.data)
        f = self.feet_world() - self.data.qpos[0:3]                  # 몸통을 기울여 네 발이 같은 높이가 되게 (앉기 등)
        pitch = math.atan2(f[:2, 2].mean() - f[2:, 2].mean(), f[:2, 0].mean() - f[2:, 0].mean())
        roll = -math.atan2(f[[0, 2], 2].mean() - f[[1, 3], 2].mean(), f[[0, 2], 1].mean() - f[[1, 3], 1].mean())
        qy = [math.cos(pitch / 2), 0, math.sin(pitch / 2), 0]
        qx = [math.cos(roll / 2), math.sin(roll / 2), 0, 0]
        mujoco.mju_mulQuat(self.data.qpos[3:7], np.array(qy), np.array(qx))
        mujoco.mj_forward(self.model, self.data)
        self.data.qpos[2] -= self._lowest_point() - 0.0005         # 가장 낮은 곳(발 또는 몸통)이 바닥에 닿게 내려 놓기
        mujoco.mj_forward(self.model, self.data)
        self.step(300)                                             # 자리 잡기
        self.temperature = np.full(12, T_AMB)

    def _lowest_point(self):
        m, d = self.model, self.data
        z = [d.geom_xpos[m.geom(f"{l}_foot").id][2] - 0.012 for l in C.LEGS]
        g = m.geom("body_collision").id
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    z.append((d.geom_xpos[g] + d.geom_xmat[g].reshape(3, 3) @ (m.geom_size[g] * [sx, sy, sz]))[2])
        return min(z)

    # 상태
    @property
    def joint_pos(self):
        return self.data.qpos[self.qadr].copy()

    @property
    def joint_vel(self):
        return self.data.qvel[self.dadr].copy()

    @property
    def q_target(self):
        return self.q_des.copy()

    @property
    def torque(self):
        return self.data.actuator_force.copy()

    @property
    def base_pos(self):
        return self.data.qpos[0:3].copy()

    def rpy(self):
        w, x, y, z = self.data.qpos[3:7]
        return np.array([math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
                         math.asin(max(-1, min(1, 2 * (w * y - z * x)))),
                         math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))])

    def gyro(self):
        """몸통 회전 속도 (roll, pitch, yaw 축, rad/s) — 자이로 센서 값."""
        return self.data.qvel[3:6].copy()                        # 자유 관절의 회전 속도는 몸통 좌표 기준

    def fallen(self):
        r, p, _ = self.rpy()
        return abs(r) > math.radians(45) or abs(p) > math.radians(45)

    # 명령
    def set_joint_targets(self, q):
        self.q_des = np.clip(np.asarray(q, float), self.lower, self.upper)

    def step(self, n=1):
        d = self.data
        for _ in range(n):
            q, qd = d.qpos[self.qadr], d.qvel[self.dadr]
            tau = self.kp * (self.q_des - q) - self.kv * qd
            hi = np.where(qd > 0, self.stall * (1 - qd / self.w0), self.stall)      # 도는 방향으로는 빠를수록 힘이 약함
            lo = np.where(qd < 0, -self.stall * (1 + qd / self.w0), -self.stall)
            d.ctrl[:] = np.clip(tau, np.minimum(lo, 0), np.maximum(hi, 0))
            mujoco.mj_step(self.model, d)
        load = self.torque / self.stall
        self.temperature += n * SIM_DT * (HEAT_GAIN * load ** 2 - (self.temperature - T_AMB) / COOL_TAU)

    # 무게중심 (조작 패널용)
    def com(self):
        return self.data.subtree_com[self.model.body("torso").id].copy()

    def feet_world(self):
        return np.array([self.data.geom_xpos[self.model.geom(f"{l}_foot").id] for l in C.LEGS])


def pose_degrees(pose):
    """자세 이름 또는 {다리: (…)} / (…) → 학생 각도 12개 (LEGS × JOINTS 순서)."""
    if isinstance(pose, str):
        pose = C.POSES[pose]
    if not isinstance(pose, dict):
        pose = {leg: pose for leg in C.LEGS}
    return [float(a) for leg in C.LEGS for a in pose[leg]]
