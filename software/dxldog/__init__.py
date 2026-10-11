"""DXL Dog — DYNAMIXEL X430 사족보행 로봇 교육용 패키지 (시뮬레이터·실물 같은 코드)."""
from .config import JOINTS, LEGS, POSES  # noqa: F401
from .dog import Dog, deg_to_tick, forward_kinematics, inverse_kinematics, motor_info, trot_feet  # noqa: F401
