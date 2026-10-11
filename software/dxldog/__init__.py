"""DXL Dog — 사족보행 로봇 교육용 패키지 (시뮬레이터·실물 같은 코드). 모터: DYNAMIXEL X430("mixed") 또는 Feetech STS3215("sts")."""
from .config import JOINTS, LEGS, POSES  # noqa: F401
from .dog import Dog, deg_to_tick, forward_kinematics, inverse_kinematics, motor_info, trot_feet  # noqa: F401
