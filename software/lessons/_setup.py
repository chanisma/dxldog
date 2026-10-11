"""예제 공통: dxldog 패키지를 불러오고, 실행 옵션에 따라 시뮬레이터/실물과 로봇 종류를 고른다.

  python j1_one_motor.py                 시뮬레이터 (3D 창)
  python j1_one_motor.py --no-view       시뮬레이터, 창 없이 빠르게 (시험용)
  python j1_one_motor.py --real COM3     실물 (노트북 + 어댑터).  Raspberry Pi: --real /dev/ttyUSB0
  python j1_one_motor.py --real fake     가짜 모터로 실물 코드 경로 시험
  python j1_one_motor.py --robot sts     로봇 종류: mixed(DYNAMIXEL) 또는 sts(STS3215). 생략하면 기본 종류
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import dxldog  # noqa: E402
from dxldog import LEGS, forward_kinematics, inverse_kinematics, trot_feet  # noqa: E402,F401

VIEW = "--no-view" not in sys.argv
PORT = sys.argv[sys.argv.index("--real") + 1] if "--real" in sys.argv else None
ROBOT = sys.argv[sys.argv.index("--robot") + 1] if "--robot" in sys.argv else None


def Dog():
    """옵션에 맞는 로봇을 만든다. 예제 코드에서는 그냥 dog = Dog() 로 쓴다."""
    return dxldog.Dog("real", port=PORT, robot=ROBOT) if PORT else dxldog.Dog("sim", viewer=VIEW, robot=ROBOT)
