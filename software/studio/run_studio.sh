#!/bin/sh
# DXL Dog Studio 실행 (Linux / Raspberry Pi)
#   ./run_studio.sh                                       시뮬레이터 (화면이 있으면 앱 창 또는 브라우저)
#   ./run_studio.sh --backend real --port /dev/ttyUSB0    실물 (U2D2)
#   ./run_studio.sh --backend real --port /dev/ttyUSB0 --host 0.0.0.0 --no-window
#        Raspberry Pi 에서 화면 없이 켜고, 같은 와이파이의 노트북 브라우저로 http://<Pi 주소>:8765/ 접속
# 필요한 것: pip install mujoco numpy aiohttp dynamixel-sdk   (앱 창을 쓰려면 pywebview)
# 시리얼 포트 권한: sudo usermod -aG dialout $USER  (한 번, 다시 로그인)
HERE=$(cd "$(dirname "$0")" && pwd)
PY=python3
for v in "$HERE/../../.venv/bin/python" "$HERE/../.venv/bin/python" "$HERE/.venv/bin/python"; do
  if [ -x "$v" ]; then PY="$v"; break; fi
done
exec "$PY" "$HERE/studio_web.py" "$@"
