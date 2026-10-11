# Raspberry Pi를 로봇에 싣기

노트북 + U2D2 케이블로 실물이 잘 움직이면, 같은 코드를 로봇에 실은 Raspberry Pi에서 돌린다. 노트북은 Wi-Fi로 Pi에 접속해 **브라우저로 스튜디오를 쓰거나** SSH로 예제를 실행한다.

```
 [3S 배터리 또는 12 V 전원] → U2D2 Power Hub ─┬─ 모터 12개 (12 V)
                                              └─ 5 V 강압 변환기 → Raspberry Pi ─ USB ─ U2D2 ─ 모터 신호선
 노트북 ── Wi-Fi ── Pi:  http://dxldog.local:8765  (DXL Dog Studio)  /  ssh pi@dxldog.local
```

## 1. 준비물
| 품목 | 비고 |
|---|---|
| Raspberry Pi 5 (4 GB 이상) 또는 Pi 4 | 윗판의 58 × 49 mm 구멍에 M2.5 스페이서로 고정 |
| microSD 32 GB | Raspberry Pi OS (64-bit) |
| 5 V 강압 변환기 | 12 V 입력. **Pi 5는 5 V 5 A, Pi 4는 5 V 3 A** 이상 ⚠️ 전류가 모자라면 Pi가 저전압 경고를 내고 느려진다 |
| U2D2 (mixed) 또는 STS 어댑터 (sts) | 노트북에서 쓰던 것을 그대로 Pi USB에 꽂는다. STS 로봇은 Feetech URT-1 또는 Waveshare 버스 서보 어댑터 |

⚠️ Pi 전원은 모터 전원과 같은 배터리에서 나오지만, **접지(GND)는 공유되고 12 V는 절대 Pi에 바로 들어가면 안 된다.** 변환기 출력 전압을 멀티미터로 먼저 확인한다.

## 2. SD 카드 만들기 (노트북에서)
1. Raspberry Pi Imager → OS: **Raspberry Pi OS (64-bit)**
2. 설정(톱니바퀴): 호스트 이름 `dxldog`, 사용자 이름·비밀번호, **Wi-Fi**(학교 네트워크 또는 휴대폰 핫스팟), **SSH 켜기**
3. SD를 Pi에 꽂고 전원을 켠 뒤 1–2분 기다린다. 노트북에서 `ssh <사용자>@dxldog.local`

## 3. 소프트웨어 설치 (Pi에서)
```bash
sudo apt update && sudo apt install -y python3-venv git
# 프로젝트 가져오기: git clone (비공개 저장소면 토큰 필요) 또는 노트북에서 복사
scp -r software <사용자>@dxldog.local:~/dxldog        # ← 노트북에서 실행하는 경우
cd ~/dxldog
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt            # numpy, mujoco, dynamixel-sdk, feetech-servo-sdk, aiohttp
.venv/bin/python -m dxldog.tools use sts             # STS3215 로봇이면 한 번 (기본 종류 저장, 이후 --robot 생략 가능)
```
- MuJoCo는 Pi(64-bit)에서도 설치된다. 실물을 움직일 때는 스튜디오 3D 화면(디지털 트윈)에만 쓴다.

## 4. U2D2 권한과 지연 시간 (한 번만)
```bash
sudo usermod -aG dialout $USER          # 시리얼 포트 권한 (Raspberry Pi OS 기본 사용자는 이미 있음)
echo 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"' | \
  sudo tee /etc/udev/rules.d/99-dynamixel.rules
sudo reboot
```
다시 접속해서 `cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer` 가 `1`이면 된다.

**STS 어댑터 (sts 로봇)**: 권한은 위와 같이 `dialout` 그룹이면 된다. 포트 이름은 어댑터의 USB 칩에 따라 다르다 —
`ls /dev/ttyACM* /dev/ttyUSB*` 로 꽂기 전·후를 비교한다 (Waveshare 어댑터는 보통 `/dev/ttyACM0`, ⚠️ 칩에 따라 `/dev/ttyUSB0`).
위의 FTDI 지연 규칙은 U2D2(FTDI 칩)용이라 STS 어댑터(CH340/CH343 계열 ⚠️)에는 해당하지 않는다. 아래 명령의 `/dev/ttyUSB0` 을 그 이름으로 바꿔 쓴다.

## 5. 확인과 영점 보정
보정값은 **로봇을 움직이는 컴퓨터**의 `~/.dxldog/calibration.json`(sts 로봇은 `calibration_sts.json`)에 저장된다. 노트북에서 보정했다면 그 파일을 Pi의 같은 위치로 복사하거나, Pi에서 다시 보정한다.
```bash
cd ~/dxldog
.venv/bin/python -m dxldog.tools scan --port /dev/ttyUSB0
.venv/bin/python -m dxldog.tools calibrate --port /dev/ttyUSB0     # 또는 노트북의 파일 복사
.venv/bin/python -m dxldog.tools check --port /dev/ttyUSB0
```

## 6. 실행
**예제 (SSH 터미널에서)**
```bash
.venv/bin/python lessons/j2_poses.py --real /dev/ttyUSB0
```

**스튜디오 (노트북 브라우저로 조종)**
```bash
chmod +x studio/run_studio.sh
studio/run_studio.sh --backend real --port /dev/ttyUSB0 --host 0.0.0.0 --no-window
```
노트북 브라우저에서 **http://dxldog.local:8765/** (또는 터미널에 찍힌 Pi 주소)를 연다. 코드 상자에 쓴 코드는 **Pi에서 실행**되고, 3D 화면은 실제 관절 각도를 따라 움직인다(기울기 센서가 없어서 몸 기울기·높이는 '계산' 값).

⚠️ 스튜디오는 같은 네트워크의 누구나 열 수 있다. 수업용 네트워크나 핫스팟에서만 `--host 0.0.0.0`을 쓴다.

## 7. 끄기
엎드리게 한 뒤 `python -m dxldog.tools relax --port /dev/ttyUSB0` → `sudo shutdown -h now` → 초록 LED가 꺼지면 배터리를 뺀다.
