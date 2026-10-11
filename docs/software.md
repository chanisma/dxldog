# 소프트웨어: 시뮬레이터와 실물에서 같은 코드

## 1. 한눈에 보기
학생 코드는 `Dog` 하나만 쓴다. 어디서 돌릴지는 **첫 줄 하나**로 정한다.

```python
from dxldog import Dog
dog = Dog()                               # ① 시뮬레이터 (노트북, 3D 창)
dog = Dog("real", port="COM3")            # ② 노트북 + U2D2 케이블 → 실물
dog = Dog("real", port="/dev/ttyUSB0")    # ③ 로봇에 실은 Raspberry Pi → 실물
dog = Dog("real", port="fake")            #    하드웨어 없이 실물 코드 경로 연습 (가짜 모터)
dog = Dog(robot="sts")                    #    로봇 종류 고르기 (생략하면 기본 종류, 아래 1-1)

dog.reset()
dog.move_to("stand", 1.5)
dog.set_angle("FL", "knee", 100)
```

```
                 ┌─ DxlSim  (MuJoCo: CAD 모양·질량, 모터 힘·속도 한계, 온도 어림)
학생 코드 ─ Dog ─┤
                 └─ DxlReal ─┬ DxlBus (dynamixel_sdk, Protocol 2.0) ─ U2D2 ─ DYNAMIXEL 12개   ← "mixed"
                             ├ StsBus (scservo_sdk) ─ URT-1·Waveshare 어댑터 ─ STS3215 12개     ← "sts"
                             └ FakeBus (가짜 모터, 지금 로봇 종류의 레지스터를 흉내, 시험용)
```

### 1-1. 로봇 종류 (같은 몸, 다른 모터)
| 종류 | 모터 | 어댑터 | 정지 토크 · 무부하 속도 (12 V) |
|---|---|---|---|
| `mixed` (기본) | hip_x XL430-W250, hip_y·knee XM430-W350 | U2D2 | XL 1.5 Nm·57 rpm, XM 4.1 Nm·46 rpm |
| `sts` | Feetech **STS3215 12 V판** × 12 (LeRobot SO-101 모터) | Feetech URT-1 또는 Waveshare 버스 서보 어댑터 | 2.94 Nm(30 kg·cm)·45 rpm, 55 g |

학생 코드는 **똑같고** 종류만 고른다. 고르는 방법(앞의 것이 우선):
1. 코드 `Dog(robot="sts")` / 예제 `python lessons/j1_one_motor.py --robot sts` / 스튜디오 `run_studio.bat --robot sts` / 도구 `--robot sts`
2. 환경 변수 `DXLDOG_ROBOT=sts`
3. 기본값 저장: `python -m dxldog.tools use sts` → `~/.dxldog/robot.json` (`{"robot": "sts"}`)
4. 아무것도 없으면 `mixed`

시뮬레이터는 `dxldog/assets/<종류>/`의 메시·질량을 쓰고(폴더가 없으면 경고 후 mixed 메시), 모터 힘·속도·온도 한계는 그 종류의 모터 값이다. 보정값 파일도 종류별이다: mixed `~/.dxldog/calibration.json`, sts `~/.dxldog/calibration_sts.json`.

robotdog(Lite3) 과정의 `JointDog`와 **함수 이름·각도 규칙이 같다**. 그래서 J1–J5에서 쓴 코드를 거의 그대로 옮길 수 있다. 바뀌는 것은 다리 이름(`HL/HR` → `RL/RR`)과 숫자(링크 길이, 서기 높이)뿐이다.

## 2. 폴더
```
software/
  dxldog/          패키지 (Raspberry Pi에는 이 폴더와 requirements.txt만 있으면 됨)
    config.py      다리·관절 이름, 모터 ID·방향, 관절 범위, 자세, 로봇 종류(ROBOTS)별 모터·컨트롤 테이블, use()
    dog.py         학생 API: Dog, forward/inverse_kinematics, trot_feet, deg_to_tick, motor_info
    sim.py         시뮬레이터 백엔드 DxlSim
    real.py        실물 백엔드 DxlReal, DxlBus(dynamixel_sdk), StsBus(scservo_sdk), FakeBus
    tools.py       실물 준비 도구 (ID 설정, 확인, 영점 보정, 방향 확인)
    live.py        스튜디오 연결용
    assets/mixed/  CAD에서 뽑은 부품 메시(STL)·관절 위치·질량 (cad/export_sim_assets.py가 만듦, 지금은 v2 모양: 덮개 cover·cover_dark 포함)
    assets/sts/    STS3215 로봇용 (같은 형식)
  lessons/         예제 J1–J6 (시뮬·실물 같은 파일, J6 균형 잡기). 옵션 --real <포트>, --robot sts
  tests/           run_lessons.py (예제 전부: 종류 × 시뮬·가짜 모터), test_buses.py (StsBus 바이트·부호, FakeBus 규칙)
  studio/          DXL Dog Studio (웹 화면: 코드 + 3D + 조작 패널)
  requirements.txt 로봇 소프트웨어용 패키지만
```

## 3. 학생 API (`Dog`)
| 함수 | 하는 일 | 시뮬 | 실물 |
|---|---|---|---|
| `set_angle(leg, joint, deg)` / `get_angle(leg, joint)` | 관절 하나 목표 주기 / 현재 각도 읽기 | ✓ | ✓ (Goal Position / Present Position, 주소는 6-1) |
| `get_angles()` | 12개 각도 `{다리: (hip_x, hip_y, knee)}` | ✓ | ✓ (12개 동시 읽기) |
| `move_to(pose, seconds)` | 자세로 부드럽게 이동 (`"lie"`, `"stand"`, `"sit"`, `"zero"`, 튜플, 딕셔너리) | ✓ | ✓ (50 Hz로 목표를 나눠 보냄) |
| `set_feet({다리: (x, z)}, seconds)` / `get_foot(leg)` | 역기구학 / 정기구학 | ✓ | ✓ |
| `wait(s)`, `reset()` | 기다리기 / 처음 자세 | ✓ (바로 되돌림) | ✓ (2초 동안 엎드림) |
| `get_height()` | 몸 높이 | 측정값 | 관절 각도로 계산 |
| `get_tilt()` | (roll, pitch) | ✓ | IMU 달기 전에는 (0, 0) |
| `get_position()`, `get_heading()` | 바닥 위 위치·방향 | ✓ | ✗ (센서 없음) |
| `get_gyro()` | 몸이 도는 속도 (roll, pitch, yaw) [°/s] | ✓ | IMU 달기 전에는 (0, 0, 0) |
| `set_balance(True, gain=0.6, damping=0.007)` | 균형 잡기: `set_feet(…, 0)`의 발 높이를 센서로 고침 | ✓ | IMU 필요 |
| `get_temperatures()` | 모터 온도 | 교육용 어림값 | Present Temperature (DXL 146, STS 63) |
| `relax()` | 모터 힘 빼기 | – | ✓ |

다른 도구: `motor_info("FL", "knee")` → `(3, "XM430-W350")` (STS 로봇은 `(3, "STS3215-12V")`), `deg_to_tick("FL", "knee", 99)` → Goal Position 값 (두 종류 같은 식).

## 4. 각도와 모터 위치값
- 학생 각도(도): hip_x +면 발이 오른쪽, hip_y +면 허벅지가 앞, knee는 클수록 많이 굽힘. 모두 0°가 **다리 똑바로 아래**(조립 자세).
- 모터 위치값(0–4095) = `2048 + 방향 × (−각도) × 4096/360 + 보정값`
  - 방향: hip_x는 네 다리 모두 +1, hip_y·knee는 왼쪽 +1, 오른쪽 −1 (오른쪽 모터는 거울 방향으로 달려 있음)
  - 보정값: 조립 오차를 영점 보정(`tools calibrate`)으로 잰 값. `~/.dxldog/calibration.json`에 저장된다.
- 서기 자세 예: 앞왼쪽 무릎 96.6° → 2048 − 1099 = **949**, 앞오른쪽 무릎 → **3147**
- 두 종류 모두 한 바퀴 4096, 가운데 2048이라 위치값 식이 같다. ⚠️ STS3215의 + 회전 방향이 DYNAMIXEL과 같은지는 미확인 → 조립 후 `tools check --robot sts`로 확인하고, 반대면 `config.ROBOTS["sts"]["dir_sign"]`을 −1로 바꾼다.

## 5. 시뮬레이터 (`DxlSim`)
- 모양·질량: CAD 부품(STL)과 부품별 질량(채움 55 % 어림), 모터 무게(XL430 65 g, XM430 82 g, 공식 사양), 배터리 150 g, Pi·U2D2 100 g, 배선 100 g → **전체 1.78 kg**
- 모터: 위치 제어(P·D) + **토크-속도 한계**(정지 토크에서 무부하 속도까지 직선, 12 V 공식 사양). 빨리 돌수록 힘이 약해지므로 너무 빠른 걸음은 시뮬에서도 따라가지 못한다.
- 온도: `dT/dt = 0.5·(토크/정지토크)² − (T−25)/300` — **교육용 어림 모델**이다 ⚠️ 실제 값은 실물로 재서 비교하는 실험 거리로 쓴다.
- 확인한 결과: 서기 몸 높이 16.6 cm(설계 15.5 cm + 발 1.2 cm), 앉기 pitch −27.6°, 트롯 직진 약 14 cm/s(방향 오차 1° 이내), 돌기 약 15°/s.
- STS 로봇(`--robot sts`): 전체 1.55 kg(모터 55 g × 12), 위치 제어 P 30 · D 0.6 ⚠️(어림값). 서기 몸 높이 16.6 cm, 서 있을 때 모터 힘은 정지 토크의 약 9 %, 기본 트롯 5초에 0.71 m(mixed 0.69 m), 넘어지지 않음.

### 걸음새에서 알아낸 것
- **무게중심이 엉덩이 중심보다 2.6 cm 뒤에 있다.** 네 다리 모두 무릎이 뒤로 굽어서, 서 있을 때 허벅지와 무릎 모터(82 g)가 엉덩이 뒤쪽에 온다.
  - 그래서 서기 자세의 발을 엉덩이보다 **3.5 cm 뒤**에 둔다: `stand = (0, −67.1, 96.6)` (SpotMicro 기본값 (0, −55.8, 99)은 발이 엉덩이 바로 아래).
  - 발을 엉덩이 바로 아래에 두고 걸으면 앞발이 땅을 거의 딛지 못하고(접촉 시간 14 %), 몸이 ±10° 흔들리며 방향이 틀어졌다.
- **딛는 시간 비율(duty) 0.65**: 0.5(순수 트롯)는 두 발로만 서는 시간이 길어 흔들림이 커지고, 발 위치가 1 cm만 달라져도 결과가 크게 바뀌었다. 0.65면 주기 0.4–0.6 s, 발 위치 −2…−5 cm, 보폭 −4…6 cm, 발 높이 2–4.5 cm, 모터 힘 60 %, 마찰 0.6에서도 넘어지지 않고 곧게 걸었다.
- 옮기는 발은 천천히 출발해서 천천히 도착하게(코사인) 움직인다. 스튜디오 걸음새는 첫 한 주기 동안 보폭을 0에서 키운다.
- 돌기: 오른발·왼발 보폭을 다르게(`turn`). 몸은 한 주기에 약 `step / duty` 만큼 나아간다.
- **기울기 센서 피드백**(J6, `set_balance`): 몸이 기운 쪽 다리를 늘이고 반대쪽을 줄인다. 고치는 양 [cm] = 0.6 × 기울기[°] + 0.007 × 기우는 속도[°/s]. duty 0.5 트롯, 10주기 결과(mixed):
  - v1 모양: 기울기 10.3° → 2.3°, 거리 0.33 → 0.84 m.
  - v2 모양(지금 시뮬레이터): 센서 없이도 0.83 m를 가서, 균형 잡기는 기울기만 4.3° → 2.2°로 줄인다(거리 0.86 m). 몸이 0.1 kg 무거워지고 덮개 무게가 앞뒤로 퍼져 덜 흔들리는 것으로 보인다 ⚠️ 추정.
  - 방향 잡기(왼발·오른발 보폭 차이 0.2 cm/°)를 더하면 20주기 방향 오차 1° 안팎.
  - gain 2.0 이상은 지나치게 고쳐서 오히려 뒤로 가고 앞뒤로 출렁인다. 시뮬 센서는 잡음·지연이 없으므로 실물에서는 값을 다시 맞춘다.
- ⚠️ 실물은 모터 강성·마찰·바닥이 시뮬과 다르다. 처음에는 받침대 위에서, 다음에는 느린 주기(T 0.6)부터 시험한다.

## 6. 실물 준비 순서
도구는 모두 `software` 폴더에서 `python -m dxldog.tools <명령> --port <포트> [--robot sts]`로 쓴다. 포트는 Windows `COM3` 같은 이름(`tools ports`로 확인), Raspberry Pi `/dev/ttyUSB0` (STS 어댑터는 `/dev/ttyACM0`일 수 있음).

| 순서 | 명령 | 하는 일 |
|---|---|---|
| 1 | `setup FL hip_x --port COM3` | **새 모터를 하나만 연결**하고 실행 → ID(1–12)·통신 속도 1 Mbps·응답 지연 0 설정, 가운데(2048)로 돌려 힘을 줌. 이 상태에서 부품을 '다리 똑바로 아래' 방향으로 끼운다. 12번 반복. STS는 broadcast ping이 없어 ID 0–252를 하나씩 찾고(약 10초), Lock을 풀고 쓴 뒤 다시 잠그며, 영점 이동(Homing Offset)을 0으로 지운다 ⚠️ |
| 2 | `scan --port COM3` | 모터 12개를 체인으로 연결한 뒤 → 응답, 종류(XL/XM/STS3215), 위치, 온도, 전압 확인 |
| 3 | `calibrate --port COM3` | 받침대에 올리고 다리를 똑바로 아래로 맞춘 다음 Enter → 영점 보정값 저장 + **모터 위치 한계(EEPROM)를 관절 범위로 설정** |
| 4 | `check --port COM3` | 관절을 하나씩 천천히 +10° 움직여 방향이 맞는지 묻는다 |
| 5 | `python lessons/j1_one_motor.py --real COM3` | 예제를 실물로 |

**USB 지연 줄이기 (한 번만, U2D2)**: U2D2는 USB 시리얼 칩(FTDI)의 기본 지연이 16 ms라서 12개 읽기가 느려진다. Windows는 장치 관리자 → 포트 → USB Serial Port → 속성 → 포트 설정 → 고급 → **대기 시간 타이머 1 ms**. Raspberry Pi는 [rpi_setup.md](rpi_setup.md)의 udev 규칙.

ID 표: FL 1·2·3, FR 4·5·6, RL 7·8·9, RR 10·11·12 (각 다리 hip_x·hip_y·knee 순서). mixed: hip_x는 XL430-W250, 나머지는 XM430-W350. sts: 12개 모두 STS3215.

### 6-1. DYNAMIXEL과 STS3215 레지스터 비교
버스 코드는 레지스터를 이름(`config.ADDR`)으로 부르고, 주소·크기·부호는 버스가 처리한다.

| 이름 (`ADDR`) | DYNAMIXEL X (Protocol 2.0) | STS3215 (SMS_STS) |
|---|---|---|
| model_number | 0 (2B) — XL430 1060, XM430 1020 | 3 (2B) — 777 |
| id / baud_rate / return_delay_time | 7 / 8 / 9 | 5 / 6 / 7 |
| 공장 설정 | ID 1, 57600 bps | ID 1, 1 Mbps (속도 코드 0) |
| operating_mode (위치 제어) | 11 = **3** | 33 = **0** |
| min / max_position_limit | 52 / 48 (4B) | 9 / 11 (2B) |
| temperature_limit | 31 (기본 XL 72 °C, XM 80 °C) | 13 (기본 70 °C ⚠️) |
| torque_enable | 64 | 40 |
| goal_position | **116 (4B)** | **42 (2B)** |
| 속도 제한 | profile_velocity 112 (0.229 rpm 단위) | goal_speed 46 (걸음/초 ⚠️, 0 = 최대 ⚠️) |
| present_position | 132 (4B) | 56 (2B) |
| present_input_voltage | 144 (2B, 0.1 V) | 62 (1B, 0.1 V) |
| present_temperature | 146 | 63 |
| present_current | 126 (2B) | 69 (2B) |
| EEPROM 쓰기 | 토크를 끈 뒤 (64 미만 주소) | 토크를 끄고 **Lock(55) = 0** → 쓰기 → Lock = 1 (40 미만 주소) |
| 음수 | 2의 보수 | 부호 비트 + 크기: 위치·속도 비트 15, Homing Offset 비트 11, 부하 비트 10 ⚠️ |
| SDK | `dynamixel-sdk` (`PacketHandler(2.0)`) | `feetech-servo-sdk` (`scservo_sdk`, `PacketHandler(0)`) |

⚠️ STS 속도 코드표: Feetech SCServo 라이브러리(4=115200, 5=76800, 6=57600, 7=38400)와 LeRobot 표(5=57600, 6=38400, 7=19200)가 다르다. 우리는 1 Mbps(코드 0, 두 표 같음)만 쓴다.
⚠️ STS Acceleration(41)은 건드리지 않는다(공장값). LeRobot은 254로 바꿔 쓴다 — 실물에서 움직임이 느리게 출발하면 확인할 것.

## 7. 노트북 연결 vs Raspberry Pi
| | ② 노트북 + U2D2 | ③ Raspberry Pi를 로봇에 |
|---|---|---|
| 연결 | USB 케이블이 로봇까지 | 케이블 없음 (전원 제외), Wi-Fi |
| 코드 실행 | 노트북 | Pi (노트북 브라우저로 스튜디오 접속, 또는 SSH) |
| 좋은 점 | 준비가 쉬움, 수업 1–3차시에 알맞음 | 자유롭게 걷기, 실제 로봇 구조 |
| 준비 | `pip install -r requirements.txt` | [rpi_setup.md](rpi_setup.md) |
| 포트 | `COM3` 등 | `/dev/ttyUSB0` (STS 어댑터는 `/dev/ttyACM0`일 수 있음) |

**같은 패키지, 같은 예제 파일**이다. 처음에는 ②로 시작하고, 걷기가 안정되면 ③으로 옮긴다.

## 8. DXL Dog Studio (웹 화면)
코드 상자 + 3D 로봇 + 조작 패널(① 관절 ② 발 위치 ③ 걸음새 ④ 모터)을 한 화면에서 쓴다. robotdog의 Lite3 Studio와 같은 구조다.
```powershell
software\studio\run_studio.bat                              # 시뮬레이터
software\studio\run_studio.bat --backend real --port COM3   # 실물 (노트북 + U2D2)
software\studio\run_studio.bat --backend real --port fake   # 가짜 모터 (연습)
software\studio\run_studio.bat --robot sts                  # STS3215 로봇 (시뮬·실물 모두 --robot 을 붙인다)
```
- 3D 화면 왼쪽 위에 로봇 종류 배지(DYNAMIXEL / STS3215)가 보인다.
- 실물 모드의 3D 화면은 **디지털 트윈**이다: 모터에서 읽은 관절 각도를 시뮬 모델에 그대로 넣어 보여 준다.
- ④ 모터 탭과 코드 상자 옆에 공식 형태가 보인다: 예) `ID 3 (XM430-W350) · Goal Position(주소 116) ← 949`, STS는 `ID 3 (STS3215-12V) · Goal Position(주소 42) ← 949`
- 모터 온도 표가 한계 8 °C 전에 노랑, 한계에서 빨강으로 바뀐다.
- Raspberry Pi에서 켜고 노트북 브라우저로 여는 방법은 [rpi_setup.md](rpi_setup.md) 6장.

## 9. 안전 장치
- **위치 한계**: 소프트웨어(`set_joint_targets`가 범위 밖 값을 자름) + 모터 EEPROM(보정 때 설정) 두 겹
- **온도**: 한계보다 8 °C 낮으면 경고, 한계(XL 72 °C, XM 80 °C, STS3215 70 °C ⚠️)를 넘으면 코드를 멈춤. 모터 자체 보호(Shutdown)도 켜져 있음
- **켜질 때 튀지 않게**: 토크를 켜기 전에 지금 위치를 목표로 먼저 써 둔다
- **끝날 때**: 프로그램이 끝나도 모터는 마지막 자세를 유지한다(쓰러지지 않게). 힘을 빼려면 엎드린 뒤 `dog.relax()` 또는 `tools relax`
- 실물에서 하지 않는 실험은 예제 안에서 `if dog.mode == "sim":`으로 막아 두었다 (J2 0.1초 엎드리기, J4 실험 A)

## 10. 아직 없는 것
- IMU(BNO055 등) 읽기 → `DxlReal.rpy()`·`gyro()` 자리만 있음(`has_imu = False`). 달면 J4 기울기와 J6 균형 잡기를 실물에서도 쓸 수 있다.
- 실물 위치·방향 측정(시뮬 전용 함수)
- 실제 모터 온도·전류와 시뮬 어림값 비교 (수업 실험 거리)
