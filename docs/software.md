# 소프트웨어: 시뮬레이터와 실물에서 같은 코드

## 1. 한눈에 보기
학생 코드는 `Dog` 하나만 쓴다. 어디서 돌릴지는 **첫 줄 하나**로 정한다.

```python
from dxldog import Dog
dog = Dog()                               # ① 시뮬레이터 (노트북, 3D 창)
dog = Dog("real", port="COM3")            # ② 노트북 + U2D2 케이블 → 실물
dog = Dog("real", port="/dev/ttyUSB0")    # ③ 로봇에 실은 Raspberry Pi → 실물
dog = Dog("real", port="fake")            #    하드웨어 없이 실물 코드 경로 연습 (가짜 모터)

dog.reset()
dog.move_to("stand", 1.5)
dog.set_angle("FL", "knee", 100)
```

```
                 ┌─ DxlSim  (MuJoCo: CAD 모양·질량, 모터 힘·속도 한계, 온도 어림)
학생 코드 ─ Dog ─┤
                 └─ DxlReal ─ DxlBus (dynamixel_sdk, Protocol 2.0) ─ U2D2 ─ 모터 12개
                              └ FakeBus (가짜 모터, 시험용)
```

robotdog(Lite3) 과정의 `JointDog`와 **함수 이름·각도 규칙이 같다**. 그래서 J1–J5에서 쓴 코드를 거의 그대로 옮길 수 있다. 바뀌는 것은 다리 이름(`HL/HR` → `RL/RR`)과 숫자(링크 길이, 서기 높이)뿐이다.

## 2. 폴더
```
software/
  dxldog/          패키지 (Raspberry Pi에는 이 폴더와 requirements.txt만 있으면 됨)
    config.py      다리·관절 이름, 모터 ID·종류·방향, 관절 범위, 자세, 컨트롤 테이블 주소
    dog.py         학생 API: Dog, forward/inverse_kinematics, trot_feet, deg_to_tick, motor_info
    sim.py         시뮬레이터 백엔드 DxlSim
    real.py        실물 백엔드 DxlReal, DxlBus(dynamixel_sdk), FakeBus
    tools.py       실물 준비 도구 (ID 설정, 확인, 영점 보정, 방향 확인)
    live.py        스튜디오 연결용
    assets/        CAD에서 뽑은 부품 메시(STL)·관절 위치·질량 (cad/export_sim_assets.py가 만듦)
  lessons/         예제 J1–J5 (시뮬·실물 같은 파일)
  studio/          DXL Dog Studio (웹 화면: 코드 + 3D + 조작 패널)
  requirements.txt 로봇 소프트웨어용 패키지만
```

## 3. 학생 API (`Dog`)
| 함수 | 하는 일 | 시뮬 | 실물 |
|---|---|---|---|
| `set_angle(leg, joint, deg)` / `get_angle(leg, joint)` | 관절 하나 목표 주기 / 현재 각도 읽기 | ✓ | ✓ (Goal Position 116 / Present Position 132) |
| `get_angles()` | 12개 각도 `{다리: (hip_x, hip_y, knee)}` | ✓ | ✓ (12개 동시 읽기) |
| `move_to(pose, seconds)` | 자세로 부드럽게 이동 (`"lie"`, `"stand"`, `"sit"`, `"zero"`, 튜플, 딕셔너리) | ✓ | ✓ (50 Hz로 목표를 나눠 보냄) |
| `set_feet({다리: (x, z)}, seconds)` / `get_foot(leg)` | 역기구학 / 정기구학 | ✓ | ✓ |
| `wait(s)`, `reset()` | 기다리기 / 처음 자세 | ✓ (바로 되돌림) | ✓ (2초 동안 엎드림) |
| `get_height()` | 몸 높이 | 측정값 | 관절 각도로 계산 |
| `get_tilt()` | (roll, pitch) | ✓ | IMU 달기 전에는 (0, 0) |
| `get_position()`, `get_heading()` | 바닥 위 위치·방향 | ✓ | ✗ (센서 없음) |
| `get_temperatures()` | 모터 온도 | 교육용 어림값 | Present Temperature 146 |
| `relax()` | 모터 힘 빼기 | – | ✓ |

다른 도구: `motor_info("FL", "knee")` → `(3, "XM430-W350")`, `deg_to_tick("FL", "knee", 99)` → Goal Position 값.

## 4. 각도와 모터 위치값
- 학생 각도(도): hip_x +면 발이 오른쪽, hip_y +면 허벅지가 앞, knee는 클수록 많이 굽힘. 모두 0°가 **다리 똑바로 아래**(조립 자세).
- 모터 위치값(0–4095) = `2048 + 방향 × (−각도) × 4096/360 + 보정값`
  - 방향: hip_x는 네 다리 모두 +1, hip_y·knee는 왼쪽 +1, 오른쪽 −1 (오른쪽 모터는 거울 방향으로 달려 있음)
  - 보정값: 조립 오차를 영점 보정(`tools calibrate`)으로 잰 값. `~/.dxldog/calibration.json`에 저장된다.
- 서기 자세 예: 앞왼쪽 무릎 96.6° → 2048 − 1099 = **949**, 앞오른쪽 무릎 → **3147**

## 5. 시뮬레이터 (`DxlSim`)
- 모양·질량: CAD 부품(STL)과 부품별 질량(채움 55 % 어림), 모터 무게(XL430 65 g, XM430 82 g, 공식 사양), 배터리 150 g, Pi·U2D2 100 g, 배선 100 g → **전체 1.78 kg**
- 모터: 위치 제어(P·D) + **토크-속도 한계**(정지 토크에서 무부하 속도까지 직선, 12 V 공식 사양). 빨리 돌수록 힘이 약해지므로 너무 빠른 걸음은 시뮬에서도 따라가지 못한다.
- 온도: `dT/dt = 0.5·(토크/정지토크)² − (T−25)/300` — **교육용 어림 모델**이다 ⚠️ 실제 값은 실물로 재서 비교하는 실험 거리로 쓴다.
- 확인한 결과: 서기 몸 높이 16.6 cm(설계 15.5 cm + 발 1.2 cm), 앉기 pitch −27.6°, 트롯 직진 약 14 cm/s(방향 오차 1° 이내), 돌기 약 15°/s.

### 걸음새에서 알아낸 것
- **무게중심이 엉덩이 중심보다 2.6 cm 뒤에 있다.** 네 다리 모두 무릎이 뒤로 굽어서, 서 있을 때 허벅지와 무릎 모터(82 g)가 엉덩이 뒤쪽에 온다.
  - 그래서 서기 자세의 발을 엉덩이보다 **3.5 cm 뒤**에 둔다: `stand = (0, −67.1, 96.6)` (SpotMicro 기본값 (0, −55.8, 99)은 발이 엉덩이 바로 아래).
  - 발을 엉덩이 바로 아래에 두고 걸으면 앞발이 땅을 거의 딛지 못하고(접촉 시간 14 %), 몸이 ±10° 흔들리며 방향이 틀어졌다.
- **딛는 시간 비율(duty) 0.65**: 0.5(순수 트롯)는 두 발로만 서는 시간이 길어 흔들림이 커지고, 발 위치가 1 cm만 달라져도 결과가 크게 바뀌었다. 0.65면 주기 0.4–0.6 s, 발 위치 −2…−5 cm, 보폭 −4…6 cm, 발 높이 2–4.5 cm, 모터 힘 60 %, 마찰 0.6에서도 넘어지지 않고 곧게 걸었다.
- 옮기는 발은 천천히 출발해서 천천히 도착하게(코사인) 움직인다. 스튜디오 걸음새는 첫 한 주기 동안 보폭을 0에서 키운다.
- 돌기: 오른발·왼발 보폭을 다르게(`turn`). 몸은 한 주기에 약 `step / duty` 만큼 나아간다.
- ⚠️ 실물은 모터 강성·마찰·바닥이 시뮬과 다르다. 처음에는 받침대 위에서, 다음에는 느린 주기(T 0.6)부터 시험한다.

## 6. 실물 준비 순서
도구는 모두 `software` 폴더에서 `python -m dxldog.tools <명령> --port <포트>`로 쓴다. 포트는 Windows `COM3` 같은 이름(`tools ports`로 확인), Raspberry Pi `/dev/ttyUSB0`.

| 순서 | 명령 | 하는 일 |
|---|---|---|
| 1 | `setup FL hip_x --port COM3` | **새 모터를 하나만 연결**하고 실행 → ID(1–12)·통신 속도 1 Mbps·응답 지연 0 설정, 가운데(2048)로 돌려 힘을 줌. 이 상태에서 부품을 '다리 똑바로 아래' 방향으로 끼운다. 12번 반복 |
| 2 | `scan --port COM3` | 모터 12개를 체인으로 연결한 뒤 → 응답, 종류(XL/XM), 위치, 온도, 전압 확인 |
| 3 | `calibrate --port COM3` | 받침대에 올리고 다리를 똑바로 아래로 맞춘 다음 Enter → 영점 보정값 저장 + **모터 위치 한계(EEPROM)를 관절 범위로 설정** |
| 4 | `check --port COM3` | 관절을 하나씩 천천히 +10° 움직여 방향이 맞는지 묻는다 |
| 5 | `python lessons/j1_one_motor.py --real COM3` | 예제를 실물로 |

**USB 지연 줄이기 (한 번만)**: U2D2는 USB 시리얼 칩(FTDI)의 기본 지연이 16 ms라서 12개 읽기가 느려진다. Windows는 장치 관리자 → 포트 → USB Serial Port → 속성 → 포트 설정 → 고급 → **대기 시간 타이머 1 ms**. Raspberry Pi는 [rpi_setup.md](rpi_setup.md)의 udev 규칙.

ID 표: FL 1·2·3, FR 4·5·6, RL 7·8·9, RR 10·11·12 (각 다리 hip_x·hip_y·knee 순서). hip_x는 XL430-W250, 나머지는 XM430-W350.

## 7. 노트북 연결 vs Raspberry Pi
| | ② 노트북 + U2D2 | ③ Raspberry Pi를 로봇에 |
|---|---|---|
| 연결 | USB 케이블이 로봇까지 | 케이블 없음 (전원 제외), Wi-Fi |
| 코드 실행 | 노트북 | Pi (노트북 브라우저로 스튜디오 접속, 또는 SSH) |
| 좋은 점 | 준비가 쉬움, 수업 1–3차시에 알맞음 | 자유롭게 걷기, 실제 로봇 구조 |
| 준비 | `pip install -r requirements.txt` | [rpi_setup.md](rpi_setup.md) |
| 포트 | `COM3` 등 | `/dev/ttyUSB0` |

**같은 패키지, 같은 예제 파일**이다. 처음에는 ②로 시작하고, 걷기가 안정되면 ③으로 옮긴다.

## 8. DXL Dog Studio (웹 화면)
코드 상자 + 3D 로봇 + 조작 패널(① 관절 ② 발 위치 ③ 걸음새 ④ 모터)을 한 화면에서 쓴다. robotdog의 Lite3 Studio와 같은 구조다.
```powershell
software\studio\run_studio.bat                              # 시뮬레이터
software\studio\run_studio.bat --backend real --port COM3   # 실물 (노트북 + U2D2)
software\studio\run_studio.bat --backend real --port fake   # 가짜 모터 (연습)
```
- 실물 모드의 3D 화면은 **디지털 트윈**이다: 모터에서 읽은 관절 각도를 시뮬 모델에 그대로 넣어 보여 준다.
- ④ 모터 탭과 코드 상자 옆에 공식 형태가 보인다: 예) `ID 3 (XM430-W350) · Goal Position(주소 116) ← 949`
- 모터 온도 표가 한계 8 °C 전에 노랑, 한계에서 빨강으로 바뀐다.
- Raspberry Pi에서 켜고 노트북 브라우저로 여는 방법은 [rpi_setup.md](rpi_setup.md) 6장.

## 9. 안전 장치
- **위치 한계**: 소프트웨어(`set_joint_targets`가 범위 밖 값을 자름) + 모터 EEPROM(보정 때 설정) 두 겹
- **온도**: 한계보다 8 °C 낮으면 경고, 한계(XL 72 °C, XM 80 °C)를 넘으면 코드를 멈춤. 모터 자체 보호(Shutdown)도 켜져 있음
- **켜질 때 튀지 않게**: 토크를 켜기 전에 지금 위치를 목표로 먼저 써 둔다
- **끝날 때**: 프로그램이 끝나도 모터는 마지막 자세를 유지한다(쓰러지지 않게). 힘을 빼려면 엎드린 뒤 `dog.relax()` 또는 `tools relax`
- 실물에서 하지 않는 실험은 예제 안에서 `if dog.mode == "sim":`으로 막아 두었다 (J2 0.1초 엎드리기, J4 실험 A)

## 10. 아직 없는 것
- IMU(BNO055 등) 읽기 → `DxlReal.rpy()` 자리만 있음. 달면 J4 기울기를 실물에서도 숫자로 볼 수 있다.
- 실물 위치·방향 측정(시뮬 전용 함수)
- 실제 모터 온도·전류와 시뮬 어림값 비교 (수업 실험 거리)
