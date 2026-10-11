# DXL Dog — SpotMicro × ROBOTIS DYNAMIXEL 교육용 로봇개

오픈소스 4족 로봇 **SpotMicro**(CC BY 4.0)의 크기와 다리 구조를 유지하고, 모터를 **DYNAMIXEL X430**(XL430 / XC430 / XM430)으로 바꿔 다시 설계한 교육용 로봇개다. LeRobot용 **Feetech STS3215**로 만드는 **STS 버전**도 있다 ([docs/sts.md](docs/sts.md)). `robotdog`(Lite3 시뮬레이터 수업)의 심화 실습을 실물로 이어 가기 위해 만들었다.

![서기 자세 미리보기](cad/out/preview_mixed_stand.png)

## 문서
- [기획서](docs/plan.md): 목표, 단계별 일정, 정해야 할 것, 위험
- [부품 선정과 BOM](docs/parts.md): 토크 시뮬레이션 결과, 모터 등급, 나사·전원
- [설계 설명](docs/design.md): SpotMicro 대비 변경점, 관절 구조, 충돌 검사, 맞춤 시험
- [소프트웨어](docs/software.md): 시뮬레이터·실물 같은 코드(`Dog`), 모터 ID 설정·영점 보정, 안전 장치
- [Raspberry Pi 싣기](docs/rpi_setup.md): 전원, 설치, 노트북 브라우저로 조종
- [STS 버전](docs/sts.md): LeRobot 모터(STS3215)로 만들기 — 힘 비교, 장착 방식, 부품, 맞춤 시험

## 폴더
```
sim/   토크 시뮬레이션 (MuJoCo): dog_model.py, torque_study.py, gait_sweep.py, scale_study.py, verify_cad_mass.py
cad/   파라메트릭 CAD (build123d): x430.py·sts3215.py(모터 치수), dxldog_cad.py(부품), build_all.py, check.py, render.py
       out/xl, out/xm, out/mixed, out/sts  → 인쇄용 STL + 편집용 STEP (모터 구성별)
software/  로봇 소프트웨어 (Raspberry Pi에는 이 폴더만)
       dxldog/   Dog API, 시뮬레이터(DxlSim), 실물(DxlReal·dynamixel_sdk), 준비 도구(tools)
       lessons/  예제 J1–J6 (--real <포트> 로 실물, J6 균형 잡기는 IMU 필요)
       studio/   DXL Dog Studio (웹: 코드 + 3D + 조작 패널)
docs/  기획·부품·설계·소프트웨어 문서
```

## 실행
```powershell
python -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
cd cad
..\.venv\Scripts\python build_all.py mixed     # 부품 STL/STEP 만들기 (xl | xm | mixed | sts)
..\.venv\Scripts\python check.py mixed         # 관절 범위 전체 충돌 검사
..\.venv\Scripts\python render.py mixed stand  # 조립 미리보기 (stand | rest | zero)
cd ..\sim
..\.venv\Scripts\python torque_study.py        # 모터별 필요 토크·성공 여부
cd ..\software
..\.venv\Scripts\python lessons\j2_poses.py              # 예제: 시뮬레이터
..\.venv\Scripts\python lessons\j2_poses.py --real COM3  # 같은 예제: 실물 (노트북 + U2D2)
..\.venv\Scripts\python -m dxldog.tools scan --port COM3  # 모터 12개 확인 (준비 순서는 docs/software.md)
studio\run_studio.bat                                      # DXL Dog Studio (시뮬레이터)
```

## 라이선스
3D 설계는 SpotMicro(KDY0523, Thingiverse 3445283, CC BY 4.0)를 개작한 것으로 **CC BY 4.0**을 따른다. 치수 기준은 mike4192/spotMicro(MIT)의 설정값, 모터 치수는 ROBOTIS 공식 도면이다.
