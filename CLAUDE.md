# CLAUDE.md

SpotMicro를 DYNAMIXEL X430으로 다시 설계하는 교육용 로봇개 프로젝트. 문서는 한국어로 작성한다. 자매 프로젝트: `../robotdog`(Lite3 시뮬 수업), `../robotarm`(SO-101).

- 모터 치수는 `cad/x430.py` 한 곳에만 둔다(ROBOTIS 공식 도면 값). 도면에 없는 값에는 ⚠️와 맞춤 시험 안내를 붙인다.
- CAD를 바꾸면 반드시 `cad/check.py`(충돌)와 `cad/render.py`(미리보기)를 다시 돌린다. 세 구성(xl, xm, mixed) 모두 확인한다.
- build123d의 `extrude`는 방향을 꼭 명시한다(`dir=plane.z_dir`). 점 순서에 따라 뒤집힌다.
- MjSpec은 관절 범위 기본 단위가 도(°)다(`spec.compiler.degree = False`). 액추에이터는 `ctrlrange`를 지정하지 않으면 목표가 0으로 잘린다.
- 모터 선정은 `sim/`의 시뮬레이션 결과(정지 토크 대비 RMS 부하율)로 판단한다. 질량을 바꾸면 `verify_cad_mass.py`를 다시 돌린다.
- Windows 콘솔은 cp949다. heredoc으로 Python에 한글 스크립트를 넘길 때는 `PYTHONUTF8=1`을 쓴다.
- 소프트웨어(`software/`)는 시뮬·실물 같은 코드가 원칙이다. 학생 API `Dog`는 robotdog `JointDog`와 이름·각도 규칙을 맞춘다. 백엔드(DxlSim, DxlReal)는 같은 속성(joint_pos, q_target, temperature, set_joint_targets, step)을 가진다.
- 각도 규칙: 모델 rad = −radians(학생 각도), 모터 틱 = 2048 + 방향 × 모델 rad × 4096/2π + 보정값 (`config.py`). 모터 ID·방향·범위는 `config.py` 한 곳에만.
- 하드웨어 없이 실물 경로를 시험할 때는 `port="fake"`(FakeBus). 예제는 `--no-view`(시뮬)와 `--real fake`로 모두 돌려 본다.
- CAD 부품을 바꾸면 `cad/export_sim_assets.py`로 `software/dxldog/assets`를 다시 만든다.
