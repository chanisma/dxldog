"""예제 J1–J6 를 시뮬레이터(창 없이)와 가짜 모터로, 로봇 종류(mixed·sts)마다 모두 실행해 본다.
실행: python tests/run_lessons.py"""
import os
import subprocess
import sys
from pathlib import Path

SOFTWARE = Path(__file__).resolve().parents[1]
LESSONS = sorted((SOFTWARE / "lessons").glob("j*.py"))
RUNS = []
for robot in ("mixed", "sts"):
    if (SOFTWARE / "dxldog" / "assets" / robot / "frames.json").exists():
        RUNS.append(["--robot", robot, "--no-view"])
    else:
        print(f"- 건너뜀: {robot} 시뮬레이터 (assets/{robot}/ 없음)")
    RUNS.append(["--robot", robot, "--real", "fake"])

failed = []
for f in LESSONS:
    for opts in RUNS:
        r = subprocess.run([sys.executable, str(f), *opts], capture_output=True, text=True, encoding="utf-8",
                           env={**os.environ, "PYTHONUTF8": "1"}, timeout=300)
        ok = r.returncode == 0
        print(f"{'✓' if ok else '✗'} {f.name:20s} {' '.join(opts)}")
        if not ok:
            failed.append(f"{f.name} {' '.join(opts)}")
            print(r.stdout[-800:], r.stderr[-1500:])
sys.exit(1 if failed else 0)
