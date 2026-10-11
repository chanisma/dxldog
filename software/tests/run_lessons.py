"""예제 J1–J5 를 시뮬레이터(창 없이)와 가짜 모터로 모두 실행해 본다.  실행: python tests/run_lessons.py"""
import subprocess
import sys
from pathlib import Path

LESSONS = sorted((Path(__file__).resolve().parents[1] / "lessons").glob("j*.py"))
failed = []
for f in LESSONS:
    for opts in (["--no-view"], ["--real", "fake"]):
        r = subprocess.run([sys.executable, str(f), *opts], capture_output=True, text=True, encoding="utf-8",
                           env={**__import__("os").environ, "PYTHONUTF8": "1"}, timeout=300)
        ok = r.returncode == 0
        print(f"{'✓' if ok else '✗'} {f.name:20s} {' '.join(opts)}")
        if not ok:
            failed.append(f.name)
            print(r.stdout[-800:], r.stderr[-1500:])
sys.exit(1 if failed else 0)
