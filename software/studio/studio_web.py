"""DXL Dog Studio (웹 UI) — 코드 편집기와 3D 로봇 화면을 한 창에서. 시뮬레이터와 실물 모두.

실행 (software/studio 폴더에서):
  ../../.venv/Scripts/python studio_web.py                          시뮬레이터 (앱 창으로 열림, run_studio.bat 과 같음)
  ../../.venv/Scripts/python studio_web.py --backend real --port COM3     실물 (노트북 + U2D2 / URT-1 등 어댑터)
  ../../.venv/Scripts/python studio_web.py --robot sts              로봇 종류: mixed(DYNAMIXEL) 또는 sts(STS3215)
  ../../.venv/Scripts/python studio_web.py --backend real --port fake     가짜 모터 (하드웨어 없이 연습)
  ../../.venv/Scripts/python studio_web.py --browser                기본 브라우저로 열기
  python3 studio_web.py --backend real --port /dev/ttyUSB0 --host 0.0.0.0 --no-window
                                    Raspberry Pi: 화면 없이 서버만 켜고, 같은 와이파이의 노트북 브라우저로 접속

구조:
  [앱 창 / 브라우저: web/index.html]  ⇄ WebSocket ⇄  [이 서버]  ⇄ stdin/stdout JSON ⇄  [server.py]
  화면은 three.js 로 로봇을 그리고, 물리 계산·모터 통신과 학생 코드 실행은 server.py 가 맡는다.
"""
import argparse
import asyncio
import json
import os
import socket
import struct
import sys
import threading
import webbrowser
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOFTWARE = HERE.parent
sys.path.insert(0, str(SOFTWARE))

import mujoco  # noqa: E402
from aiohttp import WSMsgType, web  # noqa: E402

from dxldog import config as C  # noqa: E402
from dxldog.sim import TEMP_LIMIT, build_model, pose_degrees  # noqa: E402

WEB = HERE / "web"
SAMPLE_DIR = HERE / "samples"
SAVE_DIR = Path.home() / "Documents" / "DXLDogStudio"
EVENT_PREFIX = "\x01EVT "
STOP_TIMEOUT = 3.0

SAMPLES = [
    ("s1", "1 — 모터 하나 움직이기", SAMPLE_DIR / "s1_one_motor.py"),
    ("s2", "2 — 자세 (엎드리기·서기·앉기)", SAMPLE_DIR / "s2_poses.py"),
    ("s3", "3 — 각도 ↔ 발 위치 (기구학)", SAMPLE_DIR / "s3_kinematics.py"),
    ("s4", "4 — 걸음새 직접 만들기 (트롯)", SAMPLE_DIR / "s4_trot.py"),
    ("s5", "5 — 모터 번호·위치값·온도", SAMPLE_DIR / "s5_motors.py"),
]


# ---------------- 로봇 모델 (3D 화면용) ----------------
class Scene:
    """MuJoCo 모델에서 보이는 부품(group 0, 바닥 제외)의 메시·상자와 배치·색을 꺼낸다. 시뮬레이터와 같은 모델."""

    def __init__(self):
        m = build_model()
        self.meshes = {}
        geoms = []
        for g in range(m.ngeom):
            if m.geom_group[g] != 0 or m.geom_bodyid[g] == 0:
                continue                                    # 충돌용(group 3)·바닥은 그리지 않음
            item = {"body": int(m.geom_bodyid[g]), "bodyName": m.body(m.geom_bodyid[g]).name,
                    "pos": m.geom_pos[g].round(6).tolist(), "quat": m.geom_quat[g].round(6).tolist(),
                    "rgba": m.geom_rgba[g].round(3).tolist()}
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                mid = int(m.geom_dataid[g])
                item.update(type="mesh", mesh=mid)
                if mid not in self.meshes:
                    va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
                    fa, fn = m.mesh_faceadr[mid], m.mesh_facenum[mid]
                    verts = m.mesh_vert[va:va + vn].astype("<f4")
                    faces = m.mesh_face[fa:fa + fn].astype("<u4")
                    self.meshes[mid] = struct.pack("<II", vn, fn) + verts.tobytes() + faces.tobytes()
            elif m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE:
                item.update(type="sphere", radius=float(m.geom_size[g][0]))
            elif m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                item.update(type="box", size=[float(v) for v in m.geom_size[g]])
            else:
                continue
            geoms.append(item)
        self.info = {"nbody": int(m.nbody), "bodies": [m.body(i).name for i in range(m.nbody)], "geoms": geoms}


def meta(backend, port):
    keys = [(leg, j) for leg in C.LEGS for j in C.JOINTS]
    return {
        "samples": [{"id": sid, "name": name} for sid, name, _ in SAMPLES],
        "backend": backend, "port": port,
        "robot": C.ROBOT, "robotName": C.ROBOT_INFO["name"], "brand": C.BRAND, "adapter": C.ADAPTER,
        "legs": C.LEGS, "legNames": C.LEG_NAMES, "joints": C.JOINTS,
        "jointInfo": {j: {"name": v[0], "meaning": v[1], "range": C.JOINT_RANGE_DEG[j]} for j, v in C.JOINT_INFO.items()},
        "poses": {k: pose_degrees(k) for k in C.POSES},
        "thigh": C.THIGH, "shank": C.SHANK,
        "motorIds": [C.MOTOR_IDS[k] for k in keys],
        "motorModels": [C.MOTOR_MODEL[j] for _, j in keys],
        "direction": [C.DIRECTION[k] for k in keys],
        "tempLimit": [TEMP_LIMIT[C.MOTOR_MODEL[j]] for _, j in keys],
        "center": C.CENTER, "radToTick": C.RAD_TO_TICK,
        "addr": {k: C.ADDR[k][0] for k in ("goal_position", "present_position", "present_temperature", "torque_enable")},
        "addrSize": {k: C.ADDR[k][1] for k in ("goal_position", "present_position", "present_temperature", "torque_enable")},
    }


# ---------------- 로봇 서버 프로세스 ----------------
class SimBridge:
    """server.py 를 띄우고, 웹 화면(WebSocket)들과 메시지를 주고받는다."""

    def __init__(self, backend, port):
        self.backend, self.port = backend, port
        self.proc = None
        self.clients = set()
        self.running = False
        self.run_id = 0
        self.last_state = None
        self.fatal = None                 # 실물 연결 실패 메시지 (↺ 를 누르면 다시 시도)

    async def start(self):
        env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONPATH=str(SOFTWARE))
        flags = 0x08000000 if sys.platform == "win32" else 0          # CREATE_NO_WINDOW
        args = [sys.executable, "-u", str(HERE / "server.py"), "--backend", self.backend, "--robot", C.ROBOT]
        if self.port:
            args += ["--port", self.port]
        self.fatal = None
        self.proc = await asyncio.create_subprocess_exec(
            *args, stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
            cwd=str(SAVE_DIR), env=env, creationflags=flags)
        asyncio.create_task(self._read(self.proc))

    async def _read(self, proc):
        while True:
            raw = await proc.stdout.readline()
            if not raw:
                break
            line = raw.decode("utf-8", errors="replace").replace("\r\n", "\n")
            if line.startswith(EVENT_PREFIX):
                ev = json.loads(line[len(EVENT_PREFIX):])
                name = ev.pop("event")
                if name in ("frame", "state"):
                    if name == "state":
                        self.last_state = ev
                    await self.broadcast({"type": name, **ev})
                    continue
                if name == "run_start":
                    self.running = True
                elif name == "run_end":
                    self.running = False
                elif name == "fatal":
                    self.fatal = ev["text"]
                await self.broadcast({"type": "event", "event": name, **ev})
            else:
                await self.broadcast({"type": "out", "text": line})
        await proc.wait()
        if proc is self.proc:                                         # 예상치 못한 종료 → 다시 켬
            was_running, self.running = self.running, False
            await self.broadcast({"type": "event", "event": "sim_restart", "wasRunning": was_running})
            await asyncio.sleep(0.3)
            await self.start()

    async def send(self, msg):
        if self.proc and self.proc.returncode is None:
            self.proc.stdin.write((json.dumps(msg, ensure_ascii=False) + "\n").encode("utf-8"))
            await self.proc.stdin.drain()

    async def stop(self):
        await self.send({"cmd": "stop"})
        run_id = self.run_id
        await asyncio.sleep(STOP_TIMEOUT)
        if self.running and run_id == self.run_id:                  # 응답 없음 → 서버 재시작
            await self.broadcast({"type": "out", "text": "응답이 없어 로봇 서버를 다시 시작해요.\n", "kind": "err"})
            self.proc.kill()

    async def broadcast(self, msg):
        data = json.dumps(msg, ensure_ascii=False)
        for ws in list(self.clients):
            try:
                await ws.send_str(data)
            except ConnectionError:
                self.clients.discard(ws)

    async def handle(self, msg):
        kind = msg.get("type")
        if kind == "run":
            self.run_id += 1
            await self.send({"cmd": "run", "code": msg["code"]})
        elif kind == "stop":
            asyncio.create_task(self.stop())
        elif kind == "reset":
            if self.fatal:                                           # 연결 실패 상태 → 서버를 다시 켜서 재시도
                await self.broadcast({"type": "out", "text": "모터 연결을 다시 시도해요…\n", "kind": "info"})
                self.proc.kill()
            else:
                await self.send({"cmd": "reset"})
        elif kind == "panel":
            await self.send({"cmd": "panel", **{k: v for k, v in msg.items() if k != "type"}})

    async def close(self):
        if self.proc and self.proc.returncode is None:
            proc, self.proc = self.proc, None
            try:                                                     # 실물: 연결을 닫을 시간을 준다
                await self.send_to(proc, {"cmd": "quit"})
                await asyncio.wait_for(proc.wait(), 2)
            except Exception:
                proc.kill()

    @staticmethod
    async def send_to(proc, msg):
        proc.stdin.write((json.dumps(msg) + "\n").encode("utf-8"))
        await proc.stdin.drain()


# ---------------- HTTP ----------------
def make_app(backend, port):
    scene = Scene()
    bridge = SimBridge(backend, port)
    app = web.Application()

    async def index(_):
        return web.FileResponse(WEB / "index.html", headers={"Cache-Control": "no-store"})

    async def api_meta(_):
        return web.json_response(meta(backend, port))

    async def api_scene(_):
        return web.json_response(scene.info)

    async def api_mesh(req):
        mid = int(req.match_info["mid"])
        if mid not in scene.meshes:
            raise web.HTTPNotFound()
        return web.Response(body=scene.meshes[mid], content_type="application/octet-stream")

    async def api_sample(req):
        for sid, _, path in SAMPLES:
            if sid == req.match_info["sid"]:
                return web.json_response({"code": path.read_text(encoding="utf-8")})
        raise web.HTTPNotFound()

    async def api_files(_):
        files = sorted(SAVE_DIR.glob("*.py"), key=lambda p: p.stat().st_mtime, reverse=True)
        return web.json_response([p.name for p in files])

    def safe_path(name):
        name = Path(name).name
        if not name.endswith(".py"):
            name += ".py"
        return SAVE_DIR / name

    async def api_file_get(req):
        p = safe_path(req.match_info["name"])
        if not p.exists():
            raise web.HTTPNotFound()
        return web.json_response({"name": p.name, "code": p.read_text(encoding="utf-8")})

    async def api_file_save(req):
        body = await req.json()
        p = safe_path(body["name"])
        p.write_text(body["code"], encoding="utf-8")
        return web.json_response({"name": p.name, "path": str(p)})

    async def ws_handler(req):
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(req)
        bridge.clients.add(ws)
        await ws.send_str(json.dumps({"type": "hello", "running": bridge.running, "fatal": bridge.fatal}, ensure_ascii=False))
        if bridge.last_state:
            await ws.send_str(json.dumps({"type": "state", **bridge.last_state}))
        async for m in ws:
            if m.type == WSMsgType.TEXT:
                await bridge.handle(json.loads(m.data))
        bridge.clients.discard(ws)
        return ws

    async def on_startup(_):
        SAVE_DIR.mkdir(parents=True, exist_ok=True)
        await bridge.start()

    async def on_cleanup(_):
        await bridge.close()

    app.router.add_get("/", index)
    app.router.add_get("/api/meta", api_meta)
    app.router.add_get("/api/scene", api_scene)
    app.router.add_get("/api/mesh/{mid}", api_mesh)
    app.router.add_get("/api/sample/{sid}", api_sample)
    app.router.add_get("/api/files", api_files)
    app.router.add_get("/api/file/{name}", api_file_get)
    app.router.add_post("/api/file", api_file_save)
    app.router.add_get("/ws", ws_handler)
    app.router.add_static("/web/", WEB)
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    return app


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def lan_ip():
    """같은 네트워크의 다른 컴퓨터가 접속할 주소 (패킷은 보내지 않음)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))
            return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def serve(args, ready=None):
    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    runner = web.AppRunner(make_app(args.backend, args.port))
    loop.run_until_complete(runner.setup())
    loop.run_until_complete(web.TCPSite(runner, args.host, args.http_port).start())
    if ready:
        ready.set()
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        loop.run_until_complete(runner.cleanup())


def main():
    ap = argparse.ArgumentParser(description="DXL Dog Studio — 코드 편집기 + 3D 로봇 화면")
    ap.add_argument("--backend", choices=["sim", "real"], default="sim", help="sim(시뮬레이터, 기본) 또는 real(실물)")
    ap.add_argument("--port", default=None, help="실물 모터 포트: COM3, /dev/ttyUSB0, /dev/ttyACM0, fake(가짜 모터)")
    ap.add_argument("--robot", choices=list(C.ROBOTS), default=None, help=f"로봇 종류: mixed 또는 sts (기본: {C.ROBOT})")
    ap.add_argument("--host", default="127.0.0.1", help="0.0.0.0 이면 같은 네트워크의 다른 컴퓨터에서 접속 가능")
    ap.add_argument("--http-port", type=int, default=None, help="웹 서버 포트 (기본: 빈 포트 자동, --host 0.0.0.0 이면 8765)")
    ap.add_argument("--no-window", action="store_true", help="창을 열지 않고 주소만 출력 (Raspberry Pi·시험용)")
    ap.add_argument("--browser", action="store_true", help="앱 창 대신 기본 브라우저로 열기")
    args = ap.parse_args()
    if args.backend == "real" and not args.port:
        ap.error("실물은 --port 가 필요해요. 예: --port COM3, --port /dev/ttyUSB0, 연습용 --port fake")
    if args.robot:
        C.use(args.robot)                                     # 3D 모델·모터 정보·로봇 서버 모두 이 종류로
    if args.http_port is None:
        args.http_port = 8765 if args.host != "127.0.0.1" else free_port()
    url = f"http://127.0.0.1:{args.http_port}/"
    headless = sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if args.no_window or headless:
        print(f"DXL Dog Studio: {url}", flush=True)
        if args.host != "127.0.0.1":
            print(f"다른 컴퓨터에서 접속: http://{lan_ip()}:{args.http_port}/", flush=True)
        serve(args)
        return
    ready = threading.Event()
    threading.Thread(target=serve, args=(args, ready), daemon=True).start()
    ready.wait(30)
    if args.host != "127.0.0.1":
        print(f"다른 컴퓨터에서 접속: http://{lan_ip()}:{args.http_port}/", flush=True)
    if not args.browser:
        try:
            import webview
            webview.create_window("DXL Dog Studio", url, width=1440, height=900, min_size=(1100, 700),
                                  background_color="#0e1116", maximized=True)
            webview.start()
            return
        except Exception as e:  # pywebview 가 없거나 WebView2 가 없으면 브라우저로
            print("앱 창을 열 수 없어 브라우저로 엽니다:", e)
    webbrowser.open(url)
    threading.Event().wait()


if __name__ == "__main__":
    main()
