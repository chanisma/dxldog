import { Viewer } from "/web/viewer.js";
import { Panel } from "/web/panel.js";

const $ = (id) => document.getElementById(id);
const store = {
  get(k, d) { try { return JSON.parse(localStorage.getItem(k)) ?? d; } catch { return d; } },
  set(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch { /* 저장 불가 환경 */ } },
};

const meta = await (await fetch("/api/meta")).json();
const isReal = meta.backend === "real";
const TEMP_WARN = 8;                 // 한계보다 이만큼 낮으면 경고 색 (dxldog.dog 와 같음)
let ws = null;
let running = false;
let fatal = null;
let errorLine = null;
let fileName = null;

// ---------------- 편집기 ----------------
const editor = CodeMirror($("editor"), {
  mode: "python", lineNumbers: true, indentUnit: 4, tabSize: 4, indentWithTabs: false,
  matchBrackets: true, styleActiveLine: true, lineWrapping: false,
  extraKeys: {
    Tab: (cm) => cm.somethingSelected() ? cm.indentSelection("add") : cm.replaceSelection("    ", "end"),
    "Shift-Tab": (cm) => cm.indentSelection("subtract"),
    "Ctrl-Enter": () => run(), F5: () => run(), "Ctrl-S": () => saveDialog(), Esc: () => stop(),
  },
});
editor.on("change", () => {
  store.set("dxldog.code", { sample: $("sample").value, file: fileName, code: editor.getValue() });
  clearError();
});

function clearError() {
  if (errorLine !== null) {
    editor.removeLineClass(errorLine, "background", "error-line");
    editor.removeLineClass(errorLine, "gutter", "error-gutter");
    errorLine = null;
  }
}
function markError(n) {
  clearError();
  errorLine = n - 1;
  editor.addLineClass(errorLine, "background", "error-line");
  editor.addLineClass(errorLine, "gutter", "error-gutter");
  editor.scrollIntoView({ line: errorLine, ch: 0 }, 80);
}

// ---------------- 샘플 / 파일 ----------------
const sel = $("sample");
for (const s of meta.samples) sel.add(new Option(s.name, s.id));
async function loadSample(id) {
  const { code } = await (await fetch(`/api/sample/${id}`)).json();
  fileName = null;
  setTitle();
  editor.setValue(code);
  editor.clearHistory();
}
function setTitle() {
  $("file-title").textContent = fileName ? fileName : (meta.samples.find((s) => s.id === sel.value)?.name ?? "내 코드");
}
sel.onchange = () => loadSample(sel.value);
$("btn-revert").onclick = () => loadSample(sel.value);

const saved = store.get("dxldog.code", null);
if (saved && saved.code) {
  sel.value = saved.sample || meta.samples[0].id;
  fileName = saved.file || null;
  editor.setValue(saved.code);
  setTitle();
} else {
  sel.value = meta.samples[0].id;
  await loadSample(sel.value);
}

const dlg = $("dlg");
function dialog(title, html, onOk) {
  $("dlg-title").textContent = title;
  $("dlg-content").innerHTML = html;
  dlg.returnValue = "";
  dlg.onclose = () => { if (dlg.returnValue === "ok") onOk(); };
  dlg.showModal();
}
function saveDialog() {
  dialog("코드 저장", `<input type="text" id="save-name" placeholder="파일 이름" value="${fileName ?? "my_dog.py"}">
    <div class="muted small">문서/DXLDogStudio 폴더에 저장돼요.</div>`, async () => {
    const name = $("save-name").value.trim() || "my_dog.py";
    const r = await fetch("/api/file", { method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name, code: editor.getValue() }) });
    const res = await r.json();
    fileName = res.name;
    setTitle();
    log(`💾 저장했어요: ${res.path}\n`, "info");
  });
  setTimeout(() => { const i = $("save-name"); i.focus(); i.select(); }, 30);
}
$("btn-save").onclick = saveDialog;
$("btn-open").onclick = async () => {
  const files = await (await fetch("/api/files")).json();
  let pick = null;
  dialog("저장한 코드 열기", files.length ? `<div class="file-list">${files.map((f) => `<button type="button" data-f="${f}">${f}</button>`).join("")}</div>`
    : `<div class="muted">아직 저장한 코드가 없어요.</div>`, async () => {
    if (!pick) return;
    const { name, code } = await (await fetch(`/api/file/${encodeURIComponent(pick)}`)).json();
    fileName = name;
    editor.setValue(code);
    setTitle();
  });
  dlg.querySelectorAll(".file-list button").forEach((b) => b.onclick = () => {
    pick = b.dataset.f;
    dlg.querySelectorAll(".file-list button").forEach((x) => x.classList.toggle("sel", x === b));
  });
};

// ---------------- 출력 ----------------
const con = $("console");
function log(text, cls) {
  const m = text.match(/File "내 코드", line (\d+)/);
  if (m) {
    markError(+m[1]);
    text = text.replace(/File "내 코드", line (\d+)/, "내 코드 $1번째 줄");
  }
  const isErr = cls === undefined && (/^(Traceback|\s{2}File|\s{4})/.test(text) || /^\w+(Error|Exception)\b/.test(text));
  const span = document.createElement("span");
  if (cls || isErr) span.className = cls || "err";
  span.textContent = text;
  const atBottom = con.scrollHeight - con.scrollTop - con.clientHeight < 30;
  con.appendChild(span);
  while (con.childNodes.length > 2000) con.removeChild(con.firstChild);
  if (atBottom) con.scrollTop = con.scrollHeight;
}
$("btn-clear").onclick = () => { con.innerHTML = ""; };

// ---------------- 실행 상태 ----------------
const ON_TEXT = isReal ? (meta.port === "fake" ? "가짜 모터 연결됨" : "모터 연결됨") : "시뮬레이터 켜짐";
function setRunning(on) {
  running = on;
  $("btn-run").disabled = on || !!fatal;
  $("btn-stop").disabled = !on;
  $("run-status").textContent = on ? "실행 중… (Esc 로 정지)" : "준비";
  if (!fatal) pill(on ? "busy" : "on", on ? "코드 실행 중" : ON_TEXT);
  panel.setLocked(on || !!fatal);
}
function setFatal(text) {
  fatal = text;
  if (text) {
    pill("bad", "모터 연결 실패");
    log(`✖ 모터에 연결하지 못했어요: ${text}\n  케이블·전원·포트 이름을 확인하고 ↺ 를 눌러 다시 시도하세요.\n`, "err");
  }
  setRunning(running);
}
function pill(cls, text) {
  const p = $("sim-pill");
  p.className = "pill " + cls;
  p.querySelector("b").textContent = text;
}
function send(msg) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(msg)); }
function run() {
  if (running || fatal) return;
  clearError();
  send({ type: "run", code: editor.getValue() });
}
function stop() { if (running) send({ type: "stop" }); }
$("btn-run").onclick = run;
$("btn-stop").onclick = stop;
$("btn-reset").onclick = () => {
  if (running) { log("실행 중에는 초기화할 수 없어요. 먼저 ■ 정지하세요.\n", "info"); return; }
  send({ type: "reset" });
};
if (isReal) $("btn-reset").title = "로봇이 천천히 엎드려요 (연결 실패 때는 다시 연결)";
window.addEventListener("keydown", (e) => {
  if (e.key === "F5") { e.preventDefault(); run(); }
  if (e.key === "Escape" && running) { e.preventDefault(); stop(); }
  if (e.key === "s" && e.ctrlKey) { e.preventDefault(); saveDialog(); }
});

// ---------------- 3D 화면 / 패널 ----------------
const viewer = new Viewer($("viewport"));
viewer.load().then(() => $("viewport-loading").classList.add("hidden"));
const followBtn = $("btn-follow");
followBtn.onclick = () => {
  viewer.follow = !viewer.follow;
  followBtn.setAttribute("aria-pressed", viewer.follow);
};
$("btn-view").onclick = () => viewer.resetView();
window.studio = { viewer };          // 개발자 도구에서 살펴보기용

const panel = new Panel(meta, send, (text) => {
  const doc = editor.getDoc();
  const cur = doc.getCursor();
  const line = doc.getLine(cur.line);
  doc.replaceRange((line.trim() ? "\n" : "") + text, line.trim() ? { line: cur.line, ch: line.length } : cur);
  editor.focus();
});

// 모드 표시: 시뮬레이터 / 실물(포트) / 가짜 모터
const bb = $("backend-badge");
bb.textContent = !isReal ? "시뮬레이터" : meta.port === "fake" ? "가짜 모터 (연습)" : `실물 · ${meta.port}`;
bb.classList.add(!isReal ? "sim" : meta.port === "fake" ? "fake" : "real");
$("twin-note").style.display = isReal ? "" : "none";

const temps = $("hud-temps");
temps.innerHTML = `<span></span>${["hip_x", "hip_y", "knee"].map((j) => `<span class="th">${j}</span>`).join("")}` +
  meta.legs.map((l, i) => `<span class="th">${l}</span>${[0, 1, 2].map((k) => `<b class="t" id="t-${i * 3 + k}">–</b>`).join("")}`).join("");
function tempClass(t, lim) { return t >= lim ? "hot" : t >= lim - TEMP_WARN ? "warn" : ""; }

function hud(st) {
  $("hud-height").textContent = `${(st.height * 100).toFixed(1)} cm${isReal ? " (계산)" : ""}`;
  $("hud-tilt").textContent = isReal ? "– (기울기 센서 없음)"
    : `좌우 ${st.rpy[0] >= 0 ? "+" : ""}${st.rpy[0].toFixed(0)}° · 앞뒤 ${st.rpy[1] >= 0 ? "+" : ""}${st.rpy[1].toFixed(0)}°`;
  const badge = $("mode-badge");
  badge.textContent = st.fallen ? "⚠ 넘어졌어요 · ↺ 처음 자세" : st.gait ? "걷는 중" : "관절 조작";
  badge.classList.toggle("warn", !!st.fallen);
  st.temp.forEach((t, i) => {
    const e = $(`t-${i}`);
    e.textContent = t.toFixed(0);
    e.className = "t " + tempClass(t, st.tlim[i]);
  });
}

let toastTimer = null;
function toast(text) {
  const t = $("toast");
  t.textContent = text;
  t.classList.add("show");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.remove("show"), 4500);
}

// ---------------- 서버 연결 ----------------
function connect() {
  pill("", "연결 중");
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = (e) => {
    const m = JSON.parse(e.data);
    switch (m.type) {
      case "frame": viewer.applyFrame(m.p, m.q); break;
      case "state": hud(m); panel.onState(m); break;
      case "out": log(m.text, m.kind); break;
      case "hello": fatal = null; if (m.fatal) setFatal(m.fatal); else setRunning(m.running); break;
      case "event": onEvent(m); break;
    }
  };
  ws.onclose = () => { pill("", "다시 연결 중"); setTimeout(connect, 1000); };
}
function onEvent(m) {
  switch (m.event) {
    case "ready": fatal = null; setRunning(false); pill("on", ON_TEXT); break;
    case "fatal": setFatal(m.text); break;
    case "run_start": setRunning(true); log("▶ 실행 시작\n", "info"); break;
    case "run_end":
      setRunning(false);
      panel.resync();
      if (m.status === "ok") log("✔ 끝! 로봇은 지금 자세 그대로 기다려요. 코드를 고쳐 이어서 실행해 보세요.\n", "ok");
      else if (m.status === "stopped") log("■ 정지했어요\n", "info");
      else log("✖ 오류로 멈췄어요. 편집기의 빨간 줄을 확인하세요.\n", "err");
      break;
    case "reset_done": panel.resync(); log(isReal ? "↺ 로봇이 처음 자세(엎드림)로 천천히 돌아가요\n" : "↺ 로봇을 처음 자세(엎드림)로 되돌렸어요\n", "info"); break;
    case "busy": log("로봇이 아직 바빠요. 잠시 후 다시 시도하세요.\n", "info"); break;
    case "notice": log(`ℹ ${m.text}\n`, "info"); toast(m.text); break;
    case "sim_restart":
      if (m.wasRunning) { setRunning(false); log("■ 로봇 서버를 다시 시작했어요\n", "info"); }
      pill("", isReal ? "모터 다시 연결 중" : "시뮬레이터 다시 켜는 중");
      break;
  }
}
connect();

// ---------------- 크기 조절 ----------------
const layout = $("layout");
const lay = store.get("dxldog.layout", {});
for (const [k, v] of Object.entries(lay)) document.documentElement.style.setProperty(k, v);
function drag(handle, onMove) {
  handle.addEventListener("pointerdown", (e) => {
    handle.setPointerCapture(e.pointerId);
    const mv = (ev) => onMove(ev);
    const up = () => { handle.removeEventListener("pointermove", mv); handle.removeEventListener("pointerup", up); store.set("dxldog.layout", lay); };
    handle.addEventListener("pointermove", mv);
    handle.addEventListener("pointerup", up);
  });
}
function setVar(k, v) { lay[k] = v; document.documentElement.style.setProperty(k, v); }
drag($("vsplit"), (e) => {
  const r = layout.getBoundingClientRect();
  setVar("--left", `${Math.min(70, Math.max(25, ((e.clientX - r.left) / r.width) * 100)).toFixed(1)}%`);
});
document.querySelectorAll(".hsplit").forEach((h) => {
  const target = $(h.dataset.target);
  const key = target.id === "console-pane" ? "--console-h" : "--panel-h";
  drag(h, (e) => {
    const col = h.parentElement.getBoundingClientRect();
    const px = col.bottom - e.clientY;
    setVar(key, `${Math.min(col.height - 160, Math.max(110, px)).toFixed(0)}px`);
  });
});
setTitle();
