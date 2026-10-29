"""Pagina de lucru a echipei (HTML + CSS + JS inline, fără resurse externe: Pi-ul poate fi offline).

Panouri: „Live” (fluxul camerei), „Ce vede brain-ul” (analiza adnotată + caroiajul celulelor), „Stare”
(șablonul, serviciul de mișcare, gheara), „Ținta (S1)” (poză sau editare manuală), „Plan” și, doar cu
`preview.allow_execute: true`, „Execuție”. Datele vin din `/status.json` (la ~1 s) și din rutele JSON ale mesei
de lucru; nicio reîncărcare a paginii.
"""

from __future__ import annotations

import html
import json

_HEAD = """<!doctype html>
<html lang="ro"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Brain · masa de lucru</title>
<style>
:root {
  --bg: #f3f4f7; --card: #ffffff; --text: #1c2230; --muted: #5d6878; --line: #dde1e8; --chip: #eef1f5;
  --accent: #2d6cdf; --ok: #1e9150; --warn: #c27c00; --bad: #d13f3f;
  --red: #d64545; --yellow: #e2b714; --green: #2f9e44; --blue: #2f6fde; --shadow: 0 1px 2px rgba(20, 30, 50, .06);
}
@media (prefers-color-scheme: dark) {
  :root {
    --bg: #12151a; --card: #1b1f26; --text: #e6e9ef; --muted: #98a2b3; --line: #2b313b; --chip: #252a33;
    --accent: #5b8ff0; --ok: #3fbf74; --warn: #e3a634; --bad: #ef6464; --shadow: none;
  }
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--text); font: 15px/1.45 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; }
header { position: sticky; top: 0; z-index: 5; background: var(--card); border-bottom: 1px solid var(--line);
  padding: 10px 16px; display: flex; flex-wrap: wrap; align-items: center; gap: 8px 14px; }
header h1 { font-size: 17px; margin: 0; font-weight: 650; }
.chips { display: flex; flex-wrap: wrap; gap: 6px; }
.chip { background: var(--chip); border-radius: 999px; padding: 2px 10px; font-size: 13px; color: var(--muted); white-space: nowrap; }
.chip b { color: var(--text); font-weight: 600; }
.dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; background: var(--muted); margin-right: 5px; vertical-align: 1px; }
.dot.ok { background: var(--ok); } .dot.warn { background: var(--warn); } .dot.bad { background: var(--bad); }
main { max-width: 1320px; margin: 0 auto; padding: 16px; display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 16px; }
@media (max-width: 700px) { main { grid-template-columns: minmax(0, 1fr); padding: 10px; gap: 10px; } header { padding: 8px 10px; } }
.card { background: var(--card); border: 1px solid var(--line); border-radius: 12px; padding: 14px; min-width: 0; box-shadow: var(--shadow); }
.card.wide { grid-column: 1 / -1; }
.card h2 { font-size: 15px; margin: 0 0 10px; display: flex; align-items: center; gap: 8px; }
.card h2 .sub { font-weight: 400; color: var(--muted); font-size: 13px; }
img.view { width: 100%; max-width: 100%; height: auto; display: block; border-radius: 8px; background: #000; min-height: 120px; }
img { max-width: 100%; }
.kv { display: grid; grid-template-columns: max-content minmax(0, 1fr); gap: 4px 14px; font-size: 13.5px; margin-top: 10px; }
.kv dt { color: var(--muted); } .kv dd { margin: 0; overflow-wrap: anywhere; }
.muted { color: var(--muted); } .ok { color: var(--ok); } .warn { color: var(--warn); } .bad { color: var(--bad); }
.small { font-size: 13px; }
.split { display: grid; grid-template-columns: minmax(0, 1.3fr) minmax(0, 1fr); gap: 12px; align-items: start; }
@media (max-width: 1000px) { .split { grid-template-columns: minmax(0, 1fr); } }
.board { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; max-width: 360px; }
.cell { aspect-ratio: 1 / 1; border-radius: 9px; border: 2px solid var(--line); display: flex; flex-direction: column;
  align-items: center; justify-content: center; text-align: center; padding: 4px; font-size: 12px; line-height: 1.25;
  background: var(--chip); color: var(--text); position: relative; user-select: none; min-width: 0; overflow: hidden; }
.cell .name { font-weight: 650; font-size: 13.5px; }
.cell span:not(.name):not(.coord) { font-size: 11.5px; opacity: .92; }
.board.wide-board { max-width: 440px; margin-top: 10px; }
.cell .coord { position: absolute; top: 3px; left: 6px; font-size: 10.5px; opacity: .7; }
.cell.c-red { background: var(--red); border-color: var(--red); color: #fff; }
.cell.c-yellow { background: var(--yellow); border-color: var(--yellow); color: #1c1c1c; }
.cell.c-green { background: var(--green); border-color: var(--green); color: #fff; }
.cell.c-blue { background: var(--blue); border-color: var(--blue); color: #fff; }
.cell.c-empty { background: transparent; border-style: dashed; color: var(--muted); }
.cell.c-unknown { background: repeating-linear-gradient(45deg, var(--chip), var(--chip) 6px, transparent 6px, transparent 12px); border-color: var(--warn); }
.cell.flag { box-shadow: 0 0 0 3px var(--warn); }
.cell.g-risky::after, .cell.g-blocked::after { content: "!"; position: absolute; top: 2px; right: 6px; font-weight: 800; }
.board.edit .cell { cursor: pointer; } .board.edit .cell:hover { filter: brightness(1.06); }
.toolbar { display: flex; flex-wrap: wrap; gap: 8px; margin: 10px 0; align-items: center; }
button, .button { font: inherit; font-size: 14px; border-radius: 8px; border: 1px solid var(--line); background: var(--chip);
  color: var(--text); padding: 7px 12px; cursor: pointer; display: inline-flex; align-items: center; gap: 6px; }
button:hover, .button:hover { filter: brightness(1.05); }
button.primary, .button.primary { background: var(--accent); border-color: var(--accent); color: #fff; }
button.danger { background: var(--bad); border-color: var(--bad); color: #fff; }
button:disabled { opacity: .5; cursor: default; }
input[type=file] { display: none; }
input[type=password], input[type=text] { font: inherit; padding: 7px 10px; border-radius: 8px; border: 1px solid var(--line);
  background: var(--bg); color: var(--text); width: 12em; }
.msg { font-size: 13.5px; margin: 6px 0; padding: 7px 10px; border-radius: 8px; background: var(--chip); }
.msg.ok { background: color-mix(in srgb, var(--ok) 14%, transparent); color: inherit; }
.msg.warn { background: color-mix(in srgb, var(--warn) 16%, transparent); color: inherit; }
.msg.bad { background: color-mix(in srgb, var(--bad) 15%, transparent); color: inherit; }
ul.list { margin: 6px 0; padding-left: 20px; } ul.list li { margin: 3px 0; }
ol.moves { margin: 8px 0; padding-left: 0; list-style: none; counter-reset: m; }
ol.moves li { counter-increment: m; display: grid; grid-template-columns: 26px 14px minmax(0, 1fr); gap: 4px 8px; align-items: baseline;
  padding: 6px 0; border-bottom: 1px solid var(--line); }
ol.moves li::before { content: counter(m) "."; color: var(--muted); text-align: right; }
.sw { width: 12px; height: 12px; border-radius: 3px; display: inline-block; transform: translateY(1px); }
.sw.red { background: var(--red); } .sw.yellow { background: var(--yellow); } .sw.green { background: var(--green); } .sw.blue { background: var(--blue); }
.sw.unknown { background: var(--muted); }
.legend { display: flex; flex-wrap: wrap; gap: 10px; font-size: 12.5px; color: var(--muted); margin-top: 8px; }
.photo { max-width: 100%; border-radius: 8px; margin-top: 6px; display: none; }
.hidden { display: none !important; }
</style></head><body>
<header>
  <h1>Brain · masa de lucru</h1>
  <div class="chips">
    <span class="chip">șablon <b id="h-location">__LOCATION__</b></span>
    <span class="chip"><span class="dot" id="h-live-dot"></span><span id="h-live">camera…</span></span>
    <span class="chip __WB_CLASS__"><span class="dot" id="h-svc-dot"></span><span id="h-svc">serviciul…</span></span>
    <span class="chip __EXEC_CLASS__"><span class="dot" id="h-exec-dot"></span><span id="h-exec">execuție</span></span>
  </div>
</header>
<main>
<section class="card">
  <h2>Live <span class="sub">fluxul camerei</span></h2>
  <img class="view" src="stream.mjpg" alt="fluxul live al camerei">
  <dl class="kv">
    <dt>Cadre/s</dt><dd id="l-fps">–</dd>
    <dt>Rezoluție</dt><dd id="l-res">–</dd>
    <dt>Sursa</dt><dd id="l-cam">–</dd>
    <dt>Analiza</dt><dd id="l-mode">–</dd>
    <dt>Ultima analiză</dt><dd id="l-last">–</dd>
  </dl>
  <p class="small muted">Peste flux: conturul fiecărui cub din ultima analiză, cu laturile (mm) și cel mai mic spațiu
  până la un vecin. Verde = pe centru, galben = cu offset, portocaliu/roșu = degetele ghearei nu încap.</p>
</section>
<section class="card">
  <h2>Ce vede brain-ul <span class="sub" id="a-sub"></span></h2>
  <img class="view" id="a-img" alt="analiza adnotată (apare după prima analiză)">
  <div id="a-reasons"></div>
  <div class="board wide-board" id="a-board"></div>
  <div class="legend"><span>„gol” = celulă liberă</span><span>„?” = nesigur</span><span>! = degetele ghearei nu încap</span>
    <span>„min” = cel mai mic spațiu până la un vecin</span></div>
</section>
"""

_STATUS = """<section class="card">
  <h2>Stare</h2>
  <dl class="kv" id="s-kv"></dl>
  <div id="s-msg"></div>
</section>
<section class="card">
  <h2>Ținta (S1) <span class="sub">doar culoarea feței de sus contează</span></h2>
  <div class="toolbar">
    <label class="button primary" for="t-file">Încarcă poza țintă</label>
    <input type="file" id="t-file" accept="image/*" capture="environment">
    <button id="t-from-camera" type="button">Copiază ce vede camera</button>
    <button id="t-clear" type="button">Golește</button>
  </div>
  <div id="t-msg"></div>
  <div class="split">
    <div>
      <div class="board edit" id="t-board"></div>
      <div class="toolbar">
        <button id="t-rot-l" type="button" title="rotește la stânga">⟲ 90°</button>
        <button id="t-rot-r" type="button" title="rotește la dreapta">⟳ 90°</button>
        <button id="t-confirm" class="primary" type="button">Confirmă ținta</button>
      </div>
      <p class="small muted">Click pe o celulă: gol → roșu → galben → verde → albastru. Fotografiați foaia de sus,
      cu robotul în partea de JOS a pozei (ca din camera brațului); altfel rotiți caroiajul.</p>
    </div>
    <div><img class="photo" id="t-photo" alt="poza țintă, cu analiza"></div>
  </div>
  <div id="t-saved" class="small muted"></div>
</section>
<section class="card wide">
  <h2>Plan <span class="sub">ce trebuie schimbat ca masa să arate ca ținta (brațul nu se mișcă)</span></h2>
  <div class="toolbar"><button id="p-run" class="primary" type="button">Calculează planul</button>
    <span class="small muted" id="p-when"></span></div>
  <div id="p-out" class="small muted">Starea curentă = ultima observație a camerei; ținta = cea confirmată.</div>
</section>
"""

_EXECUTE = """<section class="card wide" id="x-card">
  <h2>Execuție <span class="sub">brațul REAL se mișcă: mâna pe alimentare</span></h2>
  <div class="toolbar" id="x-unlock">
    <input type="password" id="x-token" placeholder="codul din consolă" autocomplete="off">
    <button id="x-check" type="button">Deblochează</button>
    <span class="small muted">Codul e afișat în consola Pi-ului la pornirea previzualizării.</span>
  </div>
  <div class="toolbar">
    <button id="x-run" class="danger" type="button" disabled>Execută ținta</button>
    <button id="x-stop" type="button" disabled>Oprește</button>
    <span class="small muted">Oprirea are loc după comanda curentă, fără home.</span>
  </div>
  <div id="x-msg"></div>
  <div id="x-out"></div>
</section>
"""

_SCRIPT = """</main>
<script>
"use strict";
const FEATURES = __FEATURES__;
const COLORS = ["red", "yellow", "green", "blue"];
const CYCLE = ["empty", "red", "yellow", "green", "blue"];
const RO = { red: "roșu", yellow: "galben", green: "verde", blue: "albastru", empty: "gol", unknown: "?" };
const FACE = { full: "plină", icon: "pictogramă", unknown: "" };
const GRASP = { centered: "pe centru", offset: "cu offset", risky: "riscant", blocked: "degetele ating vecinii" };
const $ = (id) => document.getElementById(id);
let editor = null, editorRows = 3, editorCols = 3, fromPhoto = false, lastUpdated = null, targetLocation = null;
let token = null;
try { token = sessionStorage.getItem("brainToken"); } catch (e) { token = null; }

function el(tag, cls, text) {
  const node = document.createElement(tag);
  if (cls) node.className = cls;
  if (text !== undefined && text !== null) node.textContent = text;
  return node;
}
function msg(box, kind, text) {
  const node = $(box); node.innerHTML = "";
  if (text) node.appendChild(el("div", "msg " + (kind || ""), text));
}
function list(box, items, cls) {
  const ul = el("ul", "list " + (cls || ""));
  items.forEach((t) => ul.appendChild(el("li", null, t)));
  box.appendChild(ul);
}
async function api(method, path, body, headers) {
  const opts = { method: method, headers: headers || {} };
  if (body !== undefined) opts.body = body;
  const response = await fetch(path, opts);
  let data = null;
  try { data = await response.json(); } catch (e) { data = null; }
  if (!response.ok) {
    const text = data && data.error ? data.error : "eroare HTTP " + response.status;
    const error = new Error(text); error.status = response.status; throw error;
  }
  return data;
}
function clock(iso) {
  if (!iso) return "–";
  const d = new Date(iso);
  return isNaN(d) ? iso : d.toLocaleTimeString("ro-RO");
}
function pct(x) { return x === null || x === undefined ? "–" : Math.round(x * 100) + "%"; }

/* ---------------------------------------------------------------- caroiajul observat */
function renderObserved(analysis) {
  const board = $("a-board"); board.innerHTML = "";
  const reasons = $("a-reasons"); reasons.innerHTML = "";
  if (!analysis) { board.appendChild(el("div", "muted small", "nicio analiză încă")); return; }
  if (analysis.error) { msg("a-reasons", "bad", "Analiza a eșuat: " + analysis.error); return; }
  $("a-sub").textContent = analysis.actionable ? "grila găsită" : "neacționabil";
  if (!analysis.actionable) {
    reasons.appendChild(el("div", "msg bad", "Imaginea nu permite decizii sigure:"));
    list(reasons, analysis.reasons_text || analysis.reasons || []);
  }
  const rows = analysis.rows || 3, cols = analysis.cols || 3;
  board.style.gridTemplateColumns = "repeat(" + cols + ", 1fr)";
  for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
    const key = "r" + r + "c" + c;
    const cell = (analysis.cells || {})[key];
    const node = el("div", "cell");
    node.appendChild(el("span", "coord", key));
    if (!cell) { node.classList.add("c-unknown"); node.appendChild(el("span", "name", "?")); board.appendChild(node); continue; }
    let detail = "";
    if (cell.state === "empty") {
      node.classList.add("c-empty"); node.appendChild(el("span", "name", "gol")); detail = pct(cell.occupancy_confidence);
    } else if (cell.state === "occupied" && COLORS.indexOf(cell.color) >= 0) {
      node.classList.add("c-" + cell.color); node.appendChild(el("span", "name", RO[cell.color]));
      detail = pct(cell.confidence) + (cell.face && FACE[cell.face] ? " · " + FACE[cell.face] : "");
      if (cell.uncertain) node.classList.add("flag");
    } else {
      node.classList.add("c-unknown"); node.appendChild(el("span", "name", "?"));
      detail = (cell.reasons_text || []).join("; ");
    }
    if (detail) node.appendChild(el("span", null, detail));
    if (cell.side_mm) node.appendChild(el("span", null, cell.side_mm.map((v) => Math.round(v)).join("×") + " mm"));
    if (cell.min_gap_mm !== null && cell.min_gap_mm !== undefined) node.appendChild(el("span", null, "min " + Math.round(cell.min_gap_mm) + " mm"));
    if (cell.grasp) { node.classList.add("g-" + cell.grasp); node.title = "prindere: " + (GRASP[cell.grasp] || cell.grasp); }
    if (cell.reasons_text && cell.reasons_text.length && cell.state === "occupied") node.title = (node.title ? node.title + "; " : "") + cell.reasons_text.join("; ");
    board.appendChild(node);
  }
}

/* ---------------------------------------------------------------- starea */
function renderStatus(s) {
  const info = s.info || {};
  $("l-fps").textContent = s.fps + " (" + s.frames + " cadre)";
  $("l-res").textContent = info.resolution || "–";
  $("l-cam").textContent = info.camera || "–";
  $("l-mode").textContent = { process: "proces separat", thread: "fir (în același proces)", none: "oprită" }[info.analysis_mode] || "–";
  const a = s.analysis;
  $("l-last").textContent = a && a.updated ? clock(a.updated) + " (acum " + (s.analysis_age_s === null ? "?" : s.analysis_age_s) + " s" + (a.processing_ms ? ", " + Math.round(a.processing_ms) + " ms" : "") + ")" : "–";
  $("h-location").textContent = s.location;
  const live = s.fps > 0;
  $("h-live-dot").className = "dot " + (live ? "ok" : "bad");
  $("h-live").textContent = live ? "camera " + s.fps + " fps" : "camera: niciun cadru";
  if (a && a.updated !== lastUpdated) {
    lastUpdated = a.updated;
    $("a-img").src = "annotated.jpg?t=" + encodeURIComponent(a.updated || Date.now());
    renderObserved(a);
  } else if (!a) renderObserved(null);
  if (FEATURES.workbench && s.workbench) renderWorkbench(s.workbench);
}

function renderWorkbench(w) {
  const kv = $("s-kv"); kv.innerHTML = "";
  const add = (k, v, cls) => { kv.appendChild(el("dt", null, k)); kv.appendChild(el("dd", cls || null, v)); };
  add("Șablonul activ", w.active_locations.join(", ") + (w.location ? " (camera: " + w.location + ")" : ""));
  const svc = w.service || {};
  if (svc.paused) {
    add("Serviciul de mișcare", "verificat la pornirea execuției (nu se interoghează în timpul mișcării)", "muted");
  } else if (!svc.ok) {
    add("Serviciul de mișcare", svc.error || "nu răspunde", "bad");
  } else {
    add("Serviciul de mișcare", (svc.sim ? "SIMULARE" : "braț REAL") + " · " + svc.url, svc.sim ? "warn" : "ok");
    add("Poze învățate", (svc.poses || []).join(", ") || "niciuna");
    add("Locuri calibrate", (svc.sites_text || []).join(", ") || "niciunul");
    if (svc.missing && svc.missing.length) add("Lipsesc", svc.missing.join(", "), "bad");
    add("Gheara în mm", svc.grip_calibrated ? "calibrată (" + svc.grip_range_mm.join("–") + " mm): regulile A și C active"
      : "necalibrată: serviciul deschide la GRIP_OPEN (npm run calib -- gheara UNGHI)", svc.grip_calibrated ? "ok" : "warn");
  }
  if (svc.checked_at) add("Verificat", clock(svc.checked_at));
  const g = w.gripper;
  if (g) add("Gheara (brain)", "max " + g.max_open_mm + " mm, deget " + g.finger_mm + " mm, nevoie " + g.side_space_mm + " mm liberi per latură");
  add("Execuția din pagină", w.execute_enabled ? (w.execute_locked ? "blocată (prea multe coduri greșite)" : "permisă, cu codul din consolă") : "dezactivată (preview.allow_execute: false)", w.execute_enabled ? "warn" : "muted");
  $("h-svc-dot").className = "dot " + (svc.paused ? "warn" : (svc.ok ? (svc.sim ? "warn" : "ok") : "bad"));
  $("h-svc").textContent = svc.paused ? "serviciul: în execuție" : (svc.ok ? (svc.sim ? "serviciul: simulare" : "serviciul: braț real") : "serviciul nu răspunde");
  if (w.target && w.target.confirmed) $("t-saved").textContent = "Ținta confirmată: " + w.target.confirmed.file + " (" + clock(w.target.confirmed.saved_at) + ")";
  if (FEATURES.execute) renderExecution(w.execution || {});
}

/* ---------------------------------------------------------------- editorul țintei */
function setEditor(cells, rows, cols) {
  editorRows = rows; editorCols = cols;
  editor = cells.map((row) => row.map((v) => (typeof v === "object" && v !== null ? v : { value: v })));
  drawEditor();
}
function blankEditor(rows, cols) {
  const cells = [];
  for (let r = 0; r < rows; r++) { cells.push([]); for (let c = 0; c < cols; c++) cells[r].push("empty"); }
  setEditor(cells, rows, cols);
}
function drawEditor() {
  const board = $("t-board"); board.innerHTML = "";
  board.style.gridTemplateColumns = "repeat(" + editorCols + ", 1fr)";
  for (let r = 0; r < editorRows; r++) for (let c = 0; c < editorCols; c++) {
    const item = editor[r][c];
    const value = item.value;
    const node = el("div", "cell c-" + (value === "unknown" ? "unknown" : value));
    node.appendChild(el("span", "coord", "r" + r + "c" + c));
    node.appendChild(el("span", "name", RO[value] || value));
    if (item.confidence !== undefined && item.confidence !== null && value !== "empty") node.appendChild(el("span", null, pct(item.confidence)));
    if (item.uncertain) { node.classList.add("flag"); node.title = item.note || "nesigur: verificați"; }
    node.addEventListener("click", () => {
      const next = CYCLE[(CYCLE.indexOf(value) + 1) % CYCLE.length];
      editor[r][c] = { value: next };
      drawEditor();
    });
    board.appendChild(node);
  }
}
function rotate(clockwise) {
  const rows = editorRows, cols = editorCols, out = [];
  for (let r = 0; r < cols; r++) {
    out.push([]);
    for (let c = 0; c < rows; c++) out[r].push(clockwise ? editor[rows - 1 - c][r] : editor[c][cols - 1 - r]);
  }
  editor = out; editorRows = cols; editorCols = rows; drawEditor();
}
async function loadTarget() {
  try {
    const t = await api("GET", "target");
    targetLocation = t.location;
    if (t.confirmed) { setEditor(t.confirmed.cells, t.rows, t.cols); fromPhoto = !!t.confirmed.photo; }
    else blankEditor(t.rows, t.cols);
    if (t.confirmed) $("t-saved").textContent = "Ținta confirmată: " + t.confirmed.file + " (" + clock(t.confirmed.saved_at) + ")";
    if (t.photo) showPhoto();
  } catch (e) { blankEditor(3, 3); msg("t-msg", "bad", e.message); }
}
function showPhoto() {
  const img = $("t-photo"); img.src = "target/annotated.jpg?t=" + Date.now(); img.style.display = "block";
}
async function uploadPhoto(file) {
  if (!file) return;
  if (file.size > 12 * 1024 * 1024) { msg("t-msg", "bad", "Poza are " + (file.size / 1048576).toFixed(1) + " MB; maximum 12 MB."); return; }
  msg("t-msg", "", "Se analizează poza…");
  try {
    const d = await api("POST", "target/image", file, { "Content-Type": file.type || "application/octet-stream" });
    targetLocation = d.location;
    setEditor(d.cells, d.rows, d.cols); fromPhoto = true;
    showPhoto();
    const box = $("t-msg"); box.innerHTML = "";
    box.appendChild(el("div", "msg " + (d.actionable ? (d.uncertain.length ? "warn" : "ok") : "bad"), d.message));
    if (d.reasons_text && d.reasons_text.length) list(box, d.reasons_text);
  } catch (e) { msg("t-msg", "bad", e.message); }
}
async function confirmTarget() {
  const rows = editor.map((row) => row.map((item) => item.value));
  const bad = [];
  rows.forEach((row, r) => row.forEach((v, c) => { if (v === "unknown") bad.push("r" + r + "c" + c); }));
  if (bad.length) { msg("t-msg", "bad", "Completați celulele nesigure (?): " + bad.join(", ")); return; }
  try {
    const t = await api("POST", "target", JSON.stringify({ location: targetLocation, rows: rows, from_photo: fromPhoto }),
      { "Content-Type": "application/json" });
    setEditor(t.confirmed.cells, t.rows, t.cols);
    msg("t-msg", "ok", "Ținta e salvată în " + t.confirmed.file + " (bună și pentru: brain run --task reproduce --target-file …).");
    $("t-saved").textContent = "Ținta confirmată: " + t.confirmed.file + " (" + clock(t.confirmed.saved_at) + ")";
  } catch (e) { msg("t-msg", "bad", e.message); }
}
function copyCamera() {
  fetch("status.json").then((r) => r.json()).then((s) => {
    const a = s.analysis;
    if (!a || !a.actionable) { msg("t-msg", "bad", "Camera nu vede acum caroiajul (analiza nu e acționabilă)."); return; }
    const cells = [];
    for (let r = 0; r < a.rows; r++) {
      cells.push([]);
      for (let c = 0; c < a.cols; c++) {
        const cell = a.cells["r" + r + "c" + c];
        let value = "unknown";
        if (cell && cell.state === "empty") value = "empty";
        else if (cell && cell.state === "occupied" && COLORS.indexOf(cell.color) >= 0) value = cell.color;
        cells[r].push({ value: value, confidence: cell ? cell.confidence : null, uncertain: !cell || !!cell.uncertain || value === "unknown" });
      }
    }
    targetLocation = a.location; fromPhoto = false; setEditor(cells, a.rows, a.cols);
    msg("t-msg", "ok", "Am copiat ce vede camera acum. Verificați și confirmați.");
  }).catch((e) => msg("t-msg", "bad", e.message));
}

/* ---------------------------------------------------------------- planul */
function renderMoves(box, moves) {
  const ol = el("ol", "moves");
  moves.forEach((m) => {
    const li = el("li");
    li.appendChild(el("span", "sw " + m.color));
    const body = el("div");
    body.appendChild(el("div", null, m.text));
    body.appendChild(el("div", "small muted", m.reason + (m.grip ? " · " + m.grip : "")));
    li.appendChild(body); ol.appendChild(li);
  });
  box.appendChild(ol);
}
async function computePlan() {
  const out = $("p-out"); out.innerHTML = ""; out.className = "small";
  out.appendChild(el("div", "muted", "Se calculează…"));
  try {
    const p = await api("POST", "plan");
    out.innerHTML = "";
    $("p-when").textContent = "calculat la " + clock(p.computed_at) + (p.observation_age_s !== null ? ", observația are " + p.observation_age_s + " s" : "");
    if (p.problems && p.problems.length) { out.appendChild(el("div", "msg bad", "Planul nu se poate calcula:")); list(out, p.problems); return; }
    if (p.already_done) out.appendChild(el("div", "msg ok", "Masa arată deja ca ținta: nimic de făcut."));
    else if (p.moves.length) out.appendChild(el("div", "msg " + (p.complete ? "ok" : "warn"), p.moves.length + " mutări, ~" + Math.round(p.estimated_s) + " s estimat" + (p.complete ? "; după ele, masa arată ca ținta." : "; ținta NU se atinge complet (vezi mai jos).")));
    renderMoves(out, p.moves);
    if (p.unmet.length) { out.appendChild(el("div", "msg bad", "Ce nu se poate face:")); list(out, p.unmet); }
    if (p.warnings.length) { out.appendChild(el("div", "msg warn", "Atenție:")); list(out, p.warnings); }
  } catch (e) { out.innerHTML = ""; out.appendChild(el("div", "msg bad", e.message)); }
}

/* ---------------------------------------------------------------- execuția */
const OUTCOME = { ok: "reușită", pick_missed: "cub neprins", dropped: "cub căzut", mismatch: "altceva decât se aștepta",
  unreachable_pick: "pick inaccesibil", unreachable_place: "place inaccesibil", "": "în curs" };
const XSTATE = { idle: "nicio execuție", running: "în curs", done: "terminată", error: "oprită cu eroare", stopped: "oprită la cerere" };
function renderExecution(x) {
  if (!$("x-card")) return;
  const running = x.state === "running";
  $("x-unlock").classList.toggle("hidden", !!token && !x.locked);
  $("x-run").disabled = running || !token || x.locked;
  $("x-stop").disabled = !running || x.stop_requested;
  $("h-exec-dot").className = "dot " + (running ? "warn" : (x.state === "error" ? "bad" : (x.state === "done" ? "ok" : "")));
  $("h-exec").textContent = "execuție: " + (XSTATE[x.state] || x.state || "–");
  const out = $("x-out"); out.innerHTML = "";
  if (x.locked) out.appendChild(el("div", "msg bad", "Prea multe coduri greșite: execuția e blocată până la repornirea previzualizării."));
  if (!x.state || x.state === "idle") return;
  const head = (XSTATE[x.state] || x.state) + (x.fsm_state ? " · FSM " + x.fsm_state : "") + (x.elapsed_s !== null && x.elapsed_s !== undefined ? " · " + Math.round(x.elapsed_s) + " s" : "") + (x.stop_requested && running ? " · oprire cerută" : "");
  out.appendChild(el("div", "msg " + (running ? "warn" : (x.state === "done" && x.goal_reached ? "ok" : "bad")), head));
  if (x.error) out.appendChild(el("div", "msg bad", x.error));
  if (x.moves && x.moves.length) {
    const ol = el("ol", "moves");
    x.moves.forEach((m) => {
      const li = el("li"); li.appendChild(el("span", "sw " + (m.color || "unknown")));
      const body = el("div"); body.appendChild(el("div", null, m.move));
      body.appendChild(el("div", "small " + (m.outcome === "ok" ? "ok" : (m.outcome ? "bad" : "muted")), (OUTCOME[m.outcome] || m.outcome) + (m.detail ? " · " + m.detail : "") + (m.duration_s ? " · " + m.duration_s + " s" : "")));
      li.appendChild(body); ol.appendChild(li);
    });
    out.appendChild(ol);
  }
  if (running && x.remaining && x.remaining.length) { out.appendChild(el("div", "small muted", "Planul curent:")); list(out, x.remaining, "small"); }
  if (x.final_check) {
    const f = x.final_check;
    out.appendChild(el("div", "msg " + (f.ok ? "ok" : "bad"), "Verificarea vizuală finală: " + f.summary));
    if (f.cells) {
      const board = el("div", "board"); board.style.gridTemplateColumns = "repeat(" + f.cells[0].length + ", 1fr)";
      f.cells.forEach((row, r) => row.forEach((c, k) => {
        const node = el("div", "cell c-" + (c.seen === "unknown" ? "unknown" : c.seen));
        node.appendChild(el("span", "coord", "r" + r + "c" + k));
        node.appendChild(el("span", "name", c.ok ? "✓" : "✗"));
        node.appendChild(el("span", null, "cerut " + (RO[c.want] || c.want) + ", văzut " + (RO[c.seen] || c.seen)));
        if (!c.ok) node.classList.add("flag");
        board.appendChild(node);
      }));
      out.appendChild(board);
    }
  }
  if (x.out_dir && !running) out.appendChild(el("div", "small muted", "Raport, durate, memorie: " + x.out_dir));
}
async function checkToken() {
  const value = $("x-token").value.trim();
  if (!value) return;
  try {
    await api("POST", "execute/token", undefined, { "X-Brain-Token": value });
    token = value; try { sessionStorage.setItem("brainToken", value); } catch (e) { /* fără stocare */ }
    msg("x-msg", "ok", "Cod corect: execuția e deblocată pentru această filă.");
  } catch (e) { msg("x-msg", "bad", e.message); }
}
async function execute() {
  if (!confirm("Brațul REAL se va mișca pentru ținta confirmată. Mâna pe alimentare. Continuați?")) return;
  try {
    await api("POST", "execute", undefined, { "X-Brain-Token": token || "" });
    msg("x-msg", "warn", "Execuția a pornit.");
  } catch (e) {
    if (e.status === 403) { token = null; try { sessionStorage.removeItem("brainToken"); } catch (err) { /* */ } }
    msg("x-msg", "bad", e.message);
  }
}
async function stop() {
  try { await api("POST", "execute/stop"); msg("x-msg", "warn", "Oprire cerută: se termină comanda curentă, fără home."); }
  catch (e) { msg("x-msg", "bad", e.message); }
}

/* ---------------------------------------------------------------- pornire */
async function poll() {
  try { renderStatus(await api("GET", "status.json")); }
  catch (e) { $("h-live-dot").className = "dot bad"; $("h-live").textContent = "pagina nu primește date (" + e.message + ")"; }
  setTimeout(poll, 1000);
}
if (FEATURES.workbench) {
  $("t-file").addEventListener("change", (ev) => { uploadPhoto(ev.target.files[0]); ev.target.value = ""; });
  $("t-clear").addEventListener("click", () => { fromPhoto = false; blankEditor(editorRows, editorCols); $("t-photo").style.display = "none"; msg("t-msg", "", ""); });
  $("t-from-camera").addEventListener("click", copyCamera);
  $("t-rot-l").addEventListener("click", () => rotate(false));
  $("t-rot-r").addEventListener("click", () => rotate(true));
  $("t-confirm").addEventListener("click", confirmTarget);
  $("p-run").addEventListener("click", computePlan);
  loadTarget();
}
if (FEATURES.execute) {
  $("x-check").addEventListener("click", checkToken);
  $("x-token").addEventListener("keydown", (ev) => { if (ev.key === "Enter") checkToken(); });
  $("x-run").addEventListener("click", execute);
  $("x-stop").addEventListener("click", stop);
}
poll();
</script>
</body></html>
"""


def render_page(location: str, workbench: bool = False, execute: bool = False) -> str:
    """Pagina, cu panourile mesei de lucru doar dacă există `api` și cu „Execuție” doar dacă e permisă."""
    page = _HEAD + (_STATUS if workbench else "") + (_EXECUTE if workbench and execute else "") + _SCRIPT
    features = json.dumps({"workbench": bool(workbench), "execute": bool(workbench and execute)})
    return (page.replace("__LOCATION__", html.escape(location))
                .replace("__WB_CLASS__", "" if workbench else "hidden")
                .replace("__EXEC_CLASS__", "" if workbench and execute else "hidden")
                .replace("__FEATURES__", features))
