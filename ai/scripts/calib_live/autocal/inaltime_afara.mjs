// Înălțimea vârfurilor degetelor în afara caroiajului, în poza de pick: din ea vin corecțiile `z_out` din calib.json
// (ca `z_rows` pe r2). Pe r1c1 vârfurile sunt la 21 mm deasupra foii în poza de pick: corecția = 21 - măsurat.
//
//   node ai/scripts/calib_live/autocal/inaltime_afara.mjs jos X Y     # gheara, deschisă ca la pick, coboară deasupra
//                                                                      # punctului (X, Y mm pe foaie) și rămâne acolo
//   node ai/scripts/calib_live/autocal/inaltime_afara.mjs nota MM     # vârfurile la MM mm deasupra foii (rigla)
//   node ai/scripts/calib_live/autocal/inaltime_afara.mjs sus         # urcă pe verticală, apoi home -> poza de start
//
// Foaia: x spre c2 de la marginea stângă a caroiajului, y spre robot de la marginea depărtată (celula = 33 mm).
// Mișcările trec prin serviciu (move_joints, go_pose); punctul și pașii coborârii sunt cei ai pick-ului (IK, înclinarea
// comună cu apropierea de la HOVER_DZ, 6 pași pe verticală). Măsurătorile se adaugă în `inaltime_afara.json`.
// Încheietura stă dreaptă (servo 5 la J5_CENTER): cu gheara înclinată, o rotire a încheieturii înclină și linia
// degetelor (2026-10-30, în stânga lui r1, rotită 16 grd ca să alinieze degetele cu caroiajul: vârfurile la 25 și
// 38 mm; dreaptă: ambele la 30 mm). Pick-ul în afara caroiajului se face tot cu încheietura dreaptă, iar cubul se
// aliniază singur între degete.
import fs from "node:fs";

const ROOT = new URL("../../../../", import.meta.url).pathname;
const R = ROOT + "dist/src";
const { ik } = await import(R + "/kinematics.js");
const { loadCalib, requireSite, gridOf } = await import(R + "/calib.js");
const C = await import(R + "/config.js");
const OUT = process.env.INALTIME_JSON ?? new URL("./inaltime_afara.json", import.meta.url);
const SERVICE = process.env.SERVICE ?? "http://127.0.0.1:8765"; // SERVICE=http://127.0.0.1:8766 pentru simulare
const R1C1_TIPS_MM = 21; // vârfurile degetelor deasupra foii în poza de pick pe r1c1 (rigla, 2026-10-30)

async function post(method, body = {}) {
  const res = await fetch(`${SERVICE}/${method}`, { method: "POST", body: JSON.stringify(body) });
  const out = await res.json();
  if (!res.ok || out.error) throw new Error(`${method}: ${JSON.stringify(out)}`);
  return out.result;
}

const state = fs.existsSync(OUT) ? JSON.parse(fs.readFileSync(OUT)) : { current: null, measured: [] };
const save = () => fs.writeFileSync(OUT, JSON.stringify(state, null, 1));
const g = gridOf(requireSite(loadCalib(ROOT + "calib.json"), "small"));

/** Unghiurile (servo 1..5) pentru gheara în (x, y, z), cu înclinarea p sau cea mai apropiată posibilă. */
function joints(x, y, z, p) {
  const near = [...C.PITCH_CANDIDATES].sort((a, b) => Math.abs(a - p) - Math.abs(b - p));
  const sol = ik(x, y, z, [p]) ?? ik(x, y, z, near);
  if (!sol) throw new Error(`punct inaccesibil x=${x.toFixed(0)} y=${y.toFixed(0)} z=${z.toFixed(0)}`);
  return [...sol.servos, C.J5_CENTER];
}

async function line(pt, z0, z1) {
  for (let i = 1; i <= C.VERTICAL_STEPS; i++) {
    const z = z0 + ((z1 - z0) * i) / C.VERTICAL_STEPS;
    await post("move_joints", { angles: [...joints(pt.x, pt.y, z, pt.pitch), C.GRIP_OPEN], time_ms: 250 });
  }
}

const [cmd, a1, a2] = process.argv.slice(2);
if (cmd === "jos") {
  if (state.current) throw new Error("gheara e deja jos: întâi `sus`");
  const X = Number(a1), Y = Number(a2);
  const u = X / 33, v = Y / 33;
  const [x, y] = g.uvToXy(u, v);
  const z = g.zAt(u, v);
  const pitch = C.PITCH_CANDIDATES.find((p) => ik(x, y, z, [p]) && ik(x, y, z + C.HOVER_DZ, [p])) ??
    C.PITCH_CANDIDATES.find((p) => ik(x, y, z, [p]));
  if (pitch === undefined) throw new Error("punct inaccesibil");
  const pt = { X, Y, u, v, x, y, z, pitch };
  await post("beep", { n: 2 });
  await post("go_pose", { name: "home" }); // gheara se deschide la GRIP_OPEN, ca la pick
  await post("move_joints", { angles: [...joints(x, y, z + C.HOVER_DZ, pitch), C.GRIP_OPEN], time_ms: 2000 });
  await line(pt, z + C.HOVER_DZ, z);
  state.current = { ...pt, servo: await post("read_joints") };
  save();
  console.log(`jos în (${X}; ${Y}) mm, uv (${u.toFixed(2)}; ${v.toFixed(2)}): robot (${x.toFixed(1)}; ${y.toFixed(1)}), ` +
    `z prindere ${z.toFixed(1)} (r1c1 ${g.zAt(1.5, 1.5).toFixed(1)}), înclinare ${pitch}; citit [${state.current.servo}]`);
} else if (cmd === "nota") {
  if (!state.current) throw new Error("gheara nu e jos");
  const mm = Number(a1);
  const dz = R1C1_TIPS_MM - mm;
  state.measured.push({ ...state.current, tips_mm: mm, dz, servo_measured: await post("read_joints"),
    at: new Date().toISOString() });
  save();
  console.log(`(${state.current.X}; ${state.current.Y}): vârfurile la ${mm} mm -> corecția ${dz >= 0 ? "+" : ""}${dz} mm; ` +
    `z_out: [${state.current.u.toFixed(2)}, ${state.current.v.toFixed(2)}, ${dz}]`);
} else if (cmd === "sus") {
  const pt = state.current;
  if (pt) await line(pt, pt.z, pt.z + C.HOVER_DZ);
  state.current = null;
  save();
  await post("go_pose", { name: "home" });
  await post("go_pose", { name: "observe_small", speed: "slow" });
  await post("beep", { n: 1 });
  console.log("sus, în poza de start");
} else {
  console.log("folosire: jos X Y | nota MM | sus");
}
