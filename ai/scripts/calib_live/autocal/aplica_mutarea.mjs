// Aplică mutarea foii (din mutare_foaie.py) peste caroiajul `small` din calib.json.
// Mutarea e în milimetrii foii (coloane 45, linii 43); modelul învățat al caroiajului e în coordonatele serviciului,
// așa că punctele trec prin celulele foii vechi: mm -> (u, v) în celule -> modelul învățat (O, ecol, erow).
//   node ai/scripts/calib_live/autocal/aplica_mutarea.mjs mutare.json
import { readFileSync } from "node:fs";
const ROOT = new URL("../../../../", import.meta.url).pathname;
const { loadCalib, saveCalib, gridOf, requireSite } = await import(ROOT + "dist/src/calib.js");
const { Grid } = await import(ROOT + "dist/src/grid.js");
const { ik } = await import(ROOT + "dist/src/kinematics.js");

const { R, t } = JSON.parse(readFileSync(process.argv[2], "utf8"));
const path = ROOT + "calib.json";
const calib = loadCalib(path);
const g = gridOf(requireSite(calib, "small"));
const f1 = (v) => v.toFixed(1);
const pts = {};
for (const [r, c] of [[0, 0], [0, 2], [2, 0], [2, 2]]) {
  const gx = 45 * (c + 0.5), gy = 43 * (r + 0.5);
  const X = R[0][0] * gx + R[0][1] * gy + t[0];
  const Y = R[1][0] * gx + R[1][1] * gy + t[1];
  const u = X / 45 - 0.5, v = Y / 43 - 0.5; // față de centrul r0c0 al foii vechi, în celule
  const x = g.O[0] + u * g.ecol[0] + v * g.erow[0];
  const y = g.O[1] + u * g.ecol[1] + v * g.erow[1];
  pts[`r${r}c${c}`] = [x, y, g.z];
  const [ox, oy] = g.cellXy(r, c);
  console.log(`r${r}c${c}: (${f1(ox)}, ${f1(oy)}) -> (${f1(x)}, ${f1(y)}) z ${f1(g.z)}`);
}
const n = Grid.fromPoints(pts).grid;
for (let r = 0; r < 3; r++) {
  for (let c = 0; c < 3; c++) {
    const [x, y] = n.cellXy(r, c);
    if (!ik(x, y, n.z) || !ik(x, y, n.z + 4 * 29 + 45)) throw new Error(`r${r}c${c} ar fi inaccesibilă: nimic scris`);
  }
}
calib.sites.small = { kind: "grid", points: pts };
saveCalib(calib, path);
console.log(`Salvat calib.json: ${n.info()}`);
