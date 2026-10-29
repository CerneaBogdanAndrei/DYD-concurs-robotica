// Calibrare prin învățare cu mâna, portată din referinta-python/calibrate.py.
// Cuplul se oprește, brațul e poziționat cu mâna, unghiurile citite trec prin FK.
//
//   npm run calib -- poza observe_source     # poza de observare (camera vede tot caroiajul)
//   npm run calib -- celule source           # gheara pe r0c0, r0c2, r2c0, r2c2 -> modelul caroiajului
//   npm run calib -- celule dest --celule r0c0 r0c2 r2c0 r2c2 r1c1
//   npm run calib -- zona zone_red           # o singură poziție (zonă de sortare)
//   npm run calib -- test source 1 1         # brațul coboară pe celula r1c1 (verificare)
//   npm run calib -- test zone_red
//   npm run calib -- citeste                 # unghiuri + FK în timp real, cu cuplul oprit
//   npm run calib -- gheara 60               # doar gheara la unghiul 60; măsurați deschiderea interioară (mm)
//   npm run calib -- gheara                  # tabelul măsurat (unghi -> mm)
//
// Opțiuni: --calib <fișier> (implicit calib.json), --sim (placă simulată, fără I2C).
// Colțurile caroiajului în imagine și pragurile HSV țin de viziune (creierul).

import { createInterface } from "node:readline/promises";
import { parseArgs } from "node:util";
import type { DofbotArm } from "../arm/arm.js";
import { openI2cArm } from "../arm/i2c.js";
import { openSimArm } from "../arm/sim-board.js";
import { type Calib, gridOf, gripTable, loadCalib, requireSite, saveCalib } from "../calib.js";
import * as C from "../config.js";
import { Grid, type Vec3, parseCellLabel } from "../grid.js";
import { fk } from "../kinematics.js";
import { Robot } from "../robot.js";

const USAGE = `Utilizare: npm run calib -- <poza NUME | celule LOC [--celule r0c0 ...] | zona NUME | test LOC [r c] | citeste | gheara [UNGHI]> [--sim] [--calib FIȘIER]`;

const { values, positionals } = parseArgs({
  allowPositionals: true,
  options: {
    sim: { type: "boolean", default: false },
    calib: { type: "string" },
    celule: { type: "string", multiple: true },
  },
});

const rl = createInterface({ input: process.stdin, output: process.stdout });
const f1 = (v: number) => v.toFixed(1);

/** Oprește cuplul, utilizatorul poziționează brațul cu mâna, citește unghiurile. */
async function teach(robot: Robot, mesaj: string): Promise<number[]> {
  for (;;) {
    await robot.torque(false);
    await rl.question(`\n>>> ${mesaj}\n    Apăsați Enter când e gata (țineți brațul nemișcat)... `);
    const s = await robot.read();
    await robot.torque(true);
    if (s.slice(0, 5).some((v) => v === null)) {
      console.log("Citire eșuată, reîncercați.", s);
      continue;
    }
    // Ține poziția. Gheara citită poate fi în afara limitelor de comandă: o aducem în ele.
    const grip = Math.min(C.JOINT_MAX[5]!, Math.max(C.JOINT_MIN[5]!, s[5] ?? robot.grip));
    const held = [...(s.slice(0, 5) as number[]), grip];
    await robot.write6(held, 300);
    robot.grip = grip;
    return held;
  }
}

async function cmdPoza(robot: Robot, calib: Calib, path: string, name: string) {
  const s = await teach(
    robot,
    `Așezați brațul astfel încât camera să vadă TOT locul pentru '${name}', cât mai de sus și cât mai perpendicular.`,
  );
  const obs = s.slice(0, 5).map((v) => Math.trunc(v));
  await robot.pose(obs, 500);
  calib.poses[name] = obs;
  saveCalib(calib, path);
  console.log(`Salvat: poza '${name}' = [${obs.join(", ")}].`);
}

async function cmdCelule(robot: Robot, calib: Calib, path: string, loc: string, labels: string[]) {
  labels.forEach(parseCellLabel); // validează înainte de prima mișcare
  const pts: Record<string, Vec3> = {};
  for (const lab of labels) {
    const s = await teach(
      robot,
      `Puneți un cub în celula ${lab.toUpperCase()} a locului '${loc}' (centrat). ` +
        "Aduceți gheara DESCHISĂ în jurul lui, la jumătatea înălțimii cubului, cât mai VERTICALĂ.",
    );
    const p = fk(s);
    pts[lab] = [p.x, p.y, p.z];
    console.log(`    ${lab}: servo=[${s.slice(0, 5).join(", ")}] -> x=${f1(p.x)} y=${f1(p.y)} z=${f1(p.z)} mm, înclinare ${p.pitch.toFixed(0)} grd`);
    if (!(p.z > -15 && p.z < 60)) {
      console.log("    ATENȚIE: z ar trebui să fie ~ jumătate din latura cubului. Verificați L1/L4 din config.ts.");
    }
    await robot.pose(C.HOME, 1000);
  }
  const { grid, residuals } = Grid.fromPoints(pts);
  console.log("\n" + grid.info());
  for (const r of residuals) console.log(`    eroare de potrivire ${r.label}: ${f1(r.errMm)} mm`);
  calib.sites[loc] = { kind: "grid", points: pts };
  saveCalib(calib, path);
  console.log(`Salvat caroiajul '${loc}'.`);
}

async function cmdZona(robot: Robot, calib: Calib, path: string, name: string) {
  const s = await teach(
    robot,
    `Puneți un cub în zona '${name}'. Aduceți gheara DESCHISĂ în jurul lui, la jumătatea înălțimii cubului, cât mai VERTICALĂ.`,
  );
  const p = fk(s);
  console.log(`    ${name}: x=${f1(p.x)} y=${f1(p.y)} z=${f1(p.z)} mm, înclinare ${p.pitch.toFixed(0)} grd`);
  await robot.pose(C.HOME, 1000);
  calib.sites[name] = { kind: "point", xyz: [p.x, p.y, p.z] };
  saveCalib(calib, path);
  console.log(`Salvată zona '${name}'.`);
}

async function cmdTest(robot: Robot, calib: Calib, loc: string, rc: string[]) {
  const site = requireSite(calib, loc);
  let x: number, y: number, z: number, yaw: number;
  if (site.kind === "grid") {
    if (rc.length !== 2) throw new Error("pentru un caroiaj dați linia și coloana (0..2), ex: test source 1 1");
    const [r, c] = parseCellLabel(`r${rc[0]}c${rc[1]}`);
    const g = gridOf(site);
    [x, y] = g.cellXy(r, c);
    z = g.zAt(c + 0.5, r + 0.5);
    yaw = g.yawRobot(0);
    console.log(`r${r}c${c}: x=${f1(x)} y=${f1(y)} z=${f1(z)}`);
  } else {
    [x, y, z] = site.xyz;
    z += C.GRASP_DZ;
    yaw = 0;
    console.log(`${loc}: x=${f1(x)} y=${f1(y)} z=${f1(z)}`);
  }
  await robot.gripper(C.GRIP_OPEN);
  await robot.goto(x, y, z + C.HOVER_DZ, yaw);
  await robot.goto(x, y, z, yaw, null, C.T_FINE * 2);
  await rl.question("Gheara trebuie să fie centrată pe celulă și paralelă cu liniile. Enter = sus ");
  await robot.goto(x, y, z + C.HOVER_DZ, yaw, null, C.T_FINE);
  await robot.home();
}

function printGripper(calib: Calib) {
  const pts = [...(calib.gripper ?? [])].sort((a, b) => a[0] - b[0]);
  if (pts.length === 0) console.log("Gheara: niciun unghi măsurat.");
  for (const [angle, mm] of pts) console.log(`    servo 6 = ${angle}  ->  ${mm} mm`);
  const ok = gripTable(calib).length > 0;
  console.log(
    ok
      ? "Gheara e calibrată în mm: pick/place folosesc deschiderile de abordare și de eliberare (regulile A, C)."
      : "Gheara NU e calibrată în mm (trebuie cel puțin 2 unghiuri, mm diferiți): pick/place deschid la GRIP_OPEN.",
  );
}

/** Pune doar gheara la un unghi; utilizatorul măsoară deschiderea INTERIOARĂ cu șublerul. */
async function cmdGheara(robot: Robot, calib: Calib, path: string, arg: string | undefined) {
  if (arg === undefined) return printGripper(calib);
  const angle = Number(arg);
  if (!Number.isInteger(angle) || angle < C.JOINT_MIN[5]! || angle > C.JOINT_MAX[5]!) {
    throw new Error(`unghiul ghearei trebuie întreg ${C.JOINT_MIN[5]}..${C.JOINT_MAX[5]} (GRIP_CLOSED=${C.GRIP_CLOSED})`);
  }
  await robot.gripper(angle);
  const text = await rl.question(
    `Gheara e la ${angle}. Măsurați distanța INTERIOARĂ dintre degete, în mm (Enter = nu salvez): `,
  );
  const mm = Number(text.trim().replace(",", "."));
  if (text.trim() === "" || !Number.isFinite(mm) || mm <= 0 || mm > C.GRIP_MAX_OPEN_MM + 20) {
    console.log("Nesalvat.");
    return printGripper(calib);
  }
  calib.gripper = [...(calib.gripper ?? []).filter((p) => p[0] !== angle), [angle, mm]];
  saveCalib(calib, path);
  console.log(`Salvat: servo 6 = ${angle} -> ${mm} mm.`);
  printGripper(calib);
}

async function cmdCiteste(robot: Robot) {
  await robot.torque(false);
  console.log("Cuplu oprit. Mișcați brațul; Ctrl+C pentru ieșire.");
  let stop = false;
  rl.on("SIGINT", () => (stop = true));
  process.on("SIGINT", () => (stop = true));
  while (!stop) {
    const s = await robot.read();
    if (!s.slice(0, 4).includes(null)) {
      const p = fk(s as number[]);
      process.stdout.write(
        `\rservo [${s.join(", ")}]  ->  x=${f1(p.x).padStart(6)} y=${f1(p.y).padStart(6)} z=${f1(p.z).padStart(6)} pitch=${p.pitch.toFixed(0).padStart(5)}   `,
      );
    }
    await new Promise((r) => setTimeout(r, 300));
  }
  await robot.torque(true);
  console.log();
}

async function main() {
  const [cmd, ...args] = positionals;
  const path = values.calib ?? C.CALIB_FILE;
  if (!cmd) throw new Error(USAGE);
  const calib = loadCalib(path);

  let arm: DofbotArm;
  if (values.sim) {
    arm = openSimArm().arm;
    console.log("[sim] placă simulată: brațul nu poate fi mișcat cu mâna, se citește poziția curentă.");
  } else {
    arm = await openI2cArm();
  }
  const robot = await Robot.create(arm, { sim: values.sim, log: () => {} });
  try {
    if (cmd === "poza" && args[0]) await cmdPoza(robot, calib, path, args[0]);
    else if (cmd === "celule" && args[0]) {
      await cmdCelule(robot, calib, path, args[0], values.celule ?? ["r0c0", "r0c2", "r2c0", "r2c2"]);
    } else if (cmd === "zona" && args[0]) await cmdZona(robot, calib, path, args[0]);
    else if (cmd === "test" && args[0]) await cmdTest(robot, calib, args[0], args.slice(1));
    else if (cmd === "citeste") await cmdCiteste(robot);
    else if (cmd === "gheara") await cmdGheara(robot, calib, path, args[0]);
    else throw new Error(USAGE);
  } finally {
    rl.close();
    await arm.close();
  }
}

main().catch((e: unknown) => {
  console.error(e instanceof Error ? e.message : e);
  process.exit(1);
});
