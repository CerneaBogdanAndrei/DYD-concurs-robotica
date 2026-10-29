// Pick/place prin Robot + ArmService.resolve, cu gheara dată aici (OPEN din mediu, implicit 0; strâns 103 pentru cubul de 40 mm),
// fără schimbări în config. Oprește fără place dacă servo 6 ajunge la comandă (gheara goală).
//   OPEN=0 node ai/scripts/calib_live/autocal/proba_pick_place.mjs uv:1.53,1.43 00     (de la uv la celula r0c0)
import { readFileSync } from "node:fs";
const ROOT = new URL("../../../../", import.meta.url).pathname;
const R = ROOT + "dist/src";
const { openI2cArm } = await import(R + "/arm/i2c.js");
const { calibLoader, requirePose } = await import(R + "/calib.js");
const C = await import(R + "/config.js");
const { Robot } = await import(R + "/robot.js");
const { ArmService } = await import(R + "/service.js");
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const OPEN = Number(process.env.OPEN ?? 0), CLOSE = 103;
const spec = (s) => s.startsWith("uv:") ? { uv: s.slice(3).split(",").map(Number) } : { cell: [Number(s[0]), Number(s[1])] };
const [from, to] = process.argv.slice(2).map(spec);
const name = (t) => t.uv ? `uv(${t.uv.join(", ")})` : `r${t.cell[0]}c${t.cell[1]}`;
const load = calibLoader(ROOT + "calib.json");
const arm = await openI2cArm();
const robot = await Robot.create(arm, { log: () => {} });
const gr = robot.gripper.bind(robot);
robot.gripper = (v, t) => gr(v === C.GRIP_CLOSED ? CLOSE : v, t);
const svc = new ArmService(robot, load, false);
try {
  for (let i = 0; i < 2; i++) { await arm.buzzerOn(1); await sleep(300); }
  const a = svc.resolve("small", from.cell ?? null, 0, 0, from.uv), b = svc.resolve("small", to.cell ?? null, 0, 0, to.uv);
  console.log(`pick ${name(from)} (${a.x.toFixed(1)}, ${a.y.toFixed(1)}, ${a.z.toFixed(1)}), gheara deschisă la ${OPEN}`);
  await robot.pick(a.x, a.y, a.z, a.yaw, OPEN);
  const held = await arm.readAll();
  console.log(`  ținut: servo [${robot.cur.join(", ")}], citit [${held.join(", ")}]`);
  if (held[5] === null || held[5] > CLOSE - 4) {
    console.log("  GHEARA GOALĂ (servo 6 a ajuns la comandă): opresc, fără place");
    await robot.gripper(OPEN);
    await robot.home();
    await robot.pose(requirePose(load(), "observe_small"), 1800);
    process.exit(3);
  }
  console.log(`  cub prins (servo 6 oprit la ${held[5]}). place ${name(to)} (${b.x.toFixed(1)}, ${b.y.toFixed(1)}, ${b.z.toFixed(1)})`);
  await robot.place(b.x, b.y, b.z, b.yaw, OPEN);
  await robot.home();
  await robot.pose(requirePose(load(), "observe_small"), 1800);
} finally {
  await arm.close();
}
