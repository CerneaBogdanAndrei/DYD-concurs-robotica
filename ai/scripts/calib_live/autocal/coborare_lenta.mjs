// Coborâre lentă cu gheara deschisă deasupra unui punct uv (celule), opriri la +70 / +40 / 0 mm cu 1 / 2 / 3 bipuri; echipa privește.
//   node ai/scripts/calib_live/autocal/coborare_lenta.mjs 1.5,1.5
const ROOT = new URL("../../../../", import.meta.url).pathname;
const R = ROOT + "dist/src";
const { openI2cArm } = await import(R + "/arm/i2c.js");
const { calibLoader, requirePose } = await import(R + "/calib.js");
const C = await import(R + "/config.js");
const { ik, fk } = await import(R + "/kinematics.js");
const { Robot } = await import(R + "/robot.js");
const { ArmService } = await import(R + "/service.js");
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const load = calibLoader(ROOT + "calib.json");
const arm = await openI2cArm();
const robot = await Robot.create(arm, { log: () => {} });
const svc = new ArmService(robot, load, false);
const uv = process.argv[2].split(",").map(Number);
const t = svc.resolve("small", null, 0, 0, uv);
const p = C.PITCH_CANDIDATES.find((p) => ik(t.x, t.y, t.z, [p]) && ik(t.x, t.y, t.z + 70, [p]));
const beep = async (n) => { for (let i = 0; i < n; i++) { await arm.buzzerOn(1); await sleep(250); } };
try {
  await arm.writeServo(6, 0, 600); robot.grip = 0; await sleep(800);
  const stages = [[70, 1, 10000], [40, 2, 10000], [0, 3, 12000]];
  let zPrev = null;
  for (const [dz, n, hold] of stages) {
    const z = t.z + dz;
    if (zPrev === null) await robot.goto(t.x, t.y, z, t.yaw, [p], 1800);
    else for (let i = 1; i <= 6; i++) await robot.goto(t.x, t.y, zPrev + ((z - zPrev) * i) / 6, t.yaw, [p], 350);
    zPrev = z;
    await beep(n);
    const rd = await arm.readAll();
    console.log(`${n} bip: punct de prindere x=${t.x.toFixed(1)} y=${t.y.toFixed(1)} z=${z.toFixed(1)} (prindere +${dz} mm), înclinare ${p}, servo [${robot.cur.join(", ")}], citit [${rd.join(", ")}]`);
    await sleep(hold);
  }
  for (let i = 1; i <= 6; i++) await robot.goto(t.x, t.y, t.z + (70 * i) / 6, t.yaw, [p], 350);
  await robot.home();
  await robot.pose(requirePose(load(), "observe_small"), 1800);
} finally {
  await arm.close();
}
