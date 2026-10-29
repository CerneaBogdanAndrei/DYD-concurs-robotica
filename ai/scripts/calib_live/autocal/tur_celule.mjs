// Proba calibrării: gheara deschisă deasupra celulelor, la DZ mm peste înălțimea de prindere (nu atinge masa).
//   node ai/scripts/calib_live/autocal/tur_celule.mjs [DZ=20] [ms pe celulă=5000] [celule=11,00,02,22,20] [dosar pentru poze]
import { writeFileSync } from "node:fs";
const ROOT = new URL("../../../../", import.meta.url).pathname;
const R = ROOT + "dist/src";
const { openI2cArm } = await import(R + "/arm/i2c.js");
const { loadCalib, gridOf, requireSite, requirePose } = await import(R + "/calib.js");
const C = await import(R + "/config.js");
const { ik } = await import(R + "/kinematics.js");
const { Robot } = await import(R + "/robot.js");
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const calib = loadCalib(ROOT + "calib.json");
const g = gridOf(requireSite(calib, "small"));
const yaw = g.yawRobot(0);
const DZ = Number(process.argv[2] ?? 20), HOLD = Number(process.argv[3] ?? 5000);
const cells = (process.argv[4] ?? "11,00,02,22,20").split(",").map((s) => [Number(s[0]), Number(s[1])]);
const arm = await openI2cArm();
const robot = await Robot.create(arm, { log: () => {} });
try {
  await robot.gripper(C.GRIP_OPEN);
  for (const [r, c] of cells) {
    const [x, y] = g.cellXy(r, c);
    const zLow = g.z + DZ, zHigh = g.z + 70;
    const p = C.PITCH_CANDIDATES.find((p) => ik(x, y, zLow, [p]) && ik(x, y, zHigh, [p]));
    await robot.goto(x, y, zHigh, yaw, [p]);
    await robot.goto(x, y, zLow, yaw, [p], 1500);
    await arm.buzzerOn(1);
    console.log(`r${r}c${c}: gheara la ${DZ} mm peste prindere, x=${x.toFixed(1)} y=${y.toFixed(1)} z=${zLow.toFixed(1)} înclinare ${p}, servo=[${robot.cur.join(", ")}], citit=[${(await arm.readAll()).join(", ")}]`);
    await sleep(HOLD - 1000);
    try { writeFileSync(`${process.argv[5] ?? "."}/hover_r${r}c${c}.jpg`, Buffer.from(await (await fetch("http://127.0.0.1:8080/snapshot.jpg")).arrayBuffer())); } catch {}
    await sleep(1000);
    await robot.goto(x, y, zHigh, yaw, [p], 1000);
  }
  await robot.pose(requirePose(calib, "observe_small"), 1500);
} finally {
  await arm.close();
}
