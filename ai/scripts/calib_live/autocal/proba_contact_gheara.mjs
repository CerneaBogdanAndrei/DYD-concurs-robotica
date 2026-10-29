// Proba: gheara deschisă la maxim coboară pe celula r1c1 și strânge în pași de 5 grd; contactul = servo 6 rămâne în urmă (cubul de 40 mm: 95).
//   node ai/scripts/calib_live/autocal/proba_contact_gheara.mjs
// Gheara deschisă la maxim coboară în jurul cubului de pe r1c1, apoi strânge în pași; contactul = servo-ul rămâne în urmă.
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
const [x, y] = g.cellXy(1, 1);
const zLow = g.z, zHigh = g.z + 70;
const p = C.PITCH_CANDIDATES.find((p) => ik(x, y, zLow, [p]) && ik(x, y, zHigh, [p]));
const arm = await openI2cArm();
const robot = await Robot.create(arm, { log: () => {} });
const g6 = async () => (await arm.readAll())[5];
try {
  for (let i = 0; i < 2; i++) { await arm.buzzerOn(1); await sleep(300); }
  robot.grip = 0;
  await arm.writeServo(6, 0, 800); await sleep(1000);
  console.log("gheara la 0, citit", await g6());
  await robot.goto(x, y, zHigh, yaw, [p], 1500);
  for (let i = 1; i <= 6; i++) await robot.goto(x, y, zHigh + ((zLow - zHigh) * i) / 6, yaw, [p], 300);
  console.log(`jos pe r1c1: servo [${robot.cur.join(", ")}], citit [${(await arm.readAll()).join(", ")}]`);
  let contact = null;
  for (let a = 5; a <= 150; a += 5) {
    await arm.writeServo(6, a, 150); await sleep(400);
    const r = await g6();
    console.log(`  comandă ${a} -> citit ${r}`);
    if (r !== null && a - r > 6) { contact = r; break; }
  }
  console.log("CONTACT la", contact);
  await arm.writeServo(6, 0, 500); await sleep(700);
  for (let i = 1; i <= 6; i++) await robot.goto(x, y, zLow + ((zHigh - zLow) * i) / 6, yaw, [p], 300);
  await robot.home();
  await robot.pose(requirePose(calib, "observe_small"), 1800);
} finally {
  await arm.close();
}
