// Test de diagnostic: gheara deschisă exact unde ar fi pus un cub în celula (r, c) la nivelul dat; așteaptă Enter.
import { createInterface } from "node:readline/promises";
const R = new URL("../../../dist/src", import.meta.url).pathname;
const { openI2cArm } = await import(R + "/arm/i2c.js");
const { loadCalib, gridOf, requireSite } = await import(R + "/calib.js");
const C = await import(R + "/config.js");
const { Robot } = await import(R + "/robot.js");
const [r, c, level] = process.argv.slice(2).map(Number);
const rl = createInterface({ input: process.stdin, output: process.stdout });
const g = gridOf(requireSite(loadCalib(new URL("../../../calib.json", import.meta.url).pathname), "small"));
const [x, y] = g.cellXy(r, c);
const z = g.z + level * C.CUBE_MM;
const yaw = g.yawRobot(0);
const arm = await openI2cArm();
const robot = await Robot.create(arm, { log: () => {} });
try {
  await robot.gripper(C.GRIP_OPEN);
  await robot.goto(x, y, z + C.HOVER_DZ, yaw);
  await robot.goto(x, y, z, yaw, null, C.T_FINE * 2);
  console.log(`r${r}c${c} nivel ${level}: x=${x.toFixed(1)} y=${y.toFixed(1)} z=${z.toFixed(1)} servo=[${robot.cur.join(", ")}]`);
  await rl.question("Enter = sus ");
  await robot.goto(x, y, z + C.HOVER_DZ, yaw, null, C.T_FINE);
  await robot.home();
} finally {
  rl.close();
  await arm.close();
}
