// Pune cubul ținut în gheară pe celula (r, c) la nivelul dat, cu o corecție dv (celule, + = spre robot) pe direcția liniilor.
const R = new URL("../../../dist/src", import.meta.url).pathname;
const { openI2cArm } = await import(R + "/arm/i2c.js");
const { loadCalib, gridOf, requireSite } = await import(R + "/calib.js");
const C = await import(R + "/config.js");
const { Robot } = await import(R + "/robot.js");
const [r, c, level, dv] = process.argv.slice(2).map(Number);
const g = gridOf(requireSite(loadCalib(new URL("../../../calib.json", import.meta.url).pathname), "medium"));
const [x, y] = g.uvToXy(c + 0.5, r + 0.5 + dv);
const z = g.z + level * C.CUBE_MM;
const arm = await openI2cArm();
const robot = await Robot.create(arm, { log: () => {} });
robot.grip = C.GRIP_CLOSED; // cubul e în gheară: nicio mișcare nu are voie să o deschidă înainte de eliberare
try {
  await robot.place(x, y, z, g.yawRobot(0));
  console.log(`pus pe r${r}c${c} nivel ${level} (corecție ${dv} celule): x=${x.toFixed(1)} y=${y.toFixed(1)} z=${z.toFixed(1)}`);
} finally {
  await arm.close();
}
