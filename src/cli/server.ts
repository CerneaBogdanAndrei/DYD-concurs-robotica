// Pornește serviciul de mișcare.
//
//   npm run server                      # braț real (Raspberry Pi), calib.json
//   npm run sim                         # placă simulată, sim/calib.json (merge și pe Windows)
//   node dist/src/cli/server.js --port 8765 --host 127.0.0.1 --calib calib.json [--sim]

import { parseArgs } from "node:util";
import type { DofbotArm } from "../arm/arm.js";
import { openI2cArm } from "../arm/i2c.js";
import { openSimArm } from "../arm/sim-board.js";
import { calibLoader } from "../calib.js";
import * as C from "../config.js";
import { log } from "../log.js";
import { Robot } from "../robot.js";
import { createArmServer } from "../server.js";
import { ArmService } from "../service.js";

const { values } = parseArgs({
  options: {
    sim: { type: "boolean", default: false },
    port: { type: "string", default: "8765" },
    host: { type: "string", default: "127.0.0.1" },
    calib: { type: "string" },
    verbose: { type: "boolean", default: false },
  },
});

const sim = values.sim;
const calibPath = values.calib ?? (sim ? "sim/calib.json" : C.CALIB_FILE);
const port = Number(values.port);

let arm: DofbotArm;
if (sim) {
  arm = openSimArm().arm;
} else {
  arm = await openI2cArm();
}
const robot = await Robot.create(arm, { sim, log: values.verbose ? log : () => {} });
const calib = calibLoader(calibPath);
const svc = new ArmService(robot, calib, sim);
const server = createArmServer(svc, log);

server.listen(port, values.host, () => {
  const st = svc.status();
  log(`Serviciu braț ${sim ? "SIMULAT" : "REAL (I2C)"} pe http://${values.host}:${port}`);
  log(`calibrare: ${calibPath} | poze: ${st.poses.join(", ")} | locuri: ${Object.keys(st.sites).join(", ") || "niciunul"}`);
  if (!sim) void robot.beep(1);
});

async function shutdown() {
  log("Oprire...");
  server.close();
  try {
    await arm.close();
  } finally {
    process.exit(0);
  }
}
process.on("SIGINT", shutdown);
process.on("SIGTERM", shutdown);
