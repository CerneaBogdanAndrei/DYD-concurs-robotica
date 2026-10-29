import assert from "node:assert/strict";
import { describe, it } from "node:test";
import { openSimArm } from "../src/arm/sim-board.js";
import { CalibError, loadCalib } from "../src/calib.js";
import * as C from "../src/config.js";
import { fk } from "../src/kinematics.js";
import { Robot } from "../src/robot.js";
import { ArgumentError, ArmService } from "../src/service.js";

const SIM_CALIB = loadCalib("sim/calib.json");

async function service() {
  const { arm, board } = openSimArm();
  const robot = await Robot.create(arm, { sim: true, log: () => {} });
  return { svc: new ArmService(robot, () => SIM_CALIB, true), robot, board };
}

describe("ArmService (ArmInterface peste Robot)", () => {
  it("calibrarea de simulare: fiecare celulă și zonă e accesibilă pe 4 niveluri", async () => {
    const { svc } = await service();
    for (const location of ["source", "dest"]) {
      for (let r = 0; r < 3; r++) {
        for (let c = 0; c < 3; c++) {
          for (let level = 0; level < 4; level++) {
            assert.deepEqual(await svc.place({ location, cell: [r, c], level }), { ok: true }, `${location} ${r},${c} L${level}`);
            assert.deepEqual(await svc.pick({ location, cell: [r, c], level }), { ok: true });
          }
        }
      }
    }
    for (const zone of ["zone_red", "zone_yellow", "zone_blue", "zone_green"]) {
      for (let level = 0; level < 4; level++) {
        assert.deepEqual(await svc.place({ location: zone, level }), { ok: true }, `${zone} L${level}`);
      }
    }
  });

  it("pick pe celulă coboară la centrul ei, la z-ul caroiajului", async () => {
    const { svc, robot } = await service();
    const t = svc.resolve("source", [1, 1], 0, 0);
    assert.ok(Math.abs(t.x - 180) < 1e-9 && Math.abs(t.y - 100) < 1e-9 && Math.abs(t.z - 15) < 1e-9);
    const seen: number[][] = [];
    const orig = robot.dev.writeAll.bind(robot.dev);
    robot.dev.writeAll = async (a, ms) => {
      seen.push([...a]);
      return orig(a, ms);
    };
    await svc.pick({ location: "source", cell: [1, 1] });
    const low = seen.reduce((best, s) => (fk(s).z < fk(best).z ? s : best));
    const p = fk(low);
    assert.ok(Math.hypot(p.x - 180, p.y - 100) < 5 && Math.abs(p.z - 15) < 5, JSON.stringify(p));
    assert.equal(robot.grip, C.GRIP_CLOSED);
  });

  it("level urcă cu latura cubului; uv precizează poziția în celulă", async () => {
    const { svc } = await service();
    assert.equal(svc.resolve("source", [0, 0], 2, 0).z, 15 + 2 * C.CUBE_MM);
    const a = svc.resolve("source", [0, 0], 0, 0, [0.5, 0.5]);
    const b = svc.resolve("source", null, 0, 0, [0.75, 0.5]);
    assert.ok(Math.abs(a.y - 60) < 1e-9 && Math.abs(b.y - 70) < 1e-9); // un sfert de celulă = 10 mm
  });

  it("zona are un singur punct; yaw e în sistemul robotului", async () => {
    const { svc } = await service();
    assert.deepEqual(svc.resolve("zone_red", null, 1, 30), { x: 265, y: 0, z: 15 + C.CUBE_MM, yaw: 30 });
    assert.throws(() => svc.resolve("zone_red", [0, 0], 0, 0), ArgumentError);
  });

  it("punct inaccesibil -> { ok: false, reason }", async () => {
    const { robot } = await service();
    const far = { poses: {}, sites: { departe: { kind: "point" as const, xyz: [480, 0, 15] as [number, number, number] } } };
    const svc = new ArmService(robot, () => far, true);
    const out = await svc.pick({ location: "departe" });
    assert.equal(out.ok, false);
    assert.match(out.reason ?? "", /inaccesibil/);
  });

  it("argumente greșite și calibrare lipsă", async () => {
    const { svc } = await service();
    await assert.rejects(svc.pick({ location: "source" }), ArgumentError); // lipsește cell
    await assert.rejects(svc.pick({ location: "source", cell: [3, 0] }), ArgumentError); // 0..2
    await assert.rejects(svc.pick({ location: "source", cell: [1, 1], level: -1 }), ArgumentError);
    await assert.rejects(svc.pick({ location: "zone_mov" }), CalibError);
    await assert.rejects(svc.goPose("observe_mov"), CalibError);
    await assert.rejects(svc.moveJoints([90, 90], 500), ArgumentError);
    await assert.rejects(svc.beep(0), ArgumentError);
  });

  it("move_joints cu 5 sau 6 unghiuri, go_pose din calibrare", async () => {
    const { svc, board, robot } = await service();
    await svc.moveJoints([100, 80, 70, 60, 90], 300);
    assert.equal(board.lastTimeMs, 300);
    await svc.moveJoints([100, 80, 70, 60, 90, 120], 300);
    assert.equal(robot.grip, 120);
    await svc.goPose("observe_source");
    assert.deepEqual(robot.cur.slice(0, 5), SIM_CALIB.poses.observe_source);
    await svc.goPose("home", "slow");
    assert.deepEqual(robot.cur.slice(0, 5), C.HOME);
    assert.deepEqual(svc.status().poses, ["home", "observe_source", "observe_dest"]);
  });

  it("beep(n) sună de n ori", async () => {
    const { svc, robot } = await service();
    let count = 0;
    robot.dev.buzzerOn = async () => {
      count++;
    };
    await svc.beep(3);
    assert.equal(count, 3);
  });
});
