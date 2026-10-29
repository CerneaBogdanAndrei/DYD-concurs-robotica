import assert from "node:assert/strict";
import type { AddressInfo } from "node:net";
import { after, before, describe, it } from "node:test";
import { openSimArm } from "../src/arm/sim-board.js";
import { loadCalib } from "../src/calib.js";
import { Robot } from "../src/robot.js";
import { createArmServer } from "../src/server.js";
import { ArmService } from "../src/service.js";

describe("server HTTP (contractul din CONTRACT_BRAT.md)", () => {
  let base = "";
  let close: () => void = () => {};
  let svc: ArmService;
  const logs: string[] = [];

  before(async () => {
    const robot = await Robot.create(openSimArm().arm, { sim: true, log: () => {} });
    const calib = loadCalib("sim/calib.json");
    svc = new ArmService(robot, () => calib, true);
    const server = createArmServer(svc, (m) => logs.push(m));
    await new Promise<void>((r) => server.listen(0, "127.0.0.1", r));
    base = `http://127.0.0.1:${(server.address() as AddressInfo).port}`;
    close = () => server.close();
  });
  after(() => close());

  const post = async (path: string, body?: unknown) => {
    const res = await fetch(base + path, {
      method: "POST",
      headers: { "content-type": "application/json" },
      body: body === undefined ? "" : JSON.stringify(body),
    });
    return { status: res.status, json: (await res.json()) as Record<string, unknown> };
  };

  it("status", async () => {
    const res = await fetch(`${base}/status`);
    const { result } = (await res.json()) as { result: { sim: boolean; sites: Record<string, string> } };
    assert.equal(result.sim, true);
    assert.equal(result.sites.source, "grid");
    assert.equal(result.sites.zone_red, "point");
  });

  it("pick și place întorc result: true", async () => {
    assert.deepEqual(await post("/pick", { location: "source", cell: [0, 1], yaw_deg: 12 }), { status: 200, json: { result: true } });
    assert.deepEqual(await post("/place", { location: "dest", cell: [0, 1], level: 1 }), { status: 200, json: { result: true } });
    assert.ok(logs.some((l) => /^pick source \(0,1\) -> OK/.test(l)));
  });

  it("read_joints, home, go_pose, beep, gheara", async () => {
    for (const [path, body] of [
      ["/home", undefined], ["/go_pose", { name: "observe_dest", speed: "slow" }], ["/open_gripper", undefined],
      ["/close_gripper", undefined], ["/beep", { n: 2 }], ["/move_joints", { angles: [90, 90, 90, 90, 90], time_ms: 500 }],
    ] as const) {
      assert.deepEqual(await post(path, body), { status: 200, json: { result: null } }, path);
    }
    const { json } = await post("/read_joints");
    assert.equal((json.result as unknown[]).length, 6);
  });

  it("erori: argument 400, calibrare 409, rută 404, JSON invalid 400", async () => {
    const e1 = await post("/pick", { location: "source", cell: [5, 5] });
    assert.equal(e1.status, 400);
    assert.equal((e1.json.error as { code: string }).code, "argument");
    const e2 = await post("/go_pose", { name: "nu_exista" });
    assert.equal(e2.status, 409);
    assert.equal((e2.json.error as { code: string }).code, "calibrare");
    assert.equal((await post("/zboara")).status, 404);
    const bad = await fetch(`${base}/pick`, { method: "POST", body: "{nu e json" });
    assert.equal(bad.status, 400);
    const lim = await post("/move_joints", { angles: [90, 90, 90, 90, 90, 175], time_ms: 100 });
    assert.equal((lim.json.error as { code: string }).code, "limite");
  });

  it("cererile simultane nu se suprapun niciodată", async () => {
    let active = 0;
    let maxActive = 0;
    const orig = svc.pick.bind(svc);
    svc.pick = async (args) => {
      maxActive = Math.max(maxActive, ++active);
      try {
        return await orig(args);
      } finally {
        active--;
      }
    };
    const results = await Promise.all(
      [[0, 0], [0, 2], [2, 0], [2, 2], [1, 1]].map((cell) => post("/pick", { location: "source", cell })),
    );
    svc.pick = orig;
    assert.ok(results.every((r) => r.json.result === true));
    assert.equal(maxActive, 1);
  });
});
