import assert from "node:assert/strict";
import { describe, it } from "node:test";
import { openSimArm } from "../src/arm/sim-board.js";
import * as C from "../src/config.js";
import { fk, Unreachable } from "../src/kinematics.js";
import { JointLimitError, Robot, roundHalfEven } from "../src/robot.js";

/** Robot simulat care ține minte fiecare comandă trimisă. */
async function recorded() {
  const { arm } = openSimArm();
  const events: ({ kind: "move"; s: number[]; t: number } | { kind: "grip"; v: number })[] = [];
  const origAll = arm.writeAll.bind(arm);
  arm.writeAll = async (a, t) => {
    events.push({ kind: "move", s: [...a], t });
    return origAll(a, t);
  };
  const origOne = arm.writeServo.bind(arm);
  arm.writeServo = async (id, a, t) => {
    events.push({ kind: "grip", v: a });
    return origOne(id, a, t);
  };
  const robot = await Robot.create(arm, { sim: true, log: () => {} });
  return { robot, events };
}

// Unghiurile se rotunjesc la grade întregi (int(round()) din referință): ~1° pe L4 = 175 mm ≈ 3 mm.
const TOL = 5;

const moves = (ev: Awaited<ReturnType<typeof recorded>>["events"]) =>
  ev.flatMap((e) => (e.kind === "move" ? [{ ...fk(e.s), s: e.s, t: e.t }] : []));

describe("Robot (mișcări portate din dofbot.py)", () => {
  it("round() ca în Python", () => {
    assert.deepEqual([0.5, 1.5, 2.5, 3.5, 2.4, 2.6, 89.5].map(roundHalfEven), [0, 2, 2, 4, 2, 3, 90]);
  });

  it("pick: apropiere sus, coborâre în VERTICAL_STEPS pași, strânge, ridică", async () => {
    const { robot, events } = await recorded();
    robot.grip = 0; // gheara nu e deschisă -> pick o deschide întâi
    await robot.pick(200, 40, 15, 10);
    assert.deepEqual(events[0], { kind: "grip", v: C.GRIP_OPEN });
    const closeAt = events.findIndex((e) => e.kind === "grip" && e.v === C.GRIP_CLOSED);
    assert.ok(closeAt > 0);
    const before = moves(events.slice(0, closeAt));
    const after = moves(events.slice(closeAt));
    // tranzit la max(HOVER, TRANSIT_MIN_Z) deasupra punctului de prindere
    const zh = 15 + C.HOVER_DZ;
    assert.ok(Math.abs(before[0]!.z - Math.max(zh, C.TRANSIT_MIN_Z)) < TOL);
    // coborârea finală are VERTICAL_STEPS pași pe aceeași verticală și se termină la z
    const descent = before.slice(-C.VERTICAL_STEPS);
    for (const p of descent) {
      assert.ok(Math.hypot(p.x - 200, p.y - 40) < TOL, `deviere laterală la z=${p.z}`);
      assert.equal(p.t, Math.trunc(C.T_FINE / C.VERTICAL_STEPS));
    }
    assert.ok(Math.abs(descent.at(-1)!.z - 15) < TOL);
    assert.equal(after.length, C.VERTICAL_STEPS);
    assert.ok(Math.abs(after.at(-1)!.z - zh) < TOL);
  });

  it("între două locuri nu se deplasează lateral sub TRANSIT_MIN_Z", async () => {
    const { robot, events } = await recorded();
    await robot.pick(200, 80, 15);
    await robot.place(180, -90, 15 + 2 * C.CUBE_MM);
    await robot.pick(240, -20, 15);
    const m = moves(events);
    for (let i = 1; i < m.length; i++) {
      const lateral = Math.hypot(m[i]!.x - m[i - 1]!.x, m[i]!.y - m[i - 1]!.y);
      if (lateral > 5) {
        assert.ok(m[i - 1]!.z >= C.TRANSIT_MIN_Z - TOL && m[i]!.z >= C.TRANSIT_MIN_Z - TOL,
          `deplasare laterală de ${lateral.toFixed(0)} mm la z=${m[i - 1]!.z.toFixed(0)}->${m[i]!.z.toFixed(0)}`);
      }
    }
  });

  it("place: eliberează la z + PLACE_GAP și urcă înapoi", async () => {
    const { robot, events } = await recorded();
    await robot.place(200, 0, 45);
    const openAt = events.findIndex((e) => e.kind === "grip" && e.v === C.GRIP_OPEN);
    const low = moves(events.slice(0, openAt)).at(-1)!;
    assert.ok(Math.abs(low.z - (45 + C.PLACE_GAP)) < TOL);
    assert.equal(robot.grip, C.GRIP_OPEN);
  });

  it("regula A: gheara se pre-închide la deschiderea de abordare înainte de orice deplasare", async () => {
    const { robot, events } = await recorded();
    await robot.place(180, -60, 15, 0, 120); // după o eliberare cu deschidere mică, brațul e sus, gheara la 120
    events.length = 0;
    await robot.pick(220, 40, 15, 0, 110);
    assert.deepEqual(events[0], { kind: "grip", v: 110 }, "prima comandă e gheara, înaintea oricărei mișcări");
    const closeAt = events.findIndex((e) => e.kind === "grip" && e.v === C.GRIP_CLOSED);
    assert.ok(events.slice(1, closeAt).every((e) => e.kind === "move"), "gheara nu se mai deschide pe drum");
    assert.ok(!events.some((e) => e.kind === "grip" && e.v === C.GRIP_OPEN), "nicio deschidere la GRIP_OPEN");
  });

  it("regula A: dacă brațul e jos, întâi urcă pe verticală, apoi schimbă gheara", async () => {
    const { robot, events } = await recorded();
    await robot.goto(200, 0, 20, 0); // jos, lângă cuburi
    robot.grip = C.GRIP_CLOSED;
    events.length = 0;
    await robot.pick(240, 40, 15, 0, 110);
    const gripAt = events.findIndex((e) => e.kind === "grip");
    const lift = moves(events.slice(0, gripAt));
    assert.ok(lift.length > 0 && lift.every((p) => Math.hypot(p.x - 200, p.y) < TOL), "urcarea e pe loc");
    assert.ok(Math.abs(lift.at(-1)!.z - C.TRANSIT_MIN_Z) < TOL);
  });

  it("regula C: eliberare cu deschidere mică, apoi urcare strict verticală până la tranzit", async () => {
    const { robot, events } = await recorded();
    await robot.place(200, 0, 15, 0, 120);
    const openAt = events.findIndex((e) => e.kind === "grip");
    assert.deepEqual(events[openAt], { kind: "grip", v: 120 });
    assert.equal(events.filter((e) => e.kind === "grip").length, 1, "gheara nu se redeschide larg");
    const lift = moves(events.slice(openAt));
    assert.equal(lift.length, C.VERTICAL_STEPS);
    for (const p of lift) assert.ok(Math.hypot(p.x - 200, p.y) < TOL, `deviere laterală la z=${p.z}`);
    assert.ok(lift.at(-1)!.z >= C.TRANSIT_MIN_Z - TOL);
    assert.equal(robot.grip, 120);
  });

  it("punct inaccesibil: Unreachable înainte de orice mișcare", async () => {
    const { robot, events } = await recorded();
    await assert.rejects(robot.pick(500, 0, 15), Unreachable);
    assert.equal(events.length, 0);
  });

  it("refuză comenzi în afara limitelor", async () => {
    const { robot } = await recorded();
    await assert.rejects(robot.write6([90, 90, 90, 90, 90, 170], 100), JointLimitError);
    await assert.rejects(robot.write6([90, 90, 90, 90, 90], 100), JointLimitError);
    await assert.rejects(robot.gripper(160), JointLimitError);
  });

  it("anunță evenimentele ghearei cu poziția curentă", async () => {
    const { arm } = openSimArm();
    const seen: [number, number | undefined][] = [];
    const robot = await Robot.create(arm, { sim: true, log: () => {}, onGripper: (v, at) => seen.push([v, at?.[2]]) });
    await robot.pick(200, 0, 15);
    assert.deepEqual(seen, [[C.GRIP_CLOSED, 15]]);
  });
});
