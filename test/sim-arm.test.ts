import assert from "node:assert/strict";
import { describe, it } from "node:test";
import { DofbotArm } from "../src/arm/arm.js";
import { SimBoard, openSimArm } from "../src/arm/sim-board.js";

// Dus-întorsul unghi -> puls -> unghi trunchiază (ca int() în Arm_Lib): eroare de cel mult 1°.
function near(actual: number | null, expected: number): void {
  assert.ok(actual !== null && Math.abs(actual - expected) <= 1, `citit ${actual}, așteptat ${expected}±1`);
}

describe("braț simulat (DofbotArm peste SimBoard)", () => {
  it("pornește din 90° pe toate servo-urile", async () => {
    const { arm } = openSimArm();
    (await arm.readAll()).forEach((a) => near(a, 90));
  });

  it("citește înapoi ce a scris, cu rotunjirile protocolului", async () => {
    const { arm, board } = openSimArm();
    await arm.writeAll([0, 45, 135, 180, 270, 30], 800);
    assert.equal(board.lastTimeMs, 800);
    const read = await arm.readAll();
    [0, 45, 135, 180, 270, 30].forEach((a, i) => near(read[i] ?? null, a));
  });

  it("scrie un singur servo", async () => {
    const { arm } = openSimArm();
    await arm.writeServo(4, 20, 300);
    near(await arm.readServo(4), 20);
    near(await arm.readServo(1), 90);
  });

  it("cuplu oprit: brațul poate fi mișcat cu mâna și citit", async () => {
    const { arm, board } = openSimArm();
    assert.throws(() => board.moveByHand(2, 60));
    await arm.setTorque(false);
    assert.equal(board.torque, false);
    board.moveByHand(2, 60);
    near(await arm.readServo(2), 60);
    await arm.setTorque(true);
    assert.equal(board.torque, true);
  });

  it("buzzer", async () => {
    const { arm, board } = openSimArm();
    await arm.buzzerOn(5);
    assert.equal(board.buzzer, 5);
    await arm.buzzerOff();
    assert.equal(board.buzzer, 0);
  });

  it("citirile concurente nu se intercalează", async () => {
    const { arm } = openSimArm();
    await arm.writeAll([10, 20, 30, 40, 50, 60], 100);
    const read = await Promise.all([1, 2, 3, 4, 5, 6].map((id) => arm.readServo(id as 1)));
    assert.deepEqual(read, await arm.readAll());
  });

  it("aruncă eroare la adresă greșită", async () => {
    const arm = new DofbotArm(new SimBoard(), 0x16);
    await assert.rejects(arm.readServo(1), /niciun dispozitiv/);
  });
});
