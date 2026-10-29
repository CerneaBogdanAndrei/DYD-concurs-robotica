import assert from "node:assert/strict";
import { describe, it } from "node:test";
import {
  SERVO_IDS,
  angleToPulse,
  decodeRead,
  encodeBuzzer,
  encodeTorque,
  encodeWrite6,
  encodeWriteServo,
  pulseToAngle,
  servoRange,
  swapWord,
} from "../src/arm/protocol.js";

describe("unghi -> puls", () => {
  it("servo normal: 0..180 -> 900..3100", () => {
    assert.equal(angleToPulse(1, 0), 900);
    assert.equal(angleToPulse(1, 90), 2000);
    assert.equal(angleToPulse(1, 180), 3100);
    assert.equal(angleToPulse(6, 45), 1450);
  });

  it("servo-urile 2/3/4 sunt inversate", () => {
    for (const id of [2, 3, 4] as const) {
      assert.equal(angleToPulse(id, 0), 3100);
      assert.equal(angleToPulse(id, 180), 900);
      assert.equal(angleToPulse(id, 90), 2000);
    }
  });

  it("servo 5: 0..270 -> 380..3700", () => {
    assert.equal(angleToPulse(5, 0), 380);
    assert.equal(angleToPulse(5, 270), 3700);
    assert.equal(angleToPulse(5, 90), 1486); // int(3320*90/270 + 380)
  });

  it("trunchiază ca int() din Python", () => {
    assert.equal(angleToPulse(1, 1), 912); // 912.22
    assert.equal(angleToPulse(1, 10.5), 1028); // 1028.33
  });

  it("respinge unghiuri din afara intervalului", () => {
    assert.throws(() => angleToPulse(1, 181), RangeError);
    assert.throws(() => angleToPulse(1, -1), RangeError);
    assert.throws(() => angleToPulse(5, 271), RangeError);
    assert.throws(() => angleToPulse(1, Number.NaN), RangeError);
    assert.doesNotThrow(() => angleToPulse(5, 200));
  });
});

describe("puls -> unghi", () => {
  it("0 și valorile în afara intervalului dau null", () => {
    assert.equal(pulseToAngle(1, 0), null);
    assert.equal(pulseToAngle(1, 3200), null);
    assert.equal(pulseToAngle(5, 3800), null);
  });

  it("dus-întors pe toate unghiurile întregi: diferență de cel mult 1°", () => {
    for (const id of SERVO_IDS) {
      for (let a = 0; a <= servoRange(id).maxAngle; a++) {
        const back = pulseToAngle(id, angleToPulse(id, a));
        assert.notEqual(back, null);
        assert.ok(Math.abs((back as number) - a) <= 1, `servo ${id}, ${a}° -> ${back}°`);
      }
    }
  });

  it("inversează octeții cuvântului citit", () => {
    assert.equal(swapWord(0xd007), 0x07d0);
    assert.equal(decodeRead(1, 0xd007), 90); // puls 2000
    assert.equal(decodeRead(2, swapWord(3100)), 0);
  });
});

describe("codarea comenzilor", () => {
  it("scriere 6 servo-uri: timp pe 0x1e, apoi 12 octeți pe 0x1d", () => {
    const ops = encodeWrite6([90, 90, 90, 90, 90, 90], 1000);
    assert.deepEqual(ops, [
      { kind: "block", reg: 0x1e, bytes: [0x03, 0xe8] },
      {
        kind: "block",
        reg: 0x1d,
        bytes: [0x07, 0xd0, 0x07, 0xd0, 0x07, 0xd0, 0x07, 0xd0, 0x05, 0xce, 0x07, 0xd0],
      },
    ]);
  });

  it("scriere un servo pe 0x10 + id", () => {
    assert.deepEqual(encodeWriteServo(3, 0, 500), [
      { kind: "block", reg: 0x13, bytes: [0x0c, 0x1c, 0x01, 0xf4] }, // 3100, 500
    ]);
  });

  it("respinge un timp invalid", () => {
    assert.throws(() => encodeWrite6([90, 90, 90, 90, 90, 90], 70000), RangeError);
    assert.throws(() => encodeWriteServo(1, 90, 1.5), RangeError);
  });

  it("cuplu și buzzer", () => {
    assert.deepEqual(encodeTorque(false), [{ kind: "byte", reg: 0x1a, value: 0 }]);
    assert.deepEqual(encodeTorque(true), [{ kind: "byte", reg: 0x1a, value: 1 }]);
    assert.deepEqual(encodeBuzzer(10), [{ kind: "byte", reg: 0x06, value: 10 }]);
    assert.deepEqual(encodeBuzzer(0xff), [{ kind: "byte", reg: 0x06, value: 0xff }]);
    assert.throws(() => encodeBuzzer(51), RangeError);
  });
});
