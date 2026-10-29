import assert from "node:assert/strict";
import { describe, it } from "node:test";
import { PITCH_CANDIDATES } from "../src/config.js";
import { fk, ik, ikPlanar, wristForYaw } from "../src/kinematics.js";

const close = (a: number, b: number, eps: number, what: string) =>
  assert.ok(Math.abs(a - b) <= eps, `${what}: ${a} vs ${b}`);

describe("cinematică", () => {
  it("PITCH_CANDIDATES = -90, -85, ..., 0", () => {
    assert.equal(PITCH_CANDIDATES.length, 19);
    assert.equal(PITCH_CANDIDATES[0], -90);
    assert.equal(PITCH_CANDIDATES.at(-1), 0);
  });

  it("HOME (toate 90) = braț drept în sus", () => {
    const p = fk([90, 90, 90, 90]);
    close(p.x, 0, 1e-9, "x");
    close(p.y, 0, 1e-9, "y");
    close(p.z, 105 + 83 + 83 + 175, 1e-9, "z");
    close(p.pitch, 90, 1e-9, "pitch");
  });

  it("servo 1 rotește în jurul axei z (x spre față la 90°)", () => {
    const p0 = fk([90, 45, 90, 45]);
    const p1 = fk([120, 45, 90, 45]);
    close(Math.hypot(p0.x, p0.y), Math.hypot(p1.x, p1.y), 1e-9, "rază");
    close(Math.atan2(p1.y, p1.x), Math.PI / 6, 1e-9, "unghi bază");
    close(p0.y, 0, 1e-9, "y la 90°");
  });

  it("IK -> FK dus-întors pe zona de lucru (14..28 cm, mai multe înălțimi)", () => {
    let tested = 0;
    for (let r = 140; r <= 280; r += 10) {
      for (let th = -60; th <= 60; th += 15) {
        for (const z of [15, 45, 60, 90, 150]) {
          const x = r * Math.cos((th * Math.PI) / 180);
          const y = r * Math.sin((th * Math.PI) / 180);
          const sol = ik(x, y, z);
          if (!sol) continue;
          const p = fk(sol.servos);
          close(p.x, x, 1e-6, "x");
          close(p.y, y, 1e-6, "y");
          close(p.z, z, 1e-6, "z");
          close(p.pitch, sol.pitch, 1e-6, "pitch");
          tested++;
        }
      }
    }
    assert.ok(tested > 400, `prea puține puncte accesibile: ${tested}`);
  });

  it("FK -> IK dus-întors (cot „deasupra”, adică s3 <= 90)", () => {
    for (const s1 of [30, 90, 150]) {
      for (const s2 of [30, 60, 90, 120]) {
        // s3 = 90 exact (braț întins) e la limita acos(1) și poate ieși din domeniu numeric
        for (const s3 of [10, 40, 70, 85]) {
          // s4 ales astfel încât înclinarea să fie unul din candidați (-90..0, pas 5)
          for (const pitch of [-90, -60, -30, 0]) {
            const s4 = pitch + 180 - s2 - s3;
            if (s4 < 0 || s4 > 180) continue;
            const p = fk([s1, s2, s3, s4]);
            // punctele din spatele axei bazei (rază planară <= 0) au altă soluție pentru s1
            const th = ((s1 - 90) * Math.PI) / 180;
            if (p.x * Math.cos(th) + p.y * Math.sin(th) <= 1) continue;
            const sol = ik(p.x, p.y, p.z, [pitch]);
            assert.ok(sol, `fără soluție pentru ${[s1, s2, s3, s4]}`);
            [s1, s2, s3, s4].forEach((v, i) => close(sol.servos[i] as number, v, 1e-6, `s${i + 1}`));
          }
        }
      }
    }
  });

  it("alege cea mai verticală înclinare posibilă", () => {
    for (const [x, z] of [[140, 15], [200, 15], [200, 60], [260, 150]] as const) {
      const sol = ik(x, 0, z);
      assert.ok(sol);
      for (const p of PITCH_CANDIDATES.filter((p) => p < sol.pitch)) {
        assert.equal(ik(x, 0, z, [p]), null, `${p} ar fi fost mai vertical la x=${x} z=${z}`);
      }
    }
  });

  it("puncte inaccesibile -> null", () => {
    assert.equal(ik(600, 0, 15), null); // prea departe
    assert.equal(ik(-200, -5, 15), null); // în spatele bazei (s1 < 0)
    assert.equal(ikPlanar(0, 900, -90), null);
  });

  it("wristForYaw: simetrie la 90° și centrul 90", () => {
    assert.equal(wristForYaw(90, 0), 90);
    for (const yaw of [-170, -100, -30, 0, 10, 44, 80, 135]) {
      const a = wristForYaw(90, yaw);
      const b = wristForYaw(90, yaw + 90);
      close(a, b, 1e-9, `yaw ${yaw}`);
      assert.ok(a >= 45 && a <= 135);
    }
    // baza rotită cu 30° compensează: un cub aliniat cu raza nu cere rotirea încheieturii
    assert.equal(wristForYaw(120, 30), 90);
  });
});
