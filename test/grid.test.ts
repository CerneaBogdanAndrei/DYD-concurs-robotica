import assert from "node:assert/strict";
import { describe, it } from "node:test";
import * as C from "../src/config.js";
import { Grid, cellLabel, parseCellLabel, type Vec3 } from "../src/grid.js";

const close = (a: number, b: number, eps = 1e-6) => assert.ok(Math.abs(a - b) <= eps, `${a} vs ${b}`);

/** Caroiaj de referință: r0c0 la (120, 90), pas 45 mm, rotit cu 20°. */
const ref = (() => {
  const a = (20 * Math.PI) / 180;
  return new Grid([120, 90], [45 * Math.cos(a), 45 * Math.sin(a)], [-45 * Math.sin(a), 45 * Math.cos(a)], 15);
})();

function taught(labels: string[], noise: (i: number) => [number, number] = () => [0, 0]) {
  const pts: Record<string, Vec3> = {};
  labels.forEach((lab, i) => {
    const [r, c] = parseCellLabel(lab);
    const [x, y] = ref.cellXy(r, c);
    const [nx, ny] = noise(i);
    pts[lab] = [x + nx, y + ny, 15];
  });
  return pts;
}

describe("caroiaj", () => {
  it("etichete de celulă 0..2", () => {
    assert.deepEqual(parseCellLabel("r0c2"), [0, 2]);
    assert.deepEqual(parseCellLabel(" R2 C1 "), [2, 1]);
    assert.equal(cellLabel(1, 2), "r1c2");
    assert.throws(() => parseCellLabel("r3c1"));
    assert.throws(() => parseCellLabel("r1"));
  });

  it("centrele celulelor și coordonatele u, v", () => {
    const [x, y] = ref.cellXy(0, 0);
    close(x, 120);
    close(y, 90);
    // u = c + 0.5, v = r + 0.5
    assert.deepEqual(ref.uvToXy(2.5, 1.5), ref.cellXy(1, 2));
  });

  it("potrivire exactă din cele 4 colțuri", () => {
    const { grid, residuals } = Grid.fromPoints(taught(["r0c0", "r0c2", "r2c0", "r2c2"]));
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        const [x, y] = grid.cellXy(r, c);
        const [ex, ey] = ref.cellXy(r, c);
        close(x, ex);
        close(y, ey);
      }
    }
    residuals.forEach((res) => close(res.errMm, 0));
    close(grid.z, 15 + C.GRASP_DZ);
    assert.match(grid.info(), /pas coloane 45\.0 mm, pas linii 45\.0 mm, unghi între axe 90\.0 grd/);
  });

  it("cele mai mici pătrate mediază zgomotul de învățare", () => {
    const noise = (i: number): [number, number] => [[2, -1], [-2, 1], [-1, -2], [1, 2], [0, 0]][i] as [number, number];
    const { grid, residuals } = Grid.fromPoints(taught(["r0c0", "r0c2", "r2c0", "r2c2", "r1c1"], noise));
    const [x, y] = grid.cellXy(1, 1);
    const [ex, ey] = ref.cellXy(1, 1);
    assert.ok(Math.hypot(x - ex, y - ey) < 1, "centrul trebuie să rămână sub 1 mm");
    assert.ok(residuals.every((r) => r.errMm < 3));
  });

  it("înălțimea de prindere urmează planul colțurilor, nu media lor", () => {
    const pts = taught(["r0c0", "r0c2", "r2c0", "r2c2"]);
    // ca pe brațul real (2026-10-29): același z fizic, dar modelul dă 9,7 / 4,3 / 14,1 / 8,7 mm
    pts["r0c0"]![2] = 9.7;
    pts["r0c2"]![2] = 4.3;
    pts["r2c0"]![2] = 14.1;
    pts["r2c2"]![2] = 8.7;
    const { grid } = Grid.fromPoints(pts);
    const dz = C.GRASP_DZ;
    close(grid.zAt(0.5, 0.5), 9.7 + dz);
    close(grid.zAt(2.5, 0.5), 4.3 + dz);
    close(grid.zAt(0.5, 2.5), 14.1 + dz);
    close(grid.zAt(2.5, 2.5), 8.7 + dz);
    close(grid.z, (9.7 + 4.3 + 14.1 + 8.7) / 4 + dz);
  });

  it("corecția pe linii se adaugă planului, liniar între centrele liniilor", () => {
    const pts = taught(["r0c0", "r0c2", "r2c0", "r2c2"]);
    const plan = Grid.fromPoints(pts).grid;
    const { grid } = Grid.fromPoints(pts, { zRows: [0, 0, -14] });
    close(grid.zAt(0.5, 0.5), plan.zAt(0.5, 0.5));
    close(grid.zAt(1.5, 1.5), plan.zAt(1.5, 1.5));
    close(grid.zAt(1.5, 2.0), plan.zAt(1.5, 2.0) - 7);
    close(grid.zAt(2.5, 2.5), plan.zAt(2.5, 2.5) - 14);
    close(grid.zAt(0.5, 2.9), plan.zAt(0.5, 2.9) - 14);
    close(grid.zAt(0.5, 0.1), plan.zAt(0.5, 0.1));
  });

  it("corecția spre r0 mută punctul pe direcția liniilor, departe de robot", () => {
    const pts = taught(["r0c0", "r0c2", "r2c0", "r2c2"]);
    const plan = Grid.fromPoints(pts).grid;
    const { grid, residuals } = Grid.fromPoints(pts, { fwdRows: [0, 0, 15] });
    const d = (u: number, v: number) => {
      const [a, b] = [grid.uvToXy(u, v), plan.uvToXy(u, v)];
      return [a[0] - b[0], a[1] - b[1]] as const;
    };
    close(Math.hypot(...d(1.5, 1.5)), 0);
    const [dx, dy] = d(2.5, 2.5);
    close(Math.hypot(dx, dy), 15);
    // spre r0 = invers lui erow (de la r0 spre r2)
    close((dx * grid.erow[0] + dy * grid.erow[1]) / Math.hypot(...grid.erow), -15);
    close(Math.hypot(...d(0.5, 2.0)), 7.5);
    assert.ok(residuals.every((r) => r.errMm < 3), "eroarea de potrivire nu include corecția");
  });

  it("respinge celule coliniare sau prea puține", () => {
    assert.throws(() => Grid.fromPoints(taught(["r0c0", "r0c1", "r0c2"])), /necoliniare/);
    assert.throws(() => Grid.fromPoints(taught(["r0c0", "r2c2"])), /necoliniare/);
  });

  it("yaw imagine -> yaw robot urmează rotația caroiajului", () => {
    close(ref.yawRobot(0), 20);
    close(ref.yawRobot(90), 110);
  });
});
