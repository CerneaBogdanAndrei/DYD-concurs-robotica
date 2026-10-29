// Modelul unui caroiaj 3x3 în sistemul robotului, portat din referinta-python/grid.py.
//
// Diferență față de referință: celulele sunt indexate 0..2 (ca în ArmInterface-ul
// creierului), nu 1..3. Formulele sunt aceleași, doar decalate cu 1.
//
// Coordonate „grilă”: u = coloana continuă (0..3), v = linia continuă (0..3).
// Celula (r, c) (0..2) are centrul la u = c + 0.5, v = r + 0.5.

import * as C from "./config.js";

export type Vec2 = [number, number];
export type Vec3 = [number, number, number];

/** 'r0c2' -> [0, 2] */
export function parseCellLabel(label: string): [number, number] {
  const m = /^r([0-2])c([0-2])$/.exec(label.toLowerCase().replace(/\s+/g, ""));
  if (!m) throw new Error(`etichetă de celulă invalidă: '${label}' (ex. r0c0, r2c1)`);
  return [Number(m[1]), Number(m[2])];
}

export const cellLabel = (r: number, c: number) => `r${r}c${c}`;

/** Rezolvă un sistem 3x3 (eliminare Gauss cu pivotare); null dacă e singular. */
function solve3(M: number[][], b: number[]): number[] | null {
  const a = M.map((row, i) => [...row, b[i] as number]);
  for (let col = 0; col < 3; col++) {
    let piv = col;
    for (let i = col + 1; i < 3; i++) {
      if (Math.abs(a[i]![col]!) > Math.abs(a[piv]![col]!)) piv = i;
    }
    if (Math.abs(a[piv]![col]!) < 1e-9) return null;
    [a[col], a[piv]] = [a[piv]!, a[col]!];
    for (let i = 0; i < 3; i++) {
      if (i === col) continue;
      const f = a[i]![col]! / a[col]![col]!;
      for (let j = col; j < 4; j++) a[i]![j]! -= f * a[col]![j]!;
    }
  }
  return a.map((row, i) => row[3]! / row[i]!);
}

/** Cele mai mici pătrate pentru A·s = y, cu A pe 3 coloane (ecuațiile normale). */
function leastSquares3(A: number[][], y: number[]): number[] | null {
  const AtA = [0, 1, 2].map((i) => [0, 1, 2].map((j) => A.reduce((s, row) => s + row[i]! * row[j]!, 0)));
  const Aty = [0, 1, 2].map((i) => A.reduce((s, row, k) => s + row[i]! * y[k]!, 0));
  return solve3(AtA, Aty);
}

export interface Residual {
  label: string;
  /** distanța în plan între punctul învățat și modelul potrivit, mm */
  errMm: number;
  z: number;
}

export class Grid {
  constructor(
    /** centrul celulei r0c0 (x, y) */
    readonly O: Vec2,
    /** deplasarea de la o coloană la următoarea */
    readonly ecol: Vec2,
    /** deplasarea de la o linie la următoarea */
    readonly erow: Vec2,
    /** înălțimea de prindere a unui cub de pe masă */
    readonly z: number,
  ) {}

  /** pts: { r0c0: [x, y, z], ... } (minim 3 celule necoliniare). */
  static fromPoints(pts: Readonly<Record<string, Vec3>>): { grid: Grid; residuals: Residual[] } {
    const A: number[][] = [];
    const X: number[] = [];
    const Y: number[] = [];
    const Z: number[] = [];
    for (const [label, p] of Object.entries(pts)) {
      const [r, c] = parseCellLabel(label);
      A.push([1, c, r]);
      X.push(p[0]);
      Y.push(p[1]);
      Z.push(p[2]);
    }
    const sx = A.length >= 3 ? leastSquares3(A, X) : null;
    const sy = A.length >= 3 ? leastSquares3(A, Y) : null;
    if (!sx || !sy) {
      throw new Error("trebuie cel puțin 3 celule necoliniare (ex. r0c0, r0c2, r2c0)");
    }
    const zMean = Z.reduce((s, v) => s + v, 0) / Z.length;
    const grid = new Grid([sx[0]!, sy[0]!], [sx[1]!, sy[1]!], [sx[2]!, sy[2]!], zMean + C.GRASP_DZ);
    const residuals = Object.entries(pts).map(([label, p]) => {
      const [r, c] = parseCellLabel(label);
      const q = grid.cellXy(r, c);
      return { label, errMm: Math.hypot(q[0] - p[0], q[1] - p[1]), z: p[2] };
    });
    return { grid, residuals };
  }

  uvToXy(u: number, v: number): Vec2 {
    return [
      this.O[0] + (u - 0.5) * this.ecol[0] + (v - 0.5) * this.erow[0],
      this.O[1] + (u - 0.5) * this.ecol[1] + (v - 0.5) * this.erow[1],
    ];
  }

  cellXy(r: number, c: number): Vec2 {
    return this.uvToXy(c + 0.5, r + 0.5);
  }

  /** Unghi măsurat în imaginea rectificată (față de axa coloanelor) -> unghi robot. */
  yawRobot(yawImgDeg: number): number {
    const a = (yawImgDeg * Math.PI) / 180;
    const vx = Math.cos(a) * this.ecol[0] + Math.sin(a) * this.erow[0];
    const vy = Math.cos(a) * this.ecol[1] + Math.sin(a) * this.erow[1];
    return (Math.atan2(vy, vx) * 180) / Math.PI;
  }

  info(): string {
    const pc = Math.hypot(...this.ecol);
    const pr = Math.hypot(...this.erow);
    const cos = (this.ecol[0] * this.erow[0] + this.ecol[1] * this.erow[1]) / (pc * pr);
    const ang = (Math.acos(Math.max(-1, Math.min(1, cos))) * 180) / Math.PI;
    return (
      `pas coloane ${pc.toFixed(1)} mm, pas linii ${pr.toFixed(1)} mm, ` +
      `unghi între axe ${ang.toFixed(1)} grd, z prindere ${this.z.toFixed(1)} mm`
    );
  }
}
