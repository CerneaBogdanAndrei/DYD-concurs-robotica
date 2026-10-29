// Cinematica directă/inversă pentru Dofbot, portată din referinta-python/dofbot.py.
//
// Sistemul de coordonate al robotului: originea pe axa bazei, la nivelul mesei,
// x spre față (servo 1 = 90), z în sus. Toate valorile în mm / grade.

import * as C from "./config.js";

export class Unreachable extends Error {
  override name = "Unreachable";
}

const rad = (d: number) => (d * Math.PI) / 180;
const deg = (r: number) => (r * 180) / Math.PI;

export interface Pose {
  x: number;
  y: number;
  z: number;
  /** unghiul ghearei față de orizontală (-90 = în jos) */
  pitch: number;
}

/** s = [s1, s2, s3, s4, ...] (grade servo) -> poziția punctului de prindere. */
export function fk(s: readonly number[]): Pose {
  const [s1, s2, s3, s4] = s as [number, number, number, number];
  const th = rad(C.BASE_SIGN * (s1 - 90));
  const a2 = rad(s2);
  const a3 = rad(s2 + s3 - 90);
  const a4 = rad(s2 + s3 + s4 - 180);
  const r = C.L2 * Math.cos(a2) + C.L3 * Math.cos(a3) + C.L4 * Math.cos(a4);
  const z = C.L1 + C.L2 * Math.sin(a2) + C.L3 * Math.sin(a3) + C.L4 * Math.sin(a4);
  return { x: r * Math.cos(th), y: r * Math.sin(th), z, pitch: deg(a4) };
}

/** IK în planul brațului. pitch = unghiul ghearei față de orizontală (-90 = în jos). */
export function ikPlanar(r: number, z: number, pitchDeg: number): [number, number, number] | null {
  const a4 = rad(pitchDeg);
  const wr = r - C.L4 * Math.cos(a4);
  const wz = z - C.L4 * Math.sin(a4) - C.L1;
  const c = (wr * wr + wz * wz - C.L2 ** 2 - C.L3 ** 2) / (2 * C.L2 * C.L3);
  if (c < -1.0 || c > 1.0) return null;
  const e = -Math.acos(c); // cotul „deasupra” liniei umăr-încheietură
  const a2 = Math.atan2(wz, wr) - Math.atan2(C.L3 * Math.sin(e), C.L2 + C.L3 * Math.cos(e));
  const a3 = a2 + e;
  const s2 = deg(a2);
  const s3 = deg(e) + 90;
  const s4 = pitchDeg - deg(a3) + 90;
  for (const v of [s2, s3, s4]) {
    if (!(v >= 0.0 && v <= 180.0)) return null;
  }
  return [s2, s3, s4];
}

/** -> { servos: [s1, s2, s3, s4], pitch } sau null dacă punctul e inaccesibil. */
export function ik(
  x: number,
  y: number,
  z: number,
  pitches?: readonly number[] | null,
): { servos: [number, number, number, number]; pitch: number } | null {
  const th = deg(Math.atan2(y, x));
  const s1 = 90 + C.BASE_SIGN * th;
  if (!(s1 >= 0.0 && s1 <= 180.0)) return null;
  const r = Math.hypot(x, y);
  // `pitches or ...` din Python: și o listă goală înseamnă candidații impliciți.
  const list = pitches && pitches.length > 0 ? pitches : C.PITCH_CANDIDATES;
  for (const p of list) {
    const sol = ikPlanar(r, z, p);
    if (sol) return { servos: [s1, ...sol], pitch: p };
  }
  return null;
}

/**
 * Unghiul servo 5 astfel încât degetele să fie aliniate cu fețele unui cub
 * rotit cu yawDeg (în sistemul robotului). Cubul e simetric la 90 de grade.
 */
export function wristForYaw(s1: number, yawDeg: number): number {
  const th = C.BASE_SIGN * (s1 - 90);
  let s5 = C.J5_CENTER + C.WRIST_SIGN * (yawDeg - th);
  while (s5 - C.J5_CENTER > 45) s5 -= 90;
  while (s5 - C.J5_CENTER < -45) s5 += 90;
  return Math.max(C.J5_MIN, Math.min(C.J5_MAX, s5));
}
