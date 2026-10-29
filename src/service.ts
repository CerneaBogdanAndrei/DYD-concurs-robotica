// ArmInterface-ul creierului (vezi docs/CONTRACT_BRAT.md), implementat peste Robot.
// Creierul decide CE se mută; aici se decide CUM: unde e celula în spațiul
// robotului (calibrare), IK, apropiere de sus, coborâre în pași.

import { type Calib, gridOf, gripServoForMm, gripTable, requirePose, requireSite } from "./calib.js";
import * as C from "./config.js";
import { Unreachable } from "./kinematics.js";
import { Robot } from "./robot.js";

export class ArgumentError extends Error {
  override name = "ArgumentError";
}

export type Speed = "fast" | "slow";
export type Cell = [number, number];

export interface PickArgs {
  location: string;
  cell?: Cell | null;
  yaw_deg?: number;
  /** nivelul din stivă de unde se ia cubul (0 = pe masă). Extensie față de ArmInterface. */
  level?: number;
  /** poziția exactă a cubului în celule (0..3), din viziune. Extensie față de ArmInterface. */
  uv?: [number, number] | null;
  /** deschiderea interioară de abordare, mm (regula A). Cere gheara calibrată în mm. Extensie. */
  grip_mm?: number | null;
}

export interface PlaceArgs {
  location: string;
  cell?: Cell | null;
  level?: number;
  yaw_deg?: number;
  /** unde se pune cubul, în celule (0..3): la stivuire, poziția cubului de dedesubt. Extensie față de ArmInterface. */
  uv?: [number, number] | null;
  /** deschiderea interioară la eliberare, mm (regula C). Cere gheara calibrată în mm. Extensie. */
  release_mm?: number | null;
}

export interface Outcome {
  ok: boolean;
  /** de ce nu s-a făcut mișcarea (doar când ok = false) */
  reason?: string;
}

interface Target {
  x: number;
  y: number;
  z: number;
  yaw: number;
}

const sleep = (ms: number) => new Promise<void>((resolve) => setTimeout(resolve, ms));

function checkCell(cell: unknown): Cell {
  const ok =
    Array.isArray(cell) &&
    cell.length === 2 &&
    cell.every((v) => Number.isInteger(v) && v >= 0 && v <= 2);
  if (!ok) throw new ArgumentError(`cell invalid: ${JSON.stringify(cell)} (trebuie [r, c] cu valori 0..2)`);
  return cell as Cell;
}

function checkLevel(level: unknown): number {
  if (!Number.isInteger(level) || (level as number) < 0 || (level as number) > 8) {
    throw new ArgumentError(`level invalid: ${JSON.stringify(level)} (trebuie întreg 0..8)`);
  }
  return level as number;
}

function checkFinite(name: string, v: unknown): number {
  if (typeof v !== "number" || !Number.isFinite(v)) {
    throw new ArgumentError(`${name} invalid: ${JSON.stringify(v)}`);
  }
  return v;
}

export class ArmService {
  constructor(
    readonly robot: Robot,
    /** citit la fiecare apel, ca recalibrarea să se vadă fără repornire */
    private readonly calib: () => Calib,
    private readonly sim = false,
  ) {}

  /** location + cell (+ level, uv) -> punct în spațiul robotului. */
  resolve(location: string, cell: unknown, level: number, yawDeg: number, uv?: unknown): Target {
    if (typeof location !== "string" || location === "") throw new ArgumentError("location lipsește");
    const site = requireSite(this.calib(), location);
    if (site.kind === "grid") {
      const g = gridOf(site);
      let xy;
      if (uv !== undefined && uv !== null) {
        const ok = Array.isArray(uv) && uv.length === 2 && uv.every((v) => typeof v === "number" && v >= 0 && v <= 3);
        if (!ok) throw new ArgumentError(`uv invalid: ${JSON.stringify(uv)} (trebuie [u, v] în 0..3)`);
        xy = g.uvToXy(uv[0] as number, uv[1] as number);
      } else {
        if (cell === undefined || cell === null) {
          throw new ArgumentError(`'${location}' e un caroiaj: cell e obligatoriu`);
        }
        const [r, c] = checkCell(cell);
        xy = g.cellXy(r, c);
      }
      return { x: xy[0], y: xy[1], z: g.z + level * C.CUBE_MM, yaw: g.yawRobot(yawDeg) };
    }
    if (cell !== undefined && cell !== null) {
      throw new ArgumentError(`'${location}' e o zonă (un singur punct): cell trebuie să lipsească`);
    }
    const [x, y, z] = site.xyz;
    return { x, y, z: z + C.GRASP_DZ + level * C.CUBE_MM, yaw: yawDeg };
  }

  async home(): Promise<void> {
    await this.robot.home();
  }

  async goPose(name: string, speed: Speed = "fast"): Promise<void> {
    if (typeof name !== "string" || name === "") throw new ArgumentError("name lipsește");
    if (speed !== "fast" && speed !== "slow") throw new ArgumentError(`speed invalid: ${speed}`);
    const t = speed === "slow" ? 2 * C.T_MOVE : C.T_MOVE;
    if (name === "home") {
      await this.robot.pose(C.HOME, speed === "slow" ? 2400 : 1200);
      return;
    }
    await this.robot.pose(requirePose(this.calib(), name), t);
  }

  /** 5 unghiuri = servo 1..5 (gheara rămâne cum e); 6 unghiuri = inclusiv gheara. */
  async moveJoints(angles: unknown, timeMs: unknown): Promise<void> {
    if (!Array.isArray(angles) || (angles.length !== 5 && angles.length !== 6)) {
      throw new ArgumentError("angles trebuie să aibă 5 sau 6 valori");
    }
    const a = angles.map((v, i) => checkFinite(`angles[${i}]`, v));
    const t = checkFinite("time_ms", timeMs);
    if (!Number.isInteger(t) || t < 0 || t > 10000) throw new ArgumentError("time_ms trebuie întreg 0..10000");
    if (a.length === 6) {
      await this.robot.write6(a, t);
      this.robot.grip = a[5] as number;
    } else {
      await this.robot.pose(a, t);
    }
    this.robot.curXyz = null;
  }

  async openGripper(): Promise<void> {
    await this.robot.gripper(C.GRIP_OPEN);
  }

  async closeGripper(): Promise<void> {
    await this.robot.gripper(C.GRIP_CLOSED);
  }

  /**
   * Unghiul ghearei pentru o deschidere în mm. Cu gheara calibrată în mm, deschiderea lipsă devine cea
   * implicită (cubul + 2 x marja); necalibrată, se folosește GRIP_OPEN, iar o deschidere cerută e refuzată.
   */
  private gripFor(name: string, mm: unknown, margin: number): number {
    const calib = this.calib();
    if (mm === undefined || mm === null) {
      return gripTable(calib).length > 0 ? gripServoForMm(calib, C.CUBE_MM + 2 * margin) : C.GRIP_OPEN;
    }
    const v = checkFinite(name, mm);
    if (v <= C.CUBE_MM || v > C.GRIP_MAX_OPEN_MM) {
      throw new ArgumentError(`${name} invalid: ${v} (trebuie între ${C.CUBE_MM} și ${C.GRIP_MAX_OPEN_MM} mm)`);
    }
    return gripServoForMm(calib, v);
  }

  async pick(args: PickArgs): Promise<Outcome> {
    const level = checkLevel(args.level ?? 0);
    const yaw = checkFinite("yaw_deg", args.yaw_deg ?? 0);
    const t = this.resolve(args.location, args.cell, level, yaw, args.uv);
    const grip = this.gripFor("grip_mm", args.grip_mm, C.APPROACH_MARGIN_MM);
    return this.attempt(() => this.robot.pick(t.x, t.y, t.z, t.yaw, grip));
  }

  async place(args: PlaceArgs): Promise<Outcome> {
    const level = checkLevel(args.level ?? 0);
    const yaw = checkFinite("yaw_deg", args.yaw_deg ?? 0);
    const t = this.resolve(args.location, args.cell, level, yaw, args.uv);
    const grip = this.gripFor("release_mm", args.release_mm, C.RELEASE_MARGIN_MM);
    return this.attempt(() => this.robot.place(t.x, t.y, t.z, t.yaw, grip));
  }

  /** Inaccesibil -> { ok: false } (creierul poate sări cubul); orice altă eroare se propagă. */
  private async attempt(fn: () => Promise<void>): Promise<Outcome> {
    try {
      await fn();
      return { ok: true };
    } catch (e) {
      if (e instanceof Unreachable) return { ok: false, reason: e.message };
      throw e;
    }
  }

  async readJoints(): Promise<(number | null)[]> {
    return this.robot.read();
  }

  /** n bipuri scurte (100 ms), la 250 ms unul de altul. */
  async beep(n: unknown = 1): Promise<void> {
    if (!Number.isInteger(n) || (n as number) < 1 || (n as number) > 10) {
      throw new ArgumentError("n trebuie întreg 1..10");
    }
    for (let i = 0; i < (n as number); i++) {
      await this.robot.beep(1);
      if (!this.sim) await sleep(250);
    }
  }

  status() {
    const calib = this.calib();
    const table = gripTable(calib);
    return {
      sim: this.sim,
      poses: ["home", ...Object.keys(calib.poses)],
      sites: Object.fromEntries(Object.entries(calib.sites).map(([k, s]) => [k, s.kind])),
      joints: this.robot.cur,
      grip: this.robot.grip,
      xyz: this.robot.curXyz,
      cube_mm: C.CUBE_MM,
      /** gheara se poate comanda în mm (grip_mm / release_mm); intervalul măsurat */
      grip_calibrated: table.length > 0,
      grip_range_mm: table.length > 0 ? [table[0]![1], table[table.length - 1]![1]] : null,
    };
  }
}
