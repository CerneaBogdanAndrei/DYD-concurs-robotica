// Mișcările brațului, portate din clasa Arm din referinta-python/dofbot.py:
// pick/place cu apropiere de sus, coborâre în pași aproape verticală și
// înălțime minimă de tranzit.

import type { Arm } from "./arm/arm.js";
import type { Angles6 } from "./arm/protocol.js";
import * as C from "./config.js";
import type { Vec3 } from "./grid.js";
import { Unreachable, ik, wristForYaw } from "./kinematics.js";

const sleep = (ms: number) => new Promise<void>((resolve) => setTimeout(resolve, ms));

/** round() din Python: la .5 exact rotunjește spre par (2.5 -> 2, 3.5 -> 4). */
export function roundHalfEven(v: number): number {
  return Math.abs(v % 1) === 0.5 ? 2 * Math.round(v / 2) : Math.round(v);
}

export class JointLimitError extends Error {
  override name = "JointLimitError";
}

export function checkJointLimits(s: readonly number[]): void {
  if (s.length !== 6) throw new JointLimitError(`trebuie 6 unghiuri, am primit ${s.length}`);
  s.forEach((v, i) => {
    const lo = C.JOINT_MIN[i] as number;
    const hi = C.JOINT_MAX[i] as number;
    if (!Number.isFinite(v) || v < lo || v > hi) {
      throw new JointLimitError(`servo ${i + 1}: ${v} în afara limitelor ${lo}..${hi}`);
    }
  });
}

export interface RobotOptions {
  /** fără pauze de așteptare (placa simulată se mișcă instantaneu) */
  sim?: boolean;
  log?: (msg: string) => void;
  /** anunțat după fiecare comandă a ghearei, cu poziția curentă dacă e cunoscută */
  onGripper?: (value: number, at: Vec3 | null) => void;
}

export class Robot {
  grip: number = C.GRIP_OPEN;
  cur: number[] = [...C.HOME, C.GRIP_OPEN];
  /** poziția carteziană curentă (dacă e cunoscută) */
  curXyz: Vec3 | null = null;
  private readonly sim: boolean;
  private readonly log: (msg: string) => void;

  private constructor(
    readonly dev: Arm,
    private readonly opts: RobotOptions,
  ) {
    this.sim = opts.sim ?? false;
    this.log = opts.log ?? ((m) => console.log(m));
  }

  static async create(dev: Arm, opts: RobotOptions = {}): Promise<Robot> {
    if (!opts.sim) await sleep(100);
    return new Robot(dev, opts);
  }

  private wait(ms: number): Promise<void> {
    return this.sim ? Promise.resolve() : sleep(ms);
  }

  // --- nivel jos
  async write6(s: readonly number[], tMs: number): Promise<void> {
    const v = s.map(roundHalfEven);
    checkJointLimits(v);
    if (this.sim) this.log(`   [sim] servo [${v.join(", ")}] în ${Math.trunc(tMs)} ms`);
    await this.dev.writeAll(v as unknown as Angles6, Math.trunc(tMs));
    await this.wait(tMs + 120);
    this.cur = v;
  }

  async gripper(val: number, tMs: number = C.T_GRIP): Promise<void> {
    const v = Math.trunc(val);
    if (!(v >= C.JOINT_MIN[5]! && v <= C.JOINT_MAX[5]!)) {
      throw new JointLimitError(`gheara: ${val} în afara limitelor ${C.JOINT_MIN[5]}..${C.JOINT_MAX[5]}`);
    }
    this.grip = val;
    if (this.sim) this.log(`   [sim] gheara -> ${v}`);
    await this.dev.writeServo(6, v, Math.trunc(tMs));
    await this.wait(tMs + 100);
    this.cur[5] = v;
    this.opts.onGripper?.(v, this.curXyz);
  }

  /** Citește cele 6 servo-uri (funcționează și cu cuplul oprit); până la 6 încercări per servo. */
  async read(): Promise<(number | null)[]> {
    const out: (number | null)[] = [];
    for (const id of [1, 2, 3, 4, 5, 6] as const) {
      let v: number | null = null;
      for (let i = 0; i < 6; i++) {
        v = await this.dev.readServo(id);
        if (v !== null) break;
        await this.wait(30);
      }
      out.push(v);
    }
    return out;
  }

  async torque(on: boolean): Promise<void> {
    await this.dev.setTorque(on);
  }

  /** Arm_Buzzer_On(units): sună units*100 ms. Erorile sunt ignorate, ca în referință. */
  async beep(units = 1): Promise<void> {
    try {
      await this.dev.buzzerOn(units);
    } catch {
      // buzzerul nu e critic
    }
  }

  /** Trimite o poză articulară (5 servo-uri) păstrând starea ghearei. */
  async pose(servos5: readonly number[], tMs: number = C.T_MOVE): Promise<void> {
    await this.write6([...servos5.slice(0, 5), this.grip], tMs);
    this.curXyz = null;
  }

  async home(): Promise<void> {
    await this.pose(C.HOME, 1200);
  }

  // --- nivel cartezian
  async goto(
    x: number,
    y: number,
    z: number,
    yaw = 0.0,
    pitches: readonly number[] | null = null,
    tMs: number = C.T_MOVE,
  ): Promise<number> {
    const sol = ik(x, y, z, pitches);
    if (sol === null) {
      throw new Unreachable(`punct inaccesibil x=${x.toFixed(0)} y=${y.toFixed(0)} z=${z.toFixed(0)}`);
    }
    const s5 = wristForYaw(sol.servos[0], yaw);
    await this.write6([...sol.servos, s5, this.grip], tMs);
    this.curXyz = [x, y, z];
    return sol.pitch;
  }

  /** Înclinările candidate, cele mai apropiate de cea preferată primele. */
  static near(pref: number): number[] {
    return [...C.PITCH_CANDIDATES].sort((a, b) => Math.abs(a - pref) - Math.abs(b - pref) || a - b);
  }

  /**
   * Coborâre/ridicare aproape verticală (în FINE_STEPS pași). Înclinarea rămâne
   * cea cerută dacă se poate, altfel cea mai apropiată posibilă.
   */
  private async line(x: number, y: number, z0: number, z1: number, yaw: number, pitch: number, tMs: number) {
    const n = Math.max(1, C.FINE_STEPS);
    for (let i = 1; i <= n; i++) {
      const z = z0 + ((z1 - z0) * i) / n;
      await this.goto(x, y, z, yaw, Robot.near(pitch), tMs / n);
    }
  }

  /** Înclinarea pentru punctul de jos (prindere/plasare); întâi una comună cu apropierea. */
  private pitchFor(x: number, y: number, zLow: number, zHigh: number): number {
    for (const p of C.PITCH_CANDIDATES) {
      if (ik(x, y, zLow, [p]) && ik(x, y, zHigh, [p])) return p;
    }
    for (const p of C.PITCH_CANDIDATES) {
      if (ik(x, y, zLow, [p])) return p;
    }
    throw new Unreachable(`punct inaccesibil x=${x.toFixed(0)} y=${y.toFixed(0)} z=${zLow.toFixed(0)}`);
  }

  /** Ajunge deasupra (x, y) fără să treacă jos peste alte obiecte. */
  private async approach(x: number, y: number, zHover: number, yaw: number, pitch: number) {
    let zT = Math.max(zHover, C.TRANSIT_MIN_Z);
    if (this.curXyz !== null) {
      const [cx, cy, cz] = this.curXyz;
      zT = Math.max(zT, cz);
      if (cz < zT) await this.line(cx, cy, cz, zT, yaw, pitch, C.T_FINE);
    }
    await this.goto(x, y, zT, yaw, Robot.near(pitch), C.T_MOVE);
    if (zT > zHover) await this.line(x, y, zT, zHover, yaw, pitch, C.T_FINE);
  }

  async pick(x: number, y: number, z: number, yaw = 0.0): Promise<void> {
    const zh = z + C.HOVER_DZ;
    const p = this.pitchFor(x, y, z, zh);
    if (this.grip !== C.GRIP_OPEN) await this.gripper(C.GRIP_OPEN);
    await this.approach(x, y, zh, yaw, p);
    await this.line(x, y, zh, z, yaw, p, C.T_FINE);
    await this.gripper(C.GRIP_CLOSED);
    await this.line(x, y, z, zh, yaw, p, C.T_FINE);
  }

  async place(x: number, y: number, z0: number, yaw = 0.0): Promise<void> {
    const z = z0 + C.PLACE_GAP;
    const zh = z + C.HOVER_DZ;
    const p = this.pitchFor(x, y, z, zh);
    await this.approach(x, y, zh, yaw, p);
    await this.line(x, y, zh, z, yaw, p, C.T_FINE);
    await this.gripper(C.GRIP_OPEN);
    await this.line(x, y, z, zh, yaw, p, C.T_FINE);
  }
}
