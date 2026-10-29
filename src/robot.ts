// Mișcările brațului, portate din clasa Arm din referinta-python/dofbot.py:
// pick/place cu apropiere de sus, coborâre în pași aproape verticală și
// înălțime minimă de tranzit.
//
// Gheara printre cuburi (2026-10-29):
//   A. la pick, gheara se pre-închide la deschiderea de abordare ÎNAINTE de deplasare: nu călătorește și nu
//      coboară larg deschisă (ar lovi cuburile vecine);
//   C. la place, se deschide doar cât să elibereze cubul, apoi urcă strict vertical până la înălțimea de
//      tranzit; abia acolo se poate mișca pe XY sau redeschide gheara.
// Deschiderile vin de la ArmService (din mm, prin calibrarea ghearei); implicit GRIP_OPEN, ca înainte.

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
  /** la ultima prindere, jos: unghiurile comandate, citite înainte și după închiderea ghearei (diagnoză) */
  lastGrasp: { sent: number[]; bottom: (number | null)[]; closed: (number | null)[] } | null = null;
  /** yaw-ul și înclinarea ghearei la ultima mișcare carteziană (pentru ridicarea pe loc) */
  private curYaw = 0.0;
  private curPitch = -90;
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
  /** `pauseMs` = așteptarea după mișcare (0 = următoarea comandă o continuă, fără oprire). */
  async write6(s: readonly number[], tMs: number, pauseMs: number = C.PAUSE_MS): Promise<void> {
    const v = s.map(roundHalfEven);
    checkJointLimits(v);
    if (this.sim) this.log(`   [sim] servo [${v.join(", ")}] în ${Math.trunc(tMs)} ms`);
    await this.dev.writeAll(v as unknown as Angles6, Math.trunc(tMs));
    await this.wait(tMs + pauseMs);
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
    pauseMs: number = C.PAUSE_MS,
  ): Promise<number> {
    const sol = ik(x, y, z, pitches);
    if (sol === null) {
      throw new Unreachable(`punct inaccesibil x=${x.toFixed(0)} y=${y.toFixed(0)} z=${z.toFixed(0)}`);
    }
    const s5 = wristForYaw(sol.servos[0], yaw);
    await this.write6([...sol.servos, s5, this.grip], tMs, pauseMs);
    this.curXyz = [x, y, z];
    this.curYaw = yaw;
    this.curPitch = sol.pitch;
    return sol.pitch;
  }

  /** Înclinările candidate, cele mai apropiate de cea preferată primele. */
  static near(pref: number): number[] {
    return [...C.PITCH_CANDIDATES].sort((a, b) => Math.abs(a - pref) - Math.abs(b - pref) || a - b);
  }

  /**
   * Coborâre/ridicare aproape verticală (în `steps` pași, implicit FINE_STEPS). Înclinarea rămâne
   * cea cerută dacă se poate, altfel cea mai apropiată posibilă. Pașii merg legați (fără oprire între ei),
   * iar la capăt brațul stă SETTLE_MS: o mișcare continuă, fără smucituri.
   */
  private async line(
    x: number,
    y: number,
    z0: number,
    z1: number,
    yaw: number,
    pitch: number,
    tMs: number,
    steps: number = C.FINE_STEPS,
  ) {
    const n = Math.max(1, steps);
    for (let i = 1; i <= n; i++) {
      const z = z0 + ((z1 - z0) * i) / n;
      // un pas intermediar poate cădea într-un gol dintre înclinările candidate (din 5 în 5 grd): se mută cu câțiva mm
      const tries = i < n ? [0, 2, -2, 4, -4] : [0];
      for (const [k, dz] of tries.entries()) {
        try {
          await this.goto(x, y, z + dz, yaw, Robot.near(pitch), tMs / n, i < n ? 0 : C.SETTLE_MS);
          break;
        } catch (e) {
          if (!(e instanceof Unreachable) || k === tries.length - 1) throw e;
        }
      }
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

  /** Ridicare strict verticală, pe loc, până la înălțimea de tranzit (dacă poziția e cunoscută și e mai jos). */
  private async liftToTransit(): Promise<void> {
    if (this.curXyz === null) return;
    const [cx, cy, cz] = this.curXyz;
    if (cz < C.TRANSIT_MIN_Z) {
      await this.line(cx, cy, cz, C.TRANSIT_MIN_Z, this.curYaw, this.curPitch, C.T_FINE, C.VERTICAL_STEPS);
    }
  }

  /** `openGrip` = unghiul ghearei la abordare (regula A); implicit GRIP_OPEN (gheara necalibrată în mm). */
  async pick(x: number, y: number, z: number, yaw = 0.0, openGrip: number = C.GRIP_OPEN): Promise<void> {
    const zh = z + C.HOVER_DZ;
    const p = this.pitchFor(x, y, z, zh);
    // A: gheara își schimbă deschiderea doar sus (deasupra cuburilor), apoi călătorește pre-închisă.
    await this.liftToTransit();
    if (this.grip !== openGrip) await this.gripper(openGrip);
    await this.approach(x, y, zh, yaw, p);
    await this.line(x, y, zh, z, yaw, p, C.T_FINE, C.VERTICAL_STEPS);
    const sent = [...this.cur];
    const bottom = await this.read();
    await this.gripper(C.GRIP_CLOSED);
    this.lastGrasp = { sent, bottom, closed: await this.read() };
    await this.line(x, y, z, zh, yaw, p, C.T_FINE, C.VERTICAL_STEPS);
  }

  /** `releaseGrip` = unghiul ghearei la eliberare (regula C); implicit GRIP_RELEASE (gheara necalibrată în mm). */
  async place(x: number, y: number, z0: number, yaw = 0.0, releaseGrip: number = C.GRIP_RELEASE): Promise<void> {
    const z = z0 + C.PLACE_GAP;
    const zh = z + C.HOVER_DZ;
    const p = this.pitchFor(x, y, z, zh);
    await this.approach(x, y, zh, yaw, p);
    await this.line(x, y, zh, z, yaw, p, C.T_FINE, C.VERTICAL_STEPS);
    // C: deschidere mică și lentă, apoi sus pe verticală, fără nicio mișcare pe XY: primii LIFT_SLOW_DZ mm încet.
    await this.gripper(releaseGrip, C.T_RELEASE);
    const zSlow = Math.min(z + C.LIFT_SLOW_DZ, Math.max(zh, C.TRANSIT_MIN_Z));
    await this.line(x, y, z, zSlow, yaw, p, C.T_LIFT_SLOW, C.VERTICAL_STEPS);
    if (zSlow < Math.max(zh, C.TRANSIT_MIN_Z)) {
      await this.line(x, y, zSlow, Math.max(zh, C.TRANSIT_MIN_Z), yaw, p, C.T_FINE, C.VERTICAL_STEPS);
    }
  }
}
