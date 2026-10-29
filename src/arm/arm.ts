import {
  type Angles6,
  type I2cOp,
  type ServoId,
  BOARD_ADDRESS,
  READ_DELAY_MS,
  REG,
  SERVO_IDS,
  assertServoId,
  decodeRead,
  encodeBuzzer,
  encodeTorque,
  encodeWrite6,
  encodeWriteServo,
} from "./protocol.js";

/**
 * Subsetul de SMBus folosit de Arm_Lib. `PromisifiedBus` din i2c-bus îl implementează
 * direct; placa simulată (sim-board.ts) la fel.
 */
export interface SmbusLike {
  writeI2cBlock(address: number, cmd: number, length: number, buffer: Buffer): Promise<unknown>;
  writeByte(address: number, cmd: number, byte: number): Promise<void>;
  readWord(address: number, cmd: number): Promise<number>;
  close(): Promise<void>;
}

/** Operațiile de bază ale brațului, echivalente cu Arm_Device din Arm_Lib. */
export interface Arm {
  writeAll(angles: Angles6, timeMs: number): Promise<void>;
  writeServo(id: ServoId, angle: number, timeMs: number): Promise<void>;
  /** null dacă placa nu răspunde corect (ca None în Arm_Lib). */
  readServo(id: ServoId): Promise<number | null>;
  readAll(): Promise<(number | null)[]>;
  setTorque(on: boolean): Promise<void>;
  /** 1..50 = n*100 ms, 0xff = continuu. */
  buzzerOn(duration100ms?: number): Promise<void>;
  buzzerOff(): Promise<void>;
  close(): Promise<void>;
}

const sleep = (ms: number) => new Promise<void>((resolve) => setTimeout(resolve, ms));

/**
 * Brațul Dofbot peste o magistrală SMBus. Spre deosebire de Arm_Lib, erorile I2C
 * sunt aruncate, nu doar afișate: decizia de reîncercare e a nivelului de sus.
 */
export class DofbotArm implements Arm {
  // Operațiile se serializează: o citire = scriere registru + pauză + citire,
  // iar alt apel intercalat ar strica secvența.
  private queue: Promise<unknown> = Promise.resolve();

  constructor(
    private readonly bus: SmbusLike,
    private readonly address: number = BOARD_ADDRESS,
  ) {}

  private exclusive<T>(fn: () => Promise<T>): Promise<T> {
    const run = this.queue.then(fn, fn);
    this.queue = run.catch(() => undefined);
    return run;
  }

  private async execute(ops: I2cOp[]): Promise<void> {
    for (const op of ops) {
      if (op.kind === "block") {
        await this.bus.writeI2cBlock(this.address, op.reg, op.bytes.length, Buffer.from(op.bytes));
      } else {
        await this.bus.writeByte(this.address, op.reg, op.value);
      }
    }
  }

  writeAll(angles: Angles6, timeMs: number): Promise<void> {
    const ops = encodeWrite6(angles, timeMs);
    return this.exclusive(() => this.execute(ops));
  }

  writeServo(id: ServoId, angle: number, timeMs: number): Promise<void> {
    assertServoId(id);
    const ops = encodeWriteServo(id, angle, timeMs);
    return this.exclusive(() => this.execute(ops));
  }

  readServo(id: ServoId): Promise<number | null> {
    assertServoId(id);
    const reg = REG.SERVO_READ + id;
    return this.exclusive(async () => {
      await this.bus.writeByte(this.address, reg, 0x00);
      await sleep(READ_DELAY_MS);
      return decodeRead(id, await this.bus.readWord(this.address, reg));
    });
  }

  async readAll(): Promise<(number | null)[]> {
    const result: (number | null)[] = [];
    for (const id of SERVO_IDS) result.push(await this.readServo(id));
    return result;
  }

  setTorque(on: boolean): Promise<void> {
    return this.exclusive(() => this.execute(encodeTorque(on)));
  }

  buzzerOn(duration100ms = 0xff): Promise<void> {
    // Arm_Buzzer_On nu trimite nimic pentru delay = 0.
    if (duration100ms === 0) return Promise.resolve();
    const ops = encodeBuzzer(duration100ms);
    return this.exclusive(() => this.execute(ops));
  }

  buzzerOff(): Promise<void> {
    return this.exclusive(() => this.execute(encodeBuzzer(0)));
  }

  close(): Promise<void> {
    return this.exclusive(() => this.bus.close());
  }
}
