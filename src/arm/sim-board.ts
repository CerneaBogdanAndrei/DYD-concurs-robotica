import { DofbotArm, type SmbusLike } from "./arm.js";
import { type ServoId, BOARD_ADDRESS, REG, SERVO_IDS, angleToPulse, assertServoId } from "./protocol.js";

/**
 * Placa Yahboom simulată la nivel de registre, pentru modul de simulare (Windows,
 * fără I2C). Brațul simulat trece prin același cod de codare ca cel real, deci și
 * prin aceleași rotunjiri.
 *
 * Mișcarea e instantanee: pulsul țintă devine imediat poziția citită.
 */
export class SimBoard implements SmbusLike {
  /** Pulsul curent al fiecărui servo, index 0 = servo 1. */
  readonly pulses: number[] = SERVO_IDS.map((id) => angleToPulse(id, 90));
  torque = true;
  buzzer = 0;
  lastTimeMs = 0;
  closed = false;

  constructor(readonly address: number = BOARD_ADDRESS) {}

  private check(address: number): void {
    if (this.closed) throw new Error("magistrala simulată e închisă");
    if (address !== this.address) {
      throw new Error(`niciun dispozitiv la 0x${address.toString(16)} (placa e la 0x${this.address.toString(16)})`);
    }
  }

  async writeI2cBlock(address: number, cmd: number, length: number, buffer: Buffer): Promise<unknown> {
    this.check(address);
    const b = [...buffer.subarray(0, length)];
    const word = (i: number) => (((b[i] ?? 0) << 8) | (b[i + 1] ?? 0)) & 0xffff;
    if (cmd === REG.SERVO_WRITE6_TIME && length === 2) {
      this.lastTimeMs = word(0);
    } else if (cmd === REG.SERVO_WRITE6 && length === 12) {
      for (let i = 0; i < 6; i++) this.pulses[i] = word(2 * i);
    } else if (cmd > REG.SERVO_WRITE && cmd <= REG.SERVO_WRITE + 6 && length === 4) {
      this.pulses[cmd - REG.SERVO_WRITE - 1] = word(0);
      this.lastTimeMs = word(2);
    } else {
      throw new Error(`scriere bloc necunoscută: reg 0x${cmd.toString(16)}, ${length} octeți`);
    }
    return { bytesWritten: length, buffer };
  }

  async writeByte(address: number, cmd: number, byte: number): Promise<void> {
    this.check(address);
    if (cmd === REG.TORQUE) this.torque = byte === 1;
    else if (cmd === REG.BUZZER) this.buzzer = byte;
    else if (cmd > REG.SERVO_READ && cmd <= REG.SERVO_READ + 6) return; // cerere de citire
    else throw new Error(`scriere octet necunoscută: reg 0x${cmd.toString(16)}`);
  }

  async readWord(address: number, cmd: number): Promise<number> {
    this.check(address);
    if (!(cmd > REG.SERVO_READ && cmd <= REG.SERVO_READ + 6)) {
      throw new Error(`citire cuvânt necunoscută: reg 0x${cmd.toString(16)}`);
    }
    const p = this.pulses[cmd - REG.SERVO_READ - 1] ?? 0;
    // Placa trimite H apoi L; SMBus le livrează little-endian.
    return ((p & 0xff) << 8) | ((p >> 8) & 0xff);
  }

  async close(): Promise<void> {
    this.closed = true;
  }

  /** Simulează mișcarea brațului cu mâna (cuplu oprit), pentru calibrarea prin învățare. */
  moveByHand(id: ServoId, angle: number): void {
    assertServoId(id);
    if (this.torque) throw new Error("cuplul e pornit: brațul nu poate fi mișcat cu mâna");
    this.pulses[id - 1] = angleToPulse(id, angle);
  }
}

/** Braț complet simulat: același DofbotArm, dar peste SimBoard în loc de I2C. */
export function openSimArm(): { arm: DofbotArm; board: SimBoard } {
  const board = new SimBoard();
  return { arm: new DofbotArm(board, board.address), board };
}
