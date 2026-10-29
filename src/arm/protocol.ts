// Protocolul plăcii Yahboom Dofbot, reprodus după Arm_Lib.py (clasa Arm_Device).
//
// DE VERIFICAT față de Arm_Lib.py de pe Pi (`pip3 show -f Arm_Lib`): registrele,
// intervalele de puls și ordinea octeților sunt scrise după versiunea publică a
// bibliotecii. `npm run compara:armlib -- codare` verifică automat codarea.
//
// Funcțiile de aici sunt pure: nu ating magistrala, doar descriu operațiile I2C.

export const BOARD_ADDRESS = 0x15;
export const I2C_BUS = 1;

export const REG = {
  /** + id (1..6): scriere un servo [puls_H, puls_L, timp_H, timp_L] */
  SERVO_WRITE: 0x10,
  /** toate cele 6 servo-uri: 12 octeți de puls (H, L pentru fiecare) */
  SERVO_WRITE6: 0x1d,
  /** timpul pentru SERVO_WRITE6: [timp_H, timp_L], trimis înaintea pulsurilor */
  SERVO_WRITE6_TIME: 0x1e,
  /** 1 = cuplu pornit, 0 = oprit (brațul se poate mișca cu mâna) */
  TORQUE: 0x1a,
  /** + id (1..6): scrie 0, apoi citește un cuvânt cu pulsul */
  SERVO_READ: 0x30,
  /** 0 = oprit, 1..50 = pornit pentru n*100 ms, 0xff = continuu */
  BUZZER: 0x06,
} as const;

/** Pauza dintre cererea de citire și citirea propriu-zisă (time.sleep(0.003) în Arm_Lib). */
export const READ_DELAY_MS = 3;

export type ServoId = 1 | 2 | 3 | 4 | 5 | 6;
export const SERVO_IDS: readonly ServoId[] = [1, 2, 3, 4, 5, 6];

/** Unghiurile celor 6 servo-uri, în ordinea id 1..6, în grade (convenția Arm_Lib). */
export type Angles6 = readonly [number, number, number, number, number, number];

export type I2cOp =
  | { kind: "block"; reg: number; bytes: number[] }
  | { kind: "byte"; reg: number; value: number };

/** Servo-urile 2, 3, 4 sunt montate invers: Arm_Lib trimite 180 - unghi. */
export function isInverted(id: ServoId): boolean {
  return id === 2 || id === 3 || id === 4;
}

interface ServoRange {
  maxAngle: number;
  pulseMin: number;
  pulseMax: number;
}

/** Servo-ul 5 (rotația încheieturii) are 0..270°, restul 0..180°. */
export function servoRange(id: ServoId): ServoRange {
  return id === 5
    ? { maxAngle: 270, pulseMin: 380, pulseMax: 3700 }
    : { maxAngle: 180, pulseMin: 900, pulseMax: 3100 };
}

export function assertServoId(id: number): asserts id is ServoId {
  if (!Number.isInteger(id) || id < 1 || id > 6) {
    throw new RangeError(`id servo invalid: ${id} (trebuie 1..6)`);
  }
}

function assertAngle(id: ServoId, angle: number): void {
  const { maxAngle } = servoRange(id);
  if (!Number.isFinite(angle) || angle < 0 || angle > maxAngle) {
    throw new RangeError(`unghi invalid pentru servo ${id}: ${angle} (trebuie 0..${maxAngle})`);
  }
}

function assertTime(timeMs: number): void {
  if (!Number.isInteger(timeMs) || timeMs < 0 || timeMs > 0xffff) {
    throw new RangeError(`timp invalid: ${timeMs} ms (trebuie întreg 0..65535)`);
  }
}

/**
 * Unghi (convenția Arm_Lib) -> puls trimis plăcii.
 * Aceeași formulă și aceeași trunchiere ca `int(...)` din Python.
 */
export function angleToPulse(id: ServoId, angle: number): number {
  assertAngle(id, angle);
  const { maxAngle, pulseMin, pulseMax } = servoRange(id);
  const a = isInverted(id) ? 180 - angle : angle;
  return Math.trunc(((pulseMax - pulseMin) * (a - 0)) / (maxAngle - 0) + pulseMin);
}

/**
 * Puls citit -> unghi, ca în Arm_serial_servo_read: 0 înseamnă eroare de citire,
 * iar valorile din afara intervalului dau null.
 */
export function pulseToAngle(id: ServoId, pulse: number): number | null {
  if (pulse === 0) return null;
  const { maxAngle, pulseMin, pulseMax } = servoRange(id);
  let a = Math.trunc(((maxAngle - 0) * (pulse - pulseMin)) / (pulseMax - pulseMin) + 0);
  if (a > maxAngle || a < 0) return null;
  if (isInverted(id)) a = 180 - a;
  return a;
}

/** Placa trimite pulsul ca H, L; SMBus îl citește little-endian, deci Arm_Lib inversează octeții. */
export function swapWord(raw: number): number {
  return ((raw >> 8) & 0xff) | ((raw << 8) & 0xff00);
}

export function decodeRead(id: ServoId, rawWord: number): number | null {
  return pulseToAngle(id, swapWord(rawWord));
}

function hiLo(value: number): [number, number] {
  return [(value >> 8) & 0xff, value & 0xff];
}

/** Arm_serial_servo_write6: întâi timpul pe 0x1e, apoi cele 12 octeți de puls pe 0x1d. */
export function encodeWrite6(angles: Angles6, timeMs: number): I2cOp[] {
  assertTime(timeMs);
  const bytes = SERVO_IDS.flatMap((id, i) => hiLo(angleToPulse(id, angles[i] as number)));
  return [
    { kind: "block", reg: REG.SERVO_WRITE6_TIME, bytes: hiLo(timeMs) },
    { kind: "block", reg: REG.SERVO_WRITE6, bytes },
  ];
}

/** Arm_serial_servo_write pentru un singur servo (id 1..6). */
export function encodeWriteServo(id: ServoId, angle: number, timeMs: number): I2cOp[] {
  assertTime(timeMs);
  const pulse = angleToPulse(id, angle);
  return [{ kind: "block", reg: REG.SERVO_WRITE + id, bytes: [...hiLo(pulse), ...hiLo(timeMs)] }];
}

/** Arm_serial_set_torque. */
export function encodeTorque(on: boolean): I2cOp[] {
  return [{ kind: "byte", reg: REG.TORQUE, value: on ? 0x01 : 0x00 }];
}

/**
 * Arm_Buzzer_On(delay) / Arm_Buzzer_Off.
 * `duration100ms`: 1..50 = n*100 ms, 0xff = continuu, 0 = oprit.
 */
export function encodeBuzzer(duration100ms: number): I2cOp[] {
  const ok =
    Number.isInteger(duration100ms) &&
    ((duration100ms >= 0 && duration100ms <= 50) || duration100ms === 0xff);
  if (!ok) throw new RangeError(`durată buzzer invalidă: ${duration100ms} (0..50 sau 255)`);
  return [{ kind: "byte", reg: REG.BUZZER, value: duration100ms }];
}
