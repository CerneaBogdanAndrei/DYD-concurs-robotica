import type * as I2cBus from "i2c-bus";
import { DofbotArm } from "./arm.js";
import { BOARD_ADDRESS, I2C_BUS } from "./protocol.js";

/** Deschide brațul real pe /dev/i2c-<bus> (doar pe Raspberry Pi). */
export async function openI2cArm(busNumber = I2C_BUS, address = BOARD_ADDRESS): Promise<DofbotArm> {
  let api: typeof I2cBus;
  try {
    // i2c-bus e CommonJS și opțional (nu se compilează pe Windows), deci îl încărcăm la nevoie.
    const imported = await import("i2c-bus");
    api = (imported as unknown as { default?: typeof I2cBus }).default ?? imported;
  } catch (cause) {
    throw new Error(
      "Pachetul i2c-bus nu e disponibil. Pe Raspberry Pi rulează `npm install`; " +
        "pe Windows folosește modul de simulare.",
      { cause },
    );
  }
  const bus = await api.openPromisified(busNumber);
  return new DofbotArm(bus, address);
}
