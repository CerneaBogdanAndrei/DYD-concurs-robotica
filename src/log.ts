const T0 = Date.now();

/** Mesaj cu timpul scurs de la pornire, ca în referință: `[  12.3s] ...`. */
export function log(msg: string): void {
  const t = ((Date.now() - T0) / 1000).toFixed(1).padStart(6);
  console.log(`[${t}s] ${msg}`);
}
