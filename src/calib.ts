// calib.json: tot ce învață brațul în sala de concurs.
//
//   poses: poze articulare cu nume (servo 1..5), ex. "observe_source"
//   sites: locurile de prindere/plasare, adresate prin `location` în ArmInterface
//     - "grid":  caroiaj 3x3 învățat pe celule (source, dest, ...)
//     - "point": o singură poziție învățată (zone_red, ...)
//   gripper: [unghi servo 6, deschiderea interioară în mm], măsurate cu șublerul (npm run calib -- gheara UNGHI)

import { existsSync, readFileSync, renameSync, statSync, writeFileSync } from "node:fs";
import { Grid, type Vec3 } from "./grid.js";

export interface GridSite {
  kind: "grid";
  /** { r0c0: [x, y, z], ... } punctele învățate cu mâna */
  points: Record<string, Vec3>;
}

export interface PointSite {
  kind: "point";
  /** punctul de prindere pentru un cub așezat pe masă în zonă */
  xyz: Vec3;
}

export type Site = GridSite | PointSite;

export interface Calib {
  poses: Record<string, number[]>;
  sites: Record<string, Site>;
  /** [unghi servo 6, deschidere interioară mm]; cel puțin 2 puncte = gheara se poate comanda în mm */
  gripper?: [number, number][];
}

export class CalibError extends Error {
  override name = "CalibError";
}

export function emptyCalib(): Calib {
  return { poses: {}, sites: {} };
}

export function loadCalib(path: string): Calib {
  if (!existsSync(path)) return emptyCalib();
  const raw = JSON.parse(readFileSync(path, "utf8")) as Partial<Calib>;
  const out: Calib = { poses: raw.poses ?? {}, sites: raw.sites ?? {} };
  if (raw.gripper) out.gripper = raw.gripper;
  return out;
}

/** Calibrarea recitită doar când fișierul s-a schimbat (ex. după `npm run calib` cu serverul pornit). */
export function calibLoader(path: string): () => Calib {
  let mtime = -1;
  let cached = emptyCalib();
  return () => {
    const m = existsSync(path) ? statSync(path).mtimeMs : 0;
    if (m !== mtime) {
      cached = loadCalib(path);
      mtime = m;
    }
    return cached;
  };
}

/** Scriere atomică: un fișier pe jumătate scris ar pierde toată calibrarea. */
export function saveCalib(calib: Calib, path: string): void {
  const tmp = `${path}.tmp`;
  writeFileSync(tmp, JSON.stringify(calib, null, 2) + "\n");
  renameSync(tmp, path);
}

export function requirePose(calib: Calib, name: string): number[] {
  const pose = calib.poses[name];
  if (!pose) {
    throw new CalibError(`Poza '${name}' nu e învățată. Rulați: npm run calib -- poza ${name}`);
  }
  return pose;
}

export function requireSite(calib: Calib, name: string): Site {
  const site = calib.sites[name];
  if (!site) {
    throw new CalibError(
      `Locul '${name}' nu e calibrat. Rulați: npm run calib -- celule ${name}  (caroiaj) ` +
        `sau  npm run calib -- zona ${name}  (punct)`,
    );
  }
  return site;
}

/** Modelul de caroiaj potrivit din punctele învățate ale unui loc. */
export function gridOf(site: GridSite): Grid {
  return Grid.fromPoints(site.points).grid;
}

/** Punctele măsurate ale ghearei, ordonate după deschidere; [] dacă sunt mai puțin de 2 sau se contrazic. */
export function gripTable(calib: Calib): [number, number][] {
  const pts = (calib.gripper ?? []).filter(
    (p) => Array.isArray(p) && p.length === 2 && p.every((v) => typeof v === "number" && Number.isFinite(v)),
  );
  if (pts.length < 2) return [];
  const byMm = [...pts].sort((a, b) => a[1] - b[1]);
  // O deschidere mai mare cere mereu un unghi în același sens: altfel măsurătorile se contrazic.
  const dir = Math.sign(byMm[1]![0] - byMm[0]![0]);
  for (let i = 1; i < byMm.length; i++) {
    if (dir === 0 || byMm[i]![1] === byMm[i - 1]![1] || Math.sign(byMm[i]![0] - byMm[i - 1]![0]) !== dir) return [];
  }
  return byMm;
}

/** Unghiul servo 6 pentru o deschidere interioară (mm), interpolat liniar între punctele măsurate. */
export function gripServoForMm(calib: Calib, mm: number): number {
  const t = gripTable(calib);
  if (t.length === 0) {
    throw new CalibError("Gheara nu e calibrată în mm. Rulați: npm run calib -- gheara UNGHI (cel puțin 2 unghiuri)");
  }
  const lo = t[0]!;
  const hi = t[t.length - 1]!;
  if (mm < lo[1] || mm > hi[1]) {
    throw new CalibError(
      `Deschiderea ${mm.toFixed(1)} mm e în afara celor măsurate (${lo[1]}..${hi[1]} mm): ` +
        "măsurați și un unghi pentru ea (npm run calib -- gheara UNGHI)",
    );
  }
  for (let i = 1; i < t.length; i++) {
    const [a0, m0] = t[i - 1]!;
    const [a1, m1] = t[i]!;
    if (mm <= m1) return a0 + ((a1 - a0) * (mm - m0)) / (m1 - m0);
  }
  return hi[0];
}
