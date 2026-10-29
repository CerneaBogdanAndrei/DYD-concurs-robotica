// calib.json: tot ce învață brațul în sala de concurs.
//
//   poses: poze articulare cu nume (servo 1..5), ex. "observe_source"
//   sites: locurile de prindere/plasare, adresate prin `location` în ArmInterface
//     - "grid":  caroiaj 3x3 învățat pe celule (source, dest, ...)
//     - "point": o singură poziție învățată (zone_red, ...)

import { existsSync, readFileSync, renameSync, writeFileSync } from "node:fs";
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
  return { poses: raw.poses ?? {}, sites: raw.sites ?? {} };
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
