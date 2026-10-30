"""Calibrează camera în fiecare vedere a sortării (centru, stânga, dreapta, jos) pe liniile caroiajului din imagine și
arată cuburile găsite, în milimetrii foii.

Imaginile vin din `vederi_zone.py` (foaia cu caroiajul gol). Liniile se găsesc în imagine și se numerotează după
vedere: la stânga prima coloană din stânga e marginea caroiajului (x = 0), la dreapta ultima e x = 99 mm, jos ultima
linie de jos e y = 99 mm; la centru se folosesc cele 16 intersecții din colțurile găsite de viziune. Din intersecții
rezultă poza camerei (PnP), scrisă în `<folder>/calib_vederi.json`; desenul de verificare (caroiajul proiectat înapoi,
cuburile) în `<folder>/verif/`.

    ai/.venv/bin/python ai/scripts/calib_live/autocal/calib_vederi.py vederi/caroiaj_gol
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from detect import intersections
from vedere import COL, CUBE_MM, DIST, K, ROW, Camera, crossing, cube_tops, distort_points, lines, outside_gap_mm, \
    undistort_points, visible_crossing


def number(view, cols, rows):
    """Indicii liniilor văzute (coloane = x constant, linii = y constant), după vedere."""
    if view.startswith("stanga"):
        ci = list(range(len(cols)))
    elif view.startswith("dreapta"):
        ci = list(range(4 - len(cols), 4))
    else:
        ci = list(range(len(cols))) if len(cols) == 4 else None
    if view.startswith("jos"):
        ri = list(range(4 - len(rows), 4))
    else:
        ri = list(range(len(rows))) if len(rows) == 4 else None
    return ci, ri


def observed(path, meta):
    """Intersecțiile văzute: (puncte pe foaie în mm, pixeli fără distorsiune, text). La centru din colțurile viziunii
    (`detect.py`), altfel din liniile găsite și numerotate după vedere."""
    raw = cv2.imread(str(path))
    geo = meta["observation"]["geometry"]
    if geo.get("ok"):
        pts, _ = intersections(raw, geo["corners_px"])
        return [(i * COL, j * ROW) for j in range(4) for i in range(4)], undistort_points(pts.reshape(-1, 2)), \
            "16 intersecții din colțurile viziunii"
    und = cv2.undistort(raw, K, DIST)
    fa, fb = lines(und)
    # coloanele caroiajului sunt aproape verticale în toate vederile
    vert = lambda fam: np.mean([abs(l[1][1]) for l in fam]) if fam else 0.0
    cols, rows = (fa, fb) if vert(fa) > vert(fb) else (fb, fa)
    h, w = und.shape[:2]
    cols = sorted(cols, key=lambda l: crossing(l, (np.array([0, h / 2]), np.array([1.0, 0])))[0])
    rows = sorted(rows, key=lambda l: crossing(l, (np.array([w / 2, 0]), np.array([0, 1.0])))[1])
    ci, ri = number(path.stem, cols, rows)
    info = f"{len(cols)} coloane, {len(rows)} linii"
    if ci is None or ri is None:
        return [], np.zeros((0, 2)), info + ": nu le pot numerota"
    obj, img = [], []
    for i, lc in zip(ci, cols):
        for j, lr in zip(ri, rows):
            x = visible_crossing(lc, lr)
            if x is not None:
                obj.append((i * COL, j * ROW))
                img.append(x)
    return obj, np.array(img).reshape(-1, 2), info + f", {len(obj)} intersecții"


def views(folder):
    for path in sorted(folder.glob("*.jpg")):
        yield path, json.load(open(path.with_suffix(".json")))


def draw(raw, cam, cubes, label):
    vis = raw.copy()
    for k in range(4):  # caroiajul proiectat înapoi (z = 0)
        for a, b in (((k * COL, 0, 0), (k * COL, 3 * ROW, 0)), ((0, k * ROW, 0), (3 * COL, k * ROW, 0))):
            t = np.linspace(0, 1, 30)[:, None]
            pts = distort_points(cam.project(np.array(a) * (1 - t) + np.array(b) * t))
            cv2.polylines(vis, [np.int32(pts)], False, (255, 0, 255), 1)
    for c in cubes:
        cnt = np.int32(c["contour_px"])
        cv2.polylines(vis, [cnt], True, (0, 255, 255) if not c["clipped"] else (0, 0, 255), 2)
        x, y = cnt.mean(0)
        txt = f"{c['color'][0]} ({c['center_mm'][0]:.0f};{c['center_mm'][1]:.0f}) {c['sides_mm'][0]:.0f}x{c['sides_mm'][1]:.0f}"
        cv2.putText(vis, txt, (int(x) - 70, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)
        cv2.putText(vis, txt, (int(x) - 70, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(vis, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    return vis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    a = ap.parse_args()
    folder = Path(a.folder).resolve()
    out = {}
    (folder / "verif").mkdir(exist_ok=True)
    for path, meta in views(folder):
        obj, img, info = observed(path, meta)
        if len(obj) < 6:
            print(f"{path.stem}: {info}: prea puține")
            continue
        cam, rms, res = Camera.fit(obj, img)
        info += f", abatere {rms:.2f} px (max {res.max():.2f})"
        print(f"{path.stem}: {info}")
        raw = cv2.imread(str(path))
        cubes = [c for c in cube_tops(raw, cam) if outside_gap_mm(c["center_mm"]) > 0]
        for c in cubes:
            print(f"   {c['color']}: centru ({c['center_mm'][0]:.1f}; {c['center_mm'][1]:.1f}) mm, fața "
                  f"{c['sides_mm'][0]:.1f} x {c['sides_mm'][1]:.1f} mm (umplere {c['fill']:.2f}), rotit {c['yaw_deg']:+.1f} grd, "
                  f"la {outside_gap_mm(c['center_mm']):.0f} mm de caroiaj{' (TĂIAT de marginea imaginii)' if c['clipped'] else ''}")
        print(f"   camera la ({cam.nadir[0]:.1f}; {cam.nadir[1]:.1f}) mm, înălțime {cam.height:.1f} mm")
        out[path.stem] = {"read": meta["read"], "sent": meta["sent"], "camera": cam.as_dict(), "rms_px": rms,
                          "points": len(obj), "cubes": [{k: v for k, v in c.items() if k != "contour_px"} for c in cubes]}
        cv2.imwrite(str(folder / "verif" / path.name), draw(raw, cam, cubes, f"{path.stem} {info}"))
    json.dump({"at": time.strftime("%Y-%m-%d %H:%M:%S"), "cube_mm": CUBE_MM, "optics": {"K": K.tolist(), "dist": DIST.tolist()},
               "views": out}, open(folder / "calib_vederi.json", "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
