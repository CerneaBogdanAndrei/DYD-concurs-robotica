"""Foaia pusă la loc după ce s-a mișcat: din poza de start, caroiajul de acum față de imaginea de referință
(`ref/observe_small`), în mm pe foaie, cu direcția în care trebuie împinsă foaia. Brațul nu se mișcă (trebuie să fie
deja în poza de start). Rulează până când foaia e la cel mult TOL_MM și rotită sub TOL_DEG în două analize la rând,
apoi bipăie de 3 ori.

2026-10-30: un cub prins doar de marginea de sus atârna mai jos în gheară și, la punere, a tras foaia ~1 cm.

    ai/.venv/bin/python ai/scripts/calib_live/autocal/foaie_la_loc.py [--o-data]
"""
import argparse
import json
import math
import sys

import cv2
import numpy as np

from traseu_cub import COL, REF, ROW, UV, analyses, post, say

TOL_MM = 1.0
TOL_DEG = 0.5


def offset(obs, H):
    """-> (dx, dy, unghi): cât s-a mutat centrul caroiajului (mm: x spre c2, y spre robot) și cât s-a rotit (grd)."""
    cur = cv2.perspectiveTransform(np.float64([obs["geometry"]["corners_px"]]), H).reshape(-1, 2) * (COL, ROW)
    true = UV.astype(np.float64) * (COL, ROW)
    a, b = true - true.mean(0), cur - cur.mean(0)
    ang = math.atan2((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum(), (a * b).sum())
    d = cur.mean(0) - true.mean(0)
    return float(d[0]), float(d[1]), math.degrees(ang)


def advice(dx, dy, ang):
    parts = []
    if abs(dx) > TOL_MM / 2:
        parts.append(f"{abs(dx):.0f} mm spre {'stânga' if dx > 0 else 'dreapta'}")
    if abs(dy) > TOL_MM / 2:
        parts.append(f"{abs(dy):.0f} mm {'departe de robot' if dy > 0 else 'spre robot'}")
    if abs(ang) > TOL_DEG / 2:
        parts.append(f"rotită {abs(ang):.1f} grd în sensul {'invers acelor de ceas' if ang > 0 else 'acelor de ceas'}")
    return "împingeți foaia " + ", ".join(parts) if parts else "la loc"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--o-data", action="store_true", help="doar o măsurătoare, fără așteptare")
    a = ap.parse_args()
    ref = json.load(open(REF.with_suffix(".json")))
    H = cv2.getPerspectiveTransform(np.float32(ref["geometry"]["corners_px"]), UV)
    good = 0
    while True:
        (obs, _), = analyses(1)
        if not obs["geometry"].get("ok"):
            say("caroiajul nu se vede întreg")
            good = 0
            continue
        dx, dy, ang = offset(obs, H)
        ok = math.hypot(dx, dy) <= TOL_MM and abs(ang) <= TOL_DEG
        say(f"caroiajul mutat {dx:+.1f} mm pe coloane (+ = spre c2), {dy:+.1f} mm spre robot, rotit {ang:+.2f} grd -> "
            f"{advice(dx, dy, ang)}")
        if a.o_data:
            return
        good = good + 1 if ok else 0
        if good >= 2:
            post("beep", {"n": 3})
            say("Foaia e la loc.")
            return


if __name__ == "__main__":
    sys.exit(main())
