"""Unde e axa bazei (servo 1) față de foaie și cu cât se rotește baza de fapt, măsurat din cameră.

Brațul stă în `observe_small` și rotește doar baza cu ±PAS grade. Camera e pe braț, deci la o rotire a bazei cu θ
punctul de pe masă văzut de un pixel fix se rotește cu θ în jurul axei bazei. Din perechile de imagini rezultă
rotirea reală θ (comparată cu cea comandată) și centrul ei, adică axa bazei în milimetrii foii. Nu folosește
modelul brațului, iar din cameră doar obiectivul (F, K1, ca în `mutare_foaie.py`).

Cere serviciul și previzualizarea pornite și foaia goală (un cub strică liniile de lângă el).

    ai/.venv/bin/python ai/scripts/calib_live/autocal/axa_bazei.py [--pas 4] [--pasi 1]
"""
import argparse
import calendar
import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE))
from detect import intersections  # noqa: E402
from mutare_foaie import COL, GRID, ROW, post, undist  # noqa: E402

OBSERVE = "observe_small"


def snapshot(preview, t0):
    """Analiza și imaginea de după t0 (brațul oprit) -> 16 intersecții (px, fără distorsiune)."""
    for _ in range(40):
        time.sleep(0.25)
        obs = json.load(open(ROOT / "ai/out/preview/latest_observation.json"))
        stamp = calendar.timegm(time.strptime(obs["captured_at"], "%Y-%m-%dT%H:%M:%SZ"))
        if stamp >= t0:
            break
    else:
        sys.exit("Previzualizarea nu a dat o analiză nouă: rulează `brain camera preview`?")
    if not obs["geometry"].get("ok"):
        return None, f"caroiajul nu e găsit ({obs['reasons']})"
    jpg = urllib.request.urlopen(f"{preview}/snapshot.jpg", timeout=10).read()
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), 1)
    try:
        pts, rms = intersections(img, obs["geometry"]["corners_px"])
    except ValueError as e:
        return None, str(e)
    if rms > 1.0:
        return None, f"liniile nu sunt clare (abatere {rms:.2f} px): e un cub pe foaie?"
    return undist(pts.reshape(-1, 2)), None


def rigid(a, b):
    """b ≈ R a + t (Procrustes 2D)."""
    ma, mb = a.mean(0), b.mean(0)
    U, _, Vt = np.linalg.svd((a - ma).T @ (b - mb))
    R = (U @ Vt).T
    if np.linalg.det(R) < 0:
        R = R @ np.diag([1, -1])
    return R, mb - R @ ma


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pas", type=float, default=4.0, help="grade de rotire a bazei față de poză, de fiecare parte")
    ap.add_argument("--pasi", type=int, default=1, help="câte rotiri de fiecare parte (pas, 2 x pas, ...)")
    ap.add_argument("--preview", default="http://127.0.0.1:8080")
    ap.add_argument("--service", default="http://127.0.0.1:8765")
    a = ap.parse_args()
    pose = None
    views = {}
    try:
        post(a.service, "beep", {"n": 2})
        post(a.service, "go_pose", {"name": "home"})
        post(a.service, "go_pose", {"name": OBSERVE, "speed": "slow"})
        pose = json.load(open(ROOT / "calib.json"))["poses"][OBSERVE]
        time.sleep(1.0)
        for k in [0] + [s * i for i in range(1, a.pasi + 1) for s in (-1, 1)]:
            s1 = pose[0] + k * a.pas
            post(a.service, "move_joints", {"angles": [s1] + pose[1:], "time_ms": 1200})
            time.sleep(1.5)
            t0 = time.time()
            read = post(a.service, "read_joints")[:5]
            pts, err = snapshot(a.preview, t0)
            print(f"baza comandată {s1:g} (citită {read[0]}): " + (err or "16 intersecții"))
            if pts is not None:
                views[s1] = (pts, read[0])
            post(a.service, "beep", {"n": 1})
    finally:
        post(a.service, "go_pose", {"name": "home"})
        post(a.service, "go_pose", {"name": OBSERVE, "speed": "slow"})
    if pose is None or pose[0] not in views or len(views) < 2:
        sys.exit("Prea puține imagini bune: nimic de calculat.")
    pts0, _ = views[pose[0]]
    H0, _ = cv2.findHomography(GRID, pts0, 0)
    rows, rhs = [], []
    for s1, (pts, _) in sorted(views.items()):
        if s1 == pose[0]:
            continue
        # pixelul în care se vede acum intersecția (i, j) -> punctul de masă pe care îl vedea în poza de bază
        P0 = cv2.perspectiveTransform(pts.reshape(-1, 1, 2), np.linalg.inv(H0)).reshape(-1, 2)
        R, t = rigid(P0, GRID)
        th = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        res = np.linalg.norm(P0 @ R.T + t - GRID, axis=1)
        c = np.linalg.solve(np.eye(2) - R, t)
        print(f"  baza {s1 - pose[0]:+g} grd comandat: rotire măsurată {th:+.2f} grd (raport {abs(th / (s1 - pose[0])):.2f}), "
              f"centru ({c[0]:.0f}; {c[1]:.0f}) mm, abatere {np.sqrt((res ** 2).mean()):.2f} mm")
        rows.append(np.eye(2) - R)
        rhs.append(t)
    c = np.linalg.lstsq(np.vstack(rows), np.concatenate(rhs), rcond=None)[0]
    cell = np.array([1.5 * COL, 1.5 * ROW])
    print(f"Axa bazei, în mm pe foaie (x spre c2 de la marginea stângă, y spre robot de la marginea depărtată): "
          f"({c[0]:.1f}; {c[1]:.1f}), adică uv ({c[0] / COL:.2f}; {c[1] / ROW:.2f})")
    print(f"  axa bazei -> centrul r1c1: {np.linalg.norm(cell - c):.0f} mm; -> marginea foii dinspre robot: "
          f"{c[1] - 3 * ROW:.0f} mm")
    out = HERE / "axa_bazei.json"
    json.dump({"at": time.strftime("%Y-%m-%d %H:%M:%S"), "base_mm": c.tolist(), "col": COL, "row": ROW,
               "pose": pose}, open(out, "w"), indent=1)
    print(f"Salvat: {out}")


if __name__ == "__main__":
    main()
