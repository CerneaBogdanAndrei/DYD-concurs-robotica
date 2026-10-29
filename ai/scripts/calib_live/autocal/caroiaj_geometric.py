"""Caroiajul `small` din geometrie, fără colțuri învățate cu mâna.

- axa bazei în mm pe foaie: `axa_bazei.json` (din `axa_bazei.py`, rotiri ale bazei văzute de cameră);
- direcția brațului (x al robotului, servo 1 = 90): de la axa bazei spre punctul de sub cameră în poza de observare
  (PnP pe cele 16 intersecții, f și K1 ca în `mutare_foaie.py`), plus `--rotire` grade (+ = spre stânga);
- centrele celulelor: liniile foii (pătrate de 33 mm), duse în sistemul robotului (x înainte, y la stânga);
- înălțimea de prindere: planul colțurilor învățate înainte (z-ul modelului depinde de poziția brațului, nu de foaie),
  evaluat în noile poziții.
`--lateral E` (mm, + = gheara ajunge la dreapta) rotește direcția astfel încât abaterea laterală E măsurată pe r1c1
să dispară. `--radial R --dz D` mută punctul de prindere R mm mai departe de bază și D mm mai sus (unde apucă degetele
cubul: aceeași mutare la pick și la place, deci cubul ajunge tot în centrul celulei). Cu `--aplica` scrie colțurile
în calib.json (copie `calib.json.bak-<oră>`). Foaia trebuie să fie goală; cu `--camera-salvata` se folosește camera
măsurată ultima dată (`camera_observare.json`, aceeași poză de observare), iar foaia poate avea cuburi.

    ai/.venv/bin/python ai/scripts/calib_live/autocal/caroiaj_geometric.py [--rotire GRD] [--lateral MM]
        [--radial MM] [--dz MM] [--camera-salvata] [--aplica]
"""
import argparse
import json
import math
import shutil
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
from mutare_foaie import C as PP, COL, F, GRID, K1, ROW  # noqa: E402

CELLS = ("r0c0", "r0c2", "r2c0", "r2c2")
CAMERA = HERE / "camera_observare.json"


def camera_now():
    """Punctul de sub cameră (mm pe foaie) și înălțimea ei, din imaginea de acum."""
    obs = json.load(open(ROOT / "ai/out/preview/latest_observation.json"))
    img = cv2.imdecode(np.frombuffer(urllib.request.urlopen("http://127.0.0.1:8080/snapshot.jpg", timeout=10).read(),
                                     np.uint8), 1)
    pts, rms = intersections(img, obs["geometry"]["corners_px"])
    if rms > 1.0:
        sys.exit(f"Liniile nu sunt clare (abatere {rms:.2f} px): e un cub pe foaie?")
    K = np.array([[F, 0, PP[0]], [0, F, PP[1]], [0, 0, 1.0]])
    obj = np.c_[GRID, np.zeros(16)]
    _, rv, tv = cv2.solvePnP(obj, pts.reshape(-1, 2), K, np.array([K1, 0, 0, 0, 0.0]))
    R, _ = cv2.Rodrigues(rv)
    cam = (-R.T @ tv).ravel()
    json.dump({"at": time.strftime("%Y-%m-%d %H:%M:%S"), "nadir_mm": cam[:2].tolist(), "height_mm": abs(cam[2]),
               "nadir_uv": [cam[0] / COL, cam[1] / ROW],
               "pose": json.load(open(ROOT / "calib.json"))["poses"]["observe_small"]}, open(CAMERA, "w"), indent=1)
    return cam[:2], abs(cam[2])


def camera_saved():
    cam = json.load(open(CAMERA))
    if cam["pose"] != json.load(open(ROOT / "calib.json"))["poses"]["observe_small"]:
        sys.exit("camera_observare.json e din altă poză de observare: rulați fără --camera-salvata, cu foaia goală.")
    return np.array(cam["nadir_uv"]) * (COL, ROW), cam["height_mm"]


def old_z_plane(points):
    """z = a + b·x + c·y prin punctele învățate cu mâna (coordonatele robotului). Planul se păstrează în `plan_z.json`
    la prima rulare: după --aplica, colțurile din calib.json au deja --dz și nu mai sunt cele învățate."""
    path = HERE / "plan_z.json"
    if path.exists():
        k = json.load(open(path))["k"]
    else:
        A = np.array([[1.0, p[0], p[1]] for p in points.values()])
        k = np.linalg.lstsq(A, np.array([p[2] for p in points.values()]), rcond=None)[0].tolist()
        json.dump({"at": time.strftime("%Y-%m-%d %H:%M:%S"), "k": k}, open(path, "w"), indent=1)
    return lambda x, y: float(k[0] + k[1] * x + k[2] * y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rotire", type=float, default=0.0, help="grade în plus pentru direcția brațului (+ = stânga)")
    ap.add_argument("--lateral", type=float, default=None, help="abaterea laterală măsurată pe r1c1, mm (+ = dreapta)")
    ap.add_argument("--radial", type=float, default=0.0, help="punctul de prindere mai departe de bază, mm")
    ap.add_argument("--dz", type=float, default=0.0, help="punctul de prindere mai sus, mm")
    ap.add_argument("--camera-salvata", action="store_true", help="camera din camera_observare.json (cuburi pe foaie)")
    ap.add_argument("--aplica", action="store_true")
    a = ap.parse_args()
    axis = json.load(open(HERE / "axa_bazei.json"))
    B = np.array(axis["base_uv"]) * (COL, ROW)  # în celule: nu depinde de mărimea celulei folosită la măsurare
    nadir, H = camera_saved() if a.camera_salvata else camera_now()
    f = (nadir - B) / np.linalg.norm(nadir - B)
    rot = math.radians(a.rotire)
    if a.lateral is not None:
        r11 = np.linalg.norm(np.array([1.5 * COL, 1.5 * ROW]) - B)
        rot -= a.lateral / r11  # gheara ajunge la dreapta: axa reală e rotită spre dreapta față de cea presupusă
    # pe foaie x e spre dreapta, deci o rotire spre stânga (privind din spatele robotului) e în sensul acelor
    c, s = math.cos(rot), math.sin(rot)
    f = np.array([c * f[0] + s * f[1], -s * f[0] + c * f[1]])
    left = np.array([f[1], -f[0]])
    calib_path = ROOT / "calib.json"
    calib = json.load(open(calib_path))
    z_of = old_z_plane(calib["sites"]["small"]["points"])
    print(f"axa bazei ({B[0]:.1f}; {B[1]:.1f}) mm pe foaie; camera: H {H:.0f} mm, nadir uv ({nadir[0] / COL:.2f}; "
          f"{nadir[1] / ROW:.2f}); direcția brațului {math.degrees(math.atan2(-f[0], -f[1])):+.2f} grd față de coloane; "
          f"prinderea cu {a.radial:+.1f} mm mai departe și {a.dz:+.1f} mm mai sus")
    pts = {}
    for r in range(3):
        for col in range(3):
            d = np.array([COL * (col + 0.5), ROW * (r + 0.5)]) - B
            x, y = float(d @ f), float(d @ left)
            x, y = x + a.radial * x / math.hypot(x, y), y + a.radial * y / math.hypot(x, y)
            z = z_of(x, y) + a.dz  # planul învățat e al brațului: evaluat unde merge gheara
            label = f"r{r}c{col}"
            if label in CELLS:
                pts[label] = [x, y, z]
            print(f"  {label}: x {x:6.1f}  y {y:+6.1f}  z {z:5.1f}  (distanța {math.hypot(x, y):.0f} mm)")
    old = calib["sites"]["small"]["points"]
    for k in CELLS:
        print(f"  {k}: față de colțul învățat {math.hypot(pts[k][0] - old[k][0], pts[k][1] - old[k][1]):.1f} mm")
    if a.aplica:
        shutil.copy(calib_path, calib_path.with_name("calib.json.bak-" + time.strftime("%H%M%S")))
        calib["sites"]["small"] = {"kind": "grid", "points": pts}
        json.dump(calib, open(calib_path, "w"), indent=2)
        print("Scris în calib.json (small); copia veche: calib.json.bak-*")


if __name__ == "__main__":
    main()
