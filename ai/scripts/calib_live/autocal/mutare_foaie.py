"""Foaia s-a mutat: recalibrează caroiajul `small` din cameră, fără să miști brațul cu mâna.

Serviciul de mișcare (`npm run server`) și previzualizarea (`brain camera preview`) rulează; foaia e goală.
Brațul e dus home -> observe_small înainte de măsurare: venind mereu din aceeași parte, servo-urile revin la
aceleași unghiuri (6 reveniri: citire identică, mutare măsurată ±0,2 mm; din altă parte: până la 7 mm eroare).
Se compară cele 16 intersecții ale caroiajului de acum cu cele din `ref/` (făcute din aceeași poză, când
`calib.json` era corect). Diferența dintre cele două imagini e chiar mutarea foii pe masă (translație + rotire),
fără modelul camerei: omografia de referință duce punctele de acum în milimetrii foii vechi. Mutarea se aplică
peste modelul învățat al caroiajului (care mergea fizic la nivelul 0), iar imaginea de acum devine noua referință.

    ai/.venv/bin/python ai/scripts/calib_live/autocal/mutare_foaie.py            # doar măsoară
    ai/.venv/bin/python ai/scripts/calib_live/autocal/mutare_foaie.py --aplica   # + scrie calib.json și noua referință

Precizia: ~2 mm cu aceleași unghiuri citite ca la referință; 1 grd diferență = ~10 px = 3-7 mm. Mutări mari (> ~1 celulă) sau rotiri mari
cer recalibrarea cu mâna (`npm run calib -- celule small`): modelul învățat ar fi extrapolat prea departe.
"""
import argparse
import json
import shutil
import subprocess
import sys
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE))
from detect import intersections  # noqa: E402

# Obiectivul, estimat pe 2026-10-29 din 9 vederi ale caroiajului (f 738±44 px, k1 -0.21±0.06; ~50 grd pe orizontală).
F, C, K1 = 740.0, (320.0, 240.0), -0.21
COL, ROW = 45.0, 43.0
GRID = np.array([[COL * i, ROW * j] for j in range(4) for i in range(4)])


def undist(p):
    K = np.array([[F, 0, C[0]], [0, F, C[1]], [0, 0, 1]])
    return cv2.undistortPoints(p.reshape(-1, 1, 2), K, np.array([K1, 0, 0, 0, 0.0]), P=K).reshape(-1, 2)


def post(service, method, body=None):
    req = urllib.request.Request(f"{service}/{method}", data=json.dumps(body or {}).encode(), method="POST")
    return json.load(urllib.request.urlopen(req, timeout=30))["result"]


def capture(service, preview):
    post(service, "go_pose", {"name": "home"})
    post(service, "go_pose", {"name": "observe_small", "speed": "slow"})
    time.sleep(1.0)
    t0 = time.time()
    joints = post(service, "read_joints")[:5]
    for _ in range(40):  # o analiză făcută după oprirea brațului
        time.sleep(0.25)
        obs = json.load(open(ROOT / "ai/out/preview/latest_observation.json"))
        stamp = datetime.strptime(obs["captured_at"], "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc).timestamp()
        if stamp >= t0:
            break
    else:
        sys.exit("Previzualizarea nu a dat o analiză nouă: rulează `brain camera preview`?")
    if not obs["actionable"]:
        sys.exit(f"Caroiajul nu e văzut complet: {obs['reasons']} (foaia e goală? e în câmpul camerei?)")
    jpg = urllib.request.urlopen(f"{preview}/snapshot.jpg", timeout=10).read()
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), 1)
    pts, rms = intersections(img, obs["geometry"]["corners_px"])
    if rms > 1.0:
        sys.exit(f"Liniile caroiajului nu sunt clare (abatere {rms:.2f} px): scoateți cuburile de pe foaie.")
    obs["read_joints"] = joints
    return img, obs, pts.reshape(-1, 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--aplica", action="store_true", help="scrie calib.json și imaginea de acum ca referință")
    ap.add_argument("--preview", default="http://127.0.0.1:8080")
    ap.add_argument("--service", default="http://127.0.0.1:8765")
    a = ap.parse_args()
    ref_img, ref_json = HERE / "ref/observe_small.jpg", HERE / "ref/observe_small.json"
    img, obs, now = capture(a.service, a.preview)
    ref_obs = json.load(open(ref_json))
    if obs["read_joints"] != ref_obs.get("read_joints"):
        print(f"ATENȚIE: unghiuri citite {obs['read_joints']}, la referință {ref_obs.get('read_joints')}: "
              "rezultatul poate avea câțiva mm eroare.")
    ref, _ = intersections(cv2.imread(str(ref_img)), ref_obs["geometry"]["corners_px"])
    H_ref, _ = cv2.findHomography(GRID, undist(ref.reshape(-1, 2)), 0)
    X = cv2.perspectiveTransform(undist(now).reshape(-1, 1, 2), np.linalg.inv(H_ref)).reshape(-1, 2)
    mc, xc = GRID.mean(0), X.mean(0)
    U, _, Vt = np.linalg.svd((GRID - mc).T @ (X - xc))
    R = (U @ Vt).T
    if np.linalg.det(R) < 0:
        R = R @ np.diag([1, -1])
    t = xc - R @ mc
    res = np.linalg.norm(GRID @ R.T + t - X, axis=1)
    rot = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    print(f"Foaia s-a mutat cu {t[0]:+.1f} mm pe coloane (+ = spre c2), {t[1]:+.1f} mm pe linii (+ = spre robot), "
          f"rotire {rot:+.2f} grd; abatere RMS {np.sqrt((res ** 2).mean()):.1f} mm, max {res.max():.1f} mm")
    if res.max() > 6:
        sys.exit("Abatere prea mare: brațul nu e în aceeași poză ca la referință sau caroiajul e detectat greșit.")
    if np.hypot(*t) > 45 or abs(rot) > 8:
        sys.exit("Mutare prea mare pentru corecția din cameră: recalibrați cu mâna (npm run calib -- celule small).")
    if not a.aplica:
        print("Nimic scris (rulați cu --aplica).")
        return
    shift = HERE / "ultima_mutare.json"
    json.dump({"R": R.tolist(), "t": t.tolist()}, open(shift, "w"))
    subprocess.run(["node", str(HERE / "aplica_mutarea.mjs"), str(shift)], cwd=ROOT, check=True)
    shutil.copy(ref_img, HERE / "ref/observe_small.prev.jpg")
    shutil.copy(ref_json, HERE / "ref/observe_small.prev.json")
    cv2.imwrite(str(ref_img), img)
    json.dump(obs, open(ref_json, "w"), indent=1)
    print("Noua referință: imaginea de acum. Verificați cu: node ai/scripts/calib_live/autocal/tur_celule.mjs")


if __name__ == "__main__":
    main()
