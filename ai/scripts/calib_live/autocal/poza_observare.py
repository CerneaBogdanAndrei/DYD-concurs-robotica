"""Caută poza de observare în care camera vede tot caroiajul, centrat pe verticală, schimbând doar cotul (servo 3).

Pornește de la `observe_small` din calib.json și încearcă servo 3 în pași (brațul rămâne sus, celelalte servo-uri
neschimbate; servo 4 e deja aproape de capătul cursei). Pentru fiecare: home -> poza, analiza camerei, marginile
caroiajului față de marginile imaginii. Alege poza cu marginile sus/jos cele mai echilibrate (minimum 25 px).
Cu `--aplica` scrie poza în calib.json (copie de siguranță `calib.json.bak-<oră>`). Foaia trebuie să fie goală.

    ai/.venv/bin/python ai/scripts/calib_live/autocal/poza_observare.py [--cot 29 26 23 20 17] [--aplica]
"""
import argparse
import calendar
import json
import shutil
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE))
from mutare_foaie import post  # noqa: E402

SERVICE = "http://127.0.0.1:8765"
POSE = "observe_small"


def fresh_observation(t0):
    for _ in range(60):
        time.sleep(0.25)
        obs = json.load(open(ROOT / "ai/out/preview/latest_observation.json"))
        if calendar.timegm(time.strptime(obs["captured_at"], "%Y-%m-%dT%H:%M:%SZ")) >= t0:
            return obs
    sys.exit("Previzualizarea nu dă analize noi: rulează `brain camera preview`?")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cot", type=float, nargs="+", default=[29, 26, 23, 20, 17])
    ap.add_argument("--aplica", action="store_true")
    a = ap.parse_args()
    calib_path = ROOT / "calib.json"
    calib = json.load(open(calib_path))
    base = list(calib["poses"][POSE])
    results = []
    try:
        for s3 in a.cot:
            pose = base[:2] + [s3] + base[3:]
            post(SERVICE, "go_pose", {"name": "home"})
            post(SERVICE, "move_joints", {"angles": pose, "time_ms": 1800})
            time.sleep(1.2)
            obs = fresh_observation(time.time())
            corners = obs["geometry"].get("corners_px") or []
            if not obs["geometry"].get("ok") or len(corners) != 4:
                print(f"cot {s3:g}: caroiajul nu e găsit ({obs['reasons']})", flush=True)
                continue
            ys = [p[1] for p in corners]
            xs = [p[0] for p in corners]
            top, bottom, left, right = min(ys), 480 - max(ys), min(xs), 640 - max(xs)
            ok = obs["actionable"] and min(top, bottom, left, right) >= 25
            print(f"cot {s3:g}: margini sus {top:.0f}, jos {bottom:.0f}, stânga {left:.0f}, dreapta {right:.0f} px"
                  f"{'' if ok else ' (prea aproape de margine)'}", flush=True)
            if ok:
                results.append((abs(top - bottom), s3, pose))
            post(SERVICE, "beep", {"n": 1})
    finally:
        post(SERVICE, "go_pose", {"name": "home"})
    if not results:
        sys.exit("Nicio poză nu vede tot caroiajul cu margine: mutați foaia sau încercați alte valori (--cot).")
    _, s3, pose = min(results)
    print(f"Cea mai bună: cot {s3:g} -> {pose}")
    if a.aplica:
        shutil.copy(calib_path, calib_path.with_name("calib.json.bak-" + time.strftime("%H%M%S")))
        calib["poses"][POSE] = pose
        json.dump(calib, open(calib_path, "w"), indent=2)
        print(f"Scris în calib.json ({POSE}); copia veche: calib.json.bak-*")
    post(SERVICE, "move_joints", {"angles": pose if a.aplica else base, "time_ms": 1800})


if __name__ == "__main__":
    main()
