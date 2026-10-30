"""Proba vederilor pentru zonele din afara caroiajului (sortarea, pașii 2-4): doar poze, nimic nu coboară pe foaie.

Din poza de start (observe_small, centrul r1c1) baza se rotește spre stânga și spre dreapta (camera se mută în lungul
rândului r1, spre marginea caroiajului și dincolo de ea), apoi brațul se strânge înapoi ca să privească spre robot
(zona de sub caroiaj). În fiecare vedere: imaginea brută (fără conturul cuburilor desenat de previzualizare), analiza
și unghiurile citite, în `vederi/[<proba>/]<nume>.jpg|json`.
Vârful ghearei rămâne tot timpul la peste 50 mm deasupra foii (cuburile au 29 mm).

    ai/.venv/bin/python ai/scripts/calib_live/autocal/vederi_zone.py caroiaj_gol
"""
import json
import sys
import time

import cv2

from traseu_cub import HERE, ROOT, analyses, post, say, start_pose

OUT = HERE / "vederi" / (sys.argv[1] if len(sys.argv) > 1 else "")
PREVIEW_DIR = ROOT / "ai/out/preview"
OBSERVE = [90, 90, 30.0, 2, 90]
# baza: + = spre c0 (stânga în imagine). 11 grd ~ 5 cm la r1c1 (marginea caroiajului), 20 grd ~ 9 cm
SIDES = [("stanga11", 11), ("stanga20", 20), ("dreapta11", -11), ("dreapta20", -20)]
# brațul strâns înapoi (umărul spre spate, cotul îndoit): după modelul camerei, fețele cuburilor de la ~20 cm, apoi
# ~19 și ~18 cm de axa bazei până la caroiaj; vârful ghearei la ~67, ~59 și ~55 mm deasupra foii
DOWN = [("jos1", [90, 105, 15, 0, 90]), ("jos2", [90, 110, 8, 0, 90]), ("jos3", [90, 112, 5, 0, 90])]


def shot(name, angles, time_ms):
    post("move_joints", {"angles": angles, "time_ms": time_ms})
    time.sleep(1.5)
    read = post("read_joints")["result"]
    for _ in range(5):  # cadrul brut al analizei: scris înaintea ei, deci verificat după
        (obs, _), = analyses(1)
        img = cv2.imread(str(PREVIEW_DIR / "latest_raw.jpg"))
        if json.load(open(PREVIEW_DIR / "latest_observation.json"))["captured_at"] == obs["captured_at"]:
            break
    OUT.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUT / f"{name}.jpg"), img)
    json.dump({"at": time.strftime("%Y-%m-%d %H:%M:%S"), "sent": angles, "read": read, "observation": obs},
              open(OUT / f"{name}.json", "w"), indent=1, ensure_ascii=False)
    say(f"{name}: comandat {angles}, citit {read[:5]}; caroiajul {'găsit' if obs['geometry'].get('ok') else 'negăsit'}")
    post("beep", {"n": 1})


def main():
    post("beep", {"n": 2})
    start_pose()
    shot("centru", OBSERVE, 1200)
    for name, d in SIDES:
        if name.endswith("11"):
            post("move_joints", {"angles": OBSERVE, "time_ms": 1200})  # mereu din centru, în aceeași parte
        shot(name, [OBSERVE[0] + d] + OBSERVE[1:], 1200)
    post("move_joints", {"angles": OBSERVE, "time_ms": 1200})
    for name, angles in DOWN:
        shot(name, angles, 2000)
    for _, angles in reversed(DOWN[:-1]):
        post("move_joints", {"angles": angles, "time_ms": 1500})
    post("move_joints", {"angles": OBSERVE, "time_ms": 2000})
    start_pose()
    say(f"Gata. Imaginile: {OUT}")


if __name__ == "__main__":
    main()
