"""Tur cu un cub pe caroiaj, prin serviciul HTTP: eroarea laterală a ghearei pe fiecare celulă, măsurată din cameră.

Degetele se închid lateral (perpendicular pe braț), deci la prindere cubul se centrează lateral între ele. Pus apoi pe
centrul celulei următoare, cubul ajunge cu eroarea laterală a ghearei pe celula aceea; camera o măsoară (după
home -> observe_small). Pe direcția brațului cubul păstrează abaterea de la prindere, deci acolo se vede doar
diferența dintre erorile celor două celule. Pe foaie trebuie să fie un singur cub.

    ai/.venv/bin/python ai/scripts/calib_live/autocal/tur_cub.py 00 02 22 20 11
"""
import json
import math
import sys
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from proba_pick_serviciu import LOCATION, ROOT, SERVICE, load_app_config, correct_parallax, observation, post  # noqa: E402

COL, ROW = 33.0, 33.0  # mm pe celulă: pătrate de 3,3 x 3,3 cm (rigla)
BASE_MM = (57.3, 309.5)  # axa bazei în mm pe foaie (axa_bazei.py 2026-10-29, cu celulele de mai sus)


def find_cube(obs):
    cells = [c for c in obs["cells"] if c["occupancy"] == "occupied" and c["uv_top"]]
    if len(cells) != 1:
        return None
    return cells[0]


def main():
    route = [(int(s[0]), int(s[1])) for s in sys.argv[1:]]
    loc = load_app_config(ROOT / "ai/config").system.location(LOCATION)
    cube = json.load(urllib.request.urlopen(f"{SERVICE}/status", timeout=10))["result"]["cube_mm"]
    base_uv = lambda c: correct_parallax(tuple(c["uv_top"]), cube, loc.camera_height_mm, tuple(loc.nadir_uv))
    rows = []
    out = HERE / "tur_cub.json"
    seen = find_cube(observation(0))
    if seen is None:
        sys.exit("Camera nu vede exact un cub pe foaie.")
    for r, c in route:
        src = base_uv(seen)
        dst = (c + 0.5, r + 0.5)
        print(f"r{seen['row']}c{seen['col']} uv ({src[0]:.3f}; {src[1]:.3f}) -> r{r}c{c}", flush=True)
        res = post("pick", {"location": LOCATION, "cell": [seen["row"], seen["col"]], "uv": list(src), "yaw_deg": seen["yaw_deg"] or 0.0})
        if res.get("result") is not True:
            sys.exit(f"pick refuzat: {res}")
        post("beep", {"n": 2})
        res = post("place", {"location": LOCATION, "cell": [r, c]})
        if res.get("result") is not True:
            sys.exit(f"place refuzat: {res} (cubul e încă în gheară)")
        post("go_pose", {"name": "home"})
        post("go_pose", {"name": f"observe_{LOCATION}", "speed": "slow"})
        time.sleep(1.0)
        seen = find_cube(observation(time.time()))
        if seen is None:
            sys.exit("După place camera nu mai vede exact un cub: opresc.")
        got = base_uv(seen)
        ex, ey = (got[0] - dst[0]) * COL, (got[1] - dst[1]) * ROW  # mm pe foaie: x spre c2, y spre robot
        tx, ty = dst[0] * COL - BASE_MM[0], dst[1] * ROW - BASE_MM[1]  # bază -> țintă
        n = math.hypot(tx, ty)
        radial = (ex * tx + ey * ty) / n  # + = departe de bază
        lateral = (ex * -ty + ey * tx) / n  # + = spre c2 (dreapta privind din spatele robotului)
        rows.append({"cell": f"r{r}c{c}", "target_uv": dst, "got_uv": got, "err_mm": [ex, ey], "lateral_mm": lateral,
                     "radial_mm": radial})
        print(f"  pus pe r{r}c{c}: uv ({got[0]:.3f}; {got[1]:.3f}), abatere {ex:+.1f} mm spre c2, {ey:+.1f} mm spre robot "
              f"-> lateral {lateral:+.1f} mm (+ = dreapta), pe direcția brațului {radial:+.1f} mm", flush=True)
        json.dump({"at": time.strftime("%Y-%m-%d %H:%M:%S"), "route": rows}, open(out, "w"), indent=1)  # după fiecare mutare
    print(f"Rezultate: {out}")


if __name__ == "__main__":
    main()
