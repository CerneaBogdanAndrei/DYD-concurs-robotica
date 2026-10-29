"""Cât rotește și cât mută cubul o rotire a încheieturii (servo 5) la punere, pe aceeași celulă.

Gheara e înclinată (-40..-60 grd), deci o rotire a încheieturii rotește cubul pe foaie doar parțial și îl și
înclină. Cubul stă pe celula dată; pentru fiecare rotire comandată (implicit 0, +10, -10, 0 grd): pick unde e,
place pe centrul celulei cu rotirea aceea, poza de start și măsurarea (rotirea cubului, mutarea față de centru).

    ai/.venv/bin/python ai/scripts/calib_live/autocal/proba_rotire.py 1 1 [0 10 -10 0]
"""
import json
import sys
import time

import numpy as np

from traseu_cub import COL, HERE, LOCATION, ROW, Cube, cell_box, grab, lines_uv, load_app_config, look, post, ROOT, \
    start_pose, status, wrap45

OUT = HERE / "proba_rotire.json"


def main():
    r, c = int(sys.argv[1]), int(sys.argv[2])
    yaws = [float(v) for v in sys.argv[3:]] or [0.0, 10.0, -10.0, 0.0]
    loc = load_app_config(ROOT / "ai/config").system.location(LOCATION)
    cube = Cube(loc, status()["cube_mm"])
    start_pose()
    found = look(cube)
    if found is None:
        sys.exit("Camera nu vede bine cubul: opresc.")
    seen, uv, obs, img = found
    left, right, top, bottom = cell_box(lines_uv(obs, img), r, c)
    target = ((left + right) / 2.0, (top + bottom) / 2.0)
    log = []
    for yaw_cmd in yaws:
        before = seen["yaw_deg"]
        ok, diag, _, _, _ = grab(cube, seen, uv, [])
        print(f"pick (cubul rotit {before:+.1f} grd): {diag} -> {'prins' if ok else 'CUBUL E ÎNCĂ PE FOAIE'}", flush=True)
        if not ok:
            post("beep", {"n": 4})
            if look(cube) is not None:
                post("open_gripper")
            sys.exit("Pick ratat: opresc.")
        res = post("place", {"location": LOCATION, "uv": list(target), "yaw_deg": yaw_cmd})
        if res.get("result") is not True:
            sys.exit(f"place refuzat: {res} (cubul e încă în gheară)")
        start_pose()
        found = look(cube)
        if found is None:
            sys.exit("După place camera nu vede bine cubul: opresc.")
        seen, uv, _, _ = found
        dx, dy = (uv[0] - target[0]) * COL, (uv[1] - target[1]) * ROW
        print(f"  rotire comandată {yaw_cmd:+.0f} grd -> cubul rotit {seen['yaw_deg']:+.1f} grd; față de centru "
              f"{dx:+.1f} mm pe coloane, {dy:+.1f} mm spre robot", flush=True)
        log.append({"yaw_cmd": yaw_cmd, "yaw_before": before, "yaw": seen["yaw_deg"], "dx_mm": dx, "dy_mm": dy,
                    "turn": wrap45(seen["yaw_deg"] - before)})
        json.dump({"at": time.strftime("%Y-%m-%d %H:%M:%S"), "cell": [r, c], "runs": log}, open(OUT, "w"), indent=1)
        post("beep", {"n": 1})
    A = np.array([[1.0, m["yaw_cmd"]] for m in log])
    k = np.linalg.lstsq(A, np.array([m["yaw"] for m in log]), rcond=None)[0]
    print(f"Rotirea cubului ≈ {k[0]:+.1f} + {k[1]:.2f} x comanda (grd). Rezultate: {OUT}")


if __name__ == "__main__":
    main()
