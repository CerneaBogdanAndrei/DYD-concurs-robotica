"""Pick pe o celulă prin serviciul HTTP (gheara din config.ts), la poziția văzută de cameră, apoi place pe loc.

Ca brain-ul: uv-ul feței de sus din ultima analiză, corectat de paralaxă (`correct_parallax`, cu înălțimea camerei și
nadirul din `system.yaml`). După pick: 2 bipuri și place pe aceeași poziție, mereu. Servo 6 nu spune dacă gheara ține
cubul: cu cubul de 29 mm prins citește 133 la comanda 135 (2026-10-29), aproape cât goală. Verificarea e din cameră:
la final home -> observe_small și ce vede camera pe celulă.

    ai/.venv/bin/python ai/scripts/calib_live/autocal/proba_pick_serviciu.py 1 1
"""
import calendar
import json
import sys
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT / "ai"))
from brain.configuration.loader import load_app_config  # noqa: E402
from brain.domain.plan import correct_parallax  # noqa: E402

SERVICE = "http://127.0.0.1:8765"
LOCATION = "small"


def post(method, body=None):
    req = urllib.request.Request(f"{SERVICE}/{method}", data=json.dumps(body or {}).encode(), method="POST")
    return json.load(urllib.request.urlopen(req, timeout=60))


def observation(after):
    for _ in range(60):
        obs = json.load(open(ROOT / "ai/out/preview/latest_observation.json"))
        if calendar.timegm(time.strptime(obs["captured_at"], "%Y-%m-%dT%H:%M:%SZ")) >= after:
            return obs
        time.sleep(0.25)
    sys.exit("Previzualizarea nu a dat o analiză nouă: rulează `brain camera preview`?")


def cell_of(obs, r, c):
    return next(x for x in obs["cells"] if x["row"] == r and x["col"] == c)


def main():
    r, c = int(sys.argv[1]), int(sys.argv[2])
    loc = load_app_config(ROOT / "ai/config").system.location(LOCATION)
    cube = json.load(urllib.request.urlopen(f"{SERVICE}/status", timeout=10))["result"]["cube_mm"]
    seen = cell_of(observation(0), r, c)
    if seen["occupancy"] != "occupied" or not seen["uv_top"]:
        sys.exit(f"Camera nu vede un cub pe r{r}c{c}: {seen['occupancy']} {seen['reasons']}")
    uv = correct_parallax(tuple(seen["uv_top"]), cube, loc.camera_height_mm, tuple(loc.nadir_uv))
    yaw = seen["yaw_deg"] or 0.0
    target = {"location": LOCATION, "cell": [r, c], "uv": [round(uv[0], 4), round(uv[1], 4)], "yaw_deg": round(yaw, 2)}
    print(f"cub {seen['color']} pe r{r}c{c}: fața de sus uv {seen['uv_top']}, baza uv ({uv[0]:.3f}; {uv[1]:.3f}), yaw {yaw:.1f} grd")
    t0 = time.time()
    print("pick:", post("pick", target))
    grip = post("read_joints")["result"][5]
    closed = json.load(urllib.request.urlopen(f"{SERVICE}/status", timeout=10))["result"]["joints"][5]
    print(f"gheara citită {grip} (comandată {closed}), {time.time() - t0:.1f} s")
    post("beep", {"n": 2})
    print("place:", post("place", target))
    post("go_pose", {"name": "home"})
    post("go_pose", {"name": f"observe_{LOCATION}", "speed": "slow"})
    time.sleep(1.0)
    after = cell_of(observation(time.time()), r, c)
    print(f"camera după: r{r}c{c} {after['occupancy']} {after['color']}, fața de sus uv {after['uv_top']} (înainte {seen['uv_top']}), "
          f"yaw {after['yaw_deg']}")


if __name__ == "__main__":
    main()
