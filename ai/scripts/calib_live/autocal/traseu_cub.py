"""Mută un cub pe rând prin celulele caroiajului, verificând totul din poza de start (observe_small).

Pentru fiecare mutare:
1. poza de start (home -> observe_small) și două analize la rând care văd cubul în același loc (±1 mm); liniile
   caroiajului măsurate sub-pixel (`grid_measure`) dau poziția reală a fiecărei celule;
2. pick prin serviciu (gheara din config.ts, la mijlocul cubului, coborâre lină), corectat cu abaterea laterală
   a ghearei învățată la mutările de dinainte;
3. înapoi în poza de start, cu cubul în gheară: celula de plecare trebuie să fie goală (altfel cubul n-a fost prins:
   oprire), iar centrul celulei-țintă se măsoară din liniile ei;
4. place pe centrul măsurat (corectat la fel), înapoi în poza de start și verificarea: cât rămâne, în mm, între
   cub și fiecare linie a celulei.
Degetele se închid lateral (perpendicular pe braț): cubul se centrează lateral în gheară, deci unde ajunge arată
abaterea laterală a ghearei pe celula aceea. Rezultatele se scriu după fiecare mutare în `traseu_cub.json`.

    ai/.venv/bin/python ai/scripts/calib_live/autocal/traseu_cub.py 01 02 10 11 12 20 21 22 01
"""
import calendar
import json
import math
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from proba_pick_serviciu import LOCATION, ROOT, SERVICE, correct_parallax, load_app_config, post  # noqa: E402
from brain.perception.grid_measure import measure_lines  # noqa: E402

PREVIEW = "http://127.0.0.1:8080"
COL, ROW = 33.0, 32.2  # mm pe celulă (rigla 3,3 x 3,3 cm; din imagine 33 x 32,2)
BASE_MM = (57.3, 309.5)  # axa bazei în mm pe foaie (axa_bazei.py, 2026-10-29)
UV = np.float32([[0, 0], [3, 0], [3, 3], [0, 3]])
# Abaterea laterală măsurată înainte (2026-10-29 ~22:35, pick/place pe r1c1 fără corecție): +9,8 mm spre dreapta.
PRIOR = [((1.5, 1.5), 9.8)]
OUT = HERE / "traseu_cub.json"


def say(msg):
    print(msg, flush=True)


def start_pose():
    post("go_pose", {"name": "home"})
    post("go_pose", {"name": f"observe_{LOCATION}", "speed": "slow"})
    time.sleep(1.0)


def analyses(n=2):
    """`n` analize făcute după ce brațul s-a oprit, fiecare cu imaginea ei (cadrul de după analiză)."""
    t0, out, last = time.time(), [], None
    while len(out) < n:
        if time.time() - t0 > 20:
            sys.exit("Previzualizarea nu dă analize noi: rulează `brain camera preview`?")
        time.sleep(0.2)
        obs = json.load(open(ROOT / "ai/out/preview/latest_observation.json"))
        stamp = calendar.timegm(time.strptime(obs["captured_at"], "%Y-%m-%dT%H:%M:%SZ"))
        if stamp < t0 or obs["captured_at"] == last:
            continue
        last = obs["captured_at"]
        jpg = urllib.request.urlopen(f"{PREVIEW}/snapshot.jpg", timeout=10).read()
        out.append((obs, cv2.imdecode(np.frombuffer(jpg, np.uint8), 1)))
    return out


def lines_uv(obs, img):
    """Intersecțiile măsurate (4 x 4, uv) în cadrul analizei; None unde o linie nu se vede."""
    pts = measure_lines(img, obs["geometry"]["corners_px"])["points"]
    H = cv2.getPerspectiveTransform(np.float32(obs["geometry"]["corners_px"]), UV)
    return [[None if p is None else cv2.perspectiveTransform(np.float64([[p]]), H).reshape(2) for p in row] for row in pts]


def cell_box(grid, r, c):
    """Liniile celulei în uv: stânga, dreapta, sus (departe), jos (spre robot); nominal unde lipsesc."""
    def avg(pts, axis, nominal):
        vals = [p[axis] for p in pts if p is not None]
        return float(np.mean(vals)) if vals else float(nominal)
    return (avg([grid[r][c], grid[r + 1][c]], 0, c), avg([grid[r][c + 1], grid[r + 1][c + 1]], 0, c + 1),
            avg([grid[r][c], grid[r][c + 1]], 1, r), avg([grid[r + 1][c], grid[r + 1][c + 1]], 1, r + 1))


class Cube:
    def __init__(self, loc, cube_mm):
        self.loc, self.mm = loc, cube_mm

    def seen(self, obs):
        cells = [c for c in obs["cells"] if c["occupancy"] == "occupied" and c["uv_top"]]
        return cells[0] if len(cells) == 1 else None

    def base_uv(self, cell):
        """Centrul bazei cubului (uv), corectat de paralaxă; fața de sus tăiată de marginea de sus a imaginii
        (rândul r0: se vede mai mică pe linii) se completează spre partea tăiată."""
        u, v = cell["uv_top"]
        H = self.loc.camera_height_mm
        side_h = cell["metrics"].get("side_h_mm")
        if side_h is not None and side_h < 0.85 * self.mm and v < 0.8:
            v -= (self.mm - side_h) / 2.0 * H / (H - self.mm) / ROW
        return correct_parallax((u, v), self.mm, H, tuple(self.loc.nadir_uv))

    def footprint_mm(self, uv, yaw_deg):
        """Colțurile bazei pe foaie, în mm (x spre c2, y spre robot)."""
        cx, cy, a, h = uv[0] * COL, uv[1] * ROW, math.radians(yaw_deg or 0.0), self.mm / 2.0
        d = [(h, h), (-h, h), (-h, -h), (h, -h)]
        return [(cx + x * math.cos(a) - y * math.sin(a), cy + x * math.sin(a) + y * math.cos(a)) for x, y in d]


def lateral_dir(uv):
    """Direcția laterală (spre dreapta, privind din spatele robotului) în punctul uv, ca vector unitar în mm."""
    tx, ty = uv[0] * COL - BASE_MM[0], uv[1] * ROW - BASE_MM[1]
    n = math.hypot(tx, ty)
    return -ty / n, tx / n


def predict(data, uv):
    """Abaterea laterală (mm) estimată în uv: plan e = a + b(u - 1,5) + c(v - 1,5) prin celulele măsurate.
    Caroiajul serviciului și foaia sunt ambele afine în uv, deci abaterea e aproape afină. Cu mai puțin de 3 celule,
    cea mai mică pantă care le potrivește (lstsq)."""
    if not data:
        return 0.0
    A = np.array([[1.0, p[0] - 1.5, p[1] - 1.5] for p, _ in data])
    k = np.linalg.lstsq(A, np.array([e for _, e in data]), rcond=None)[0]
    return float(k[0] + k[1] * (uv[0] - 1.5) + k[2] * (uv[1] - 1.5))


def corrected(uv, e_mm):
    """Punctul în care trebuie trimisă gheara ca să ajungă în uv, cu abaterea laterală e_mm."""
    lx, ly = lateral_dir(uv)
    return [uv[0] - e_mm * lx / COL, uv[1] - e_mm * ly / ROW]


def main():
    route = [(int(s[0]), int(s[1])) for s in sys.argv[1:]]
    loc = load_app_config(ROOT / "ai/config").system.location(LOCATION)
    cube = Cube(loc, json.load(urllib.request.urlopen(f"{SERVICE}/status", timeout=10))["result"]["cube_mm"])
    learned, log = [], []
    start_pose()
    for r, c in route:
        # 1. cubul și caroiajul, din poza de start
        (o1, _), (o2, img) = analyses(2)
        a, b = cube.seen(o1), cube.seen(o2)
        if a is None or b is None:
            sys.exit("Camera nu vede exact un cub pe foaie: opresc.")
        ua, ub = cube.base_uv(a), cube.base_uv(b)
        if math.hypot((ua[0] - ub[0]) * COL, (ua[1] - ub[1]) * ROW) > 1.5:
            (_, _), (o2, img) = analyses(2)
            b = cube.seen(o2)
            if b is None:
                sys.exit("Cubul nu se mai vede: opresc.")
            ub = cube.base_uv(b)
        src_cell, src = (b["row"], b["col"]), ub
        e_src = predict(learned, src)  # fără măsurători încă: 0 (degetele centrează cubul și cu ~10 mm abatere)
        pick_uv = corrected(src, e_src)
        say(f"r{src_cell[0]}c{src_cell[1]} -> r{r}c{c}: cubul la uv ({src[0]:.3f}; {src[1]:.3f}), yaw {b['yaw_deg']}; "
            f"pick cu corecția laterală {-e_src:+.1f} mm")
        # 2. pick
        res = post("pick", {"location": LOCATION, "uv": pick_uv, "yaw_deg": b["yaw_deg"] or 0.0})
        if res.get("result") is not True:
            sys.exit(f"pick refuzat: {res}")
        post("beep", {"n": 2})
        # 3. cu cubul în gheară, din poza de start: plecarea goală? unde e ținta?
        start_pose()
        (o3, img3), = analyses(1)
        if cube.seen(o3) is not None:
            post("beep", {"n": 4})
            post("open_gripper")
            sys.exit(f"Cubul e încă pe foaie (r{cube.seen(o3)['row']}c{cube.seen(o3)['col']}): pick ratat, opresc.")
        grid = lines_uv(o3, img3)
        left, right, top, bottom = cell_box(grid, r, c)
        target = ((left + right) / 2.0, (top + bottom) / 2.0)
        e_dst = predict(learned or PRIOR, target)
        place_uv = corrected(target, e_dst)
        say(f"  ținta r{r}c{c}: centrul măsurat uv ({target[0]:.3f}; {target[1]:.3f}), celula "
            f"{(right - left) * COL:.1f} x {(bottom - top) * ROW:.1f} mm; place cu corecția laterală {-e_dst:+.1f} mm")
        # 4. place și verificarea
        res = post("place", {"location": LOCATION, "uv": place_uv, "yaw_deg": 0.0})
        if res.get("result") is not True:
            sys.exit(f"place refuzat: {res} (cubul e încă în gheară)")
        start_pose()
        (o4, img4), = analyses(1)
        got = cube.seen(o4)
        if got is None:
            sys.exit("După place camera nu vede exact un cub: opresc.")
        landed = cube.base_uv(got)
        grid = lines_uv(o4, img4)
        left, right, top, bottom = cell_box(grid, r, c)
        fp = cube.footprint_mm(landed, got["yaw_deg"])
        margins = {"stânga": min(x for x, _ in fp) - left * COL, "dreapta": right * COL - max(x for x, _ in fp),
                   "sus": min(y for _, y in fp) - top * ROW, "jos": bottom * ROW - max(y for _, y in fp)}
        lx, ly = lateral_dir(target)
        ex, ey = (landed[0] - place_uv[0]) * COL, (landed[1] - place_uv[1]) * ROW  # față de unde a fost trimisă gheara
        e_true = ex * lx + ey * ly  # abaterea laterală a ghearei pe celula-țintă
        off = math.hypot((landed[0] - target[0]) * COL, (landed[1] - target[1]) * ROW)
        learned.append(((target[0], target[1]), e_true))
        inside = all(m >= 0 for m in margins.values())
        say(f"  pus: uv ({landed[0]:.3f}; {landed[1]:.3f}), {off:.1f} mm de centru, yaw {got['yaw_deg']}; până la linii: "
            + ", ".join(f"{k} {v:+.1f}" for k, v in margins.items()) + f" mm -> {'în celulă' if inside else 'DEPĂȘEȘTE'}; "
            f"abaterea laterală a ghearei aici {e_true:+.1f} mm")
        log.append({"cell": f"r{r}c{c}", "from": f"r{src_cell[0]}c{src_cell[1]}", "target_uv": target, "sent_uv": place_uv,
                    "landed_uv": landed, "off_mm": off, "margins_mm": margins, "lateral_mm": e_true, "inside": inside})
        json.dump({"at": time.strftime("%Y-%m-%d %H:%M:%S"), "moves": log}, open(OUT, "w"), indent=1, ensure_ascii=False)
        post("beep", {"n": 1})
    say(f"Gata: {sum(m['inside'] for m in log)}/{len(log)} mutări în celulă. Rezultate: {OUT}")


if __name__ == "__main__":
    main()
