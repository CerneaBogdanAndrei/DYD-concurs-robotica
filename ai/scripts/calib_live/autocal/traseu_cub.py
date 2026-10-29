"""Mută un cub pe rând prin celulele caroiajului, verificând totul din poza de start (observe_small).

Pentru fiecare mutare:
1. poza de start (home -> observe_small) și două analize la rând care văd cubul în același loc (±1,5 mm), cu
   rotirea lui. Liniile celulelor sunt cele salvate din imaginea de referință (foaia goală, aceeași poză:
   `ref/observe_small.jpg`, măsurate sub-pixel cu `grid_measure`): foaia nu se mișcă, iar cubul (pe foaie sau în
   gheară, jos în imagine peste r2) ar acoperi liniile. Pozițiile din fiecare analiză se duc în uv-ul referinței;
2. pick prin serviciu (gheara din config.ts, la mijlocul cubului, coborâre lină), corectat cu abaterea laterală
   a ghearei învățată la mutările de dinainte; serviciul dă unghiurile citite jos, înainte și după închidere;
3. prins = gheara citită strânge ceva (140 comandat: goală 139, cu cubul ~133-136) și, din poza de start, nicio
   celulă ocupată (cubul din gheară se poate vedea jos, peste r2, ca „nesigur”). Gheara goală: 4 bipuri, gheara se
   deschide, cubul e măsurat din nou și pick-ul se reia o dată;
4. place pe centrul celulei-țintă din liniile salvate (corectat pe ambele axe), înapoi în poza de start și
   verificarea: cât rămâne, în mm, între cub și fiecare linie a celulei, și rotirea cubului;
5. dacă cubul e la peste GOOD_OFF_MM de centru, intră pe linii mai mult de jumătate din grosimea lor sau e rotit
   peste YAW_TOL: îl ia din nou de unde a ajuns (degetele aliniate cu el) și îl pune corectat cu ce s-a măsurat chiar
   acolo: poziția cu abaterea ultimei puneri, rotirea încheieturii cu pasul -rotire / k (de cel mult MAX_FIXES ori).
Cum stă cubul în gheară: degetele se închid lateral (perpendicular pe braț), deci cubul se centrează lateral; unde
ajunge arată abaterea laterală a ghearei (folosită la pick). La punere se învață abaterea întreagă (pe ambele axe,
punere - trimitere), pe fiecare celulă: pe 2026-10-29 cubul a ajuns cu ~4 mm spre robot pe r1c1 și ~6 mm pe r0c0.
Rotirea: cu gheara înclinată, încheietura rotește cubul pe foaie doar cu o parte k din comandă (`proba_rotire.py` pe
r1c1, cu eliberarea de dinainte, de sus: k ~0,23) și îl mută și lateral (-4,3 mm la -10 grd). k se învață din punerile
pe aceeași celulă cu comenzi diferite. Cubul păstrează cam rotirea brațului: pe c0 și c2 baza e rotită ~7 grd.
Rezultatele se scriu după fiecare punere în `traseu_cub.json`.

    ai/.venv/bin/python ai/scripts/calib_live/autocal/traseu_cub.py 00 01 02 12 22 21 20 10 11
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
COL, ROW = 33.0, 33.0  # mm pe celulă: pătrate de 3,3 x 3,3 cm (rigla; 32,2 din imagine era eroarea modelului camerei)
BASE_MM = tuple(np.array(json.load(open(HERE / "axa_bazei.json"))["base_uv"]) * (COL, ROW))  # axa bazei pe foaie (axa_bazei.py)
UV = np.float32([[0, 0], [3, 0], [3, 3], [0, 3]])
REF = HERE / "ref/observe_small"  # imaginea foii goale din poza de start (mutare_foaie.py --ref-nou)
POSE_PX = 6.0  # colțurile mai departe de atât față de referință (~1,6 mm): poza de start nu s-a repetat (de obicei 1-3 px)
GRIP_EMPTY = 2  # gheara citită la cel mult atât sub comandă = goală
MAX_FIX_MM = 8.0  # corecția nu trece de atât: peste, e mai probabil o măsurătoare greșită
K_WRIST = 0.5  # cât din rotirea comandată a cubului se vede pe foaie, la început: 0,54 pe r1c1 cu eliberarea de
# acum (2026-10-30; 0,23 cu eliberarea de sus); geometric ~0,77 la gheara înclinată -50 grd
K_RANGE = (0.3, 1.0)  # k învățat rămâne între acestea
YAW_STEP = 0.7  # din rotirea calculată se comandă doar atât: punerea are ~2 grd zgomot, un pas întreg ar oscila
MAX_FIX_YAW = 25.0  # rotirea comandată a încheieturii, cel mult (grd)
GOOD_OFF_MM = 2.0  # cubul bine pus: centrul lui la cel mult atât de centrul celulei
GOOD_MARGIN_MM = -0.4  # ... intră pe linii (0,8 mm) cel mult jumătate din grosimea lor
YAW_TOL = 3.0  # ... și e rotit cel mult atât. Camera: ±0,5 grd pe un cub care stă (6 analize); punerea însă rotește
# cubul la întâmplare cu ~±2 grd (r1c1, 2026-10-30: +0,6, +2,5 cu încheietura la 0), deci sub ~3 grd o nouă punere
# nu îl îndreaptă, doar îl rotește altfel
# Abaterea de punere de la început (punere - trimitere, mm: x spre c2, y spre robot): r1c1, 2026-10-30 ~00:15, cu
# eliberarea de acum (coboară până atinge foaia): 2 puneri cu încheietura la 0. Cu eliberarea de sus cubul ajungea
# cu 4-6 mm spre robot; echipa măsurase 6 mm, corecție care acum îl ducea cu 7 mm prea departe.
PRIOR_XY = [((1.5, 1.5), (-2.4, -0.6))]
MAX_FIXES = 3
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


class Sheet:
    """Caroiajul salvat din imaginea de referință: colțurile (pixeli) și cele 16 intersecții (uv)."""

    def __init__(self):
        obs = json.load(open(REF.with_suffix(".json")))
        self.corners = np.float32(obs["geometry"]["corners_px"])
        self.H = cv2.getPerspectiveTransform(self.corners, UV)
        pts = measure_lines(cv2.imread(str(REF.with_suffix(".jpg"))), obs["geometry"]["corners_px"])["points"]
        if pts is None or any(p is None for row in pts for p in row):
            sys.exit("În imaginea de referință nu se văd toate liniile: refaceți-o cu foaia goală (mutare_foaie.py --ref-nou).")
        self.lines = [[cv2.perspectiveTransform(np.float64([[p]]), self.H).reshape(2) for p in row] for row in pts]
        self.warned = False

    def to_ref(self, obs, uv):
        """uv din analiza `obs` -> uv-ul referinței, prin pixeli. Dacă poza de start nu s-a repetat (colțurile s-au
        mutat), rămâne uv-ul analizei: e tot pe foaie, doar cu colțurile de acum."""
        c = np.float32(obs["geometry"]["corners_px"])
        moved = float(np.abs(c - self.corners).max())
        if moved > POSE_PX:
            if not self.warned:
                say(f"  ATENȚIE: colțurile caroiajului sunt la {moved:.1f} px de referință (poza sau foaia s-a mișcat)")
                self.warned = True
            return tuple(uv)
        px = cv2.perspectiveTransform(np.float64([[uv]]), np.linalg.inv(cv2.getPerspectiveTransform(c, UV)))
        return tuple(cv2.perspectiveTransform(px, self.H).reshape(2))

    def box(self, r, c):
        """Liniile celulei în uv: stânga, dreapta, sus (departe), jos (spre robot)."""
        g = self.lines
        return ((g[r][c][0] + g[r + 1][c][0]) / 2, (g[r][c + 1][0] + g[r + 1][c + 1][0]) / 2,
                (g[r][c][1] + g[r][c + 1][1]) / 2, (g[r + 1][c][1] + g[r + 1][c + 1][1]) / 2)


SHEET = None  # Sheet(), încărcat în main


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


def idw(data, uv):
    """Ponderile 1/distanța² (mm) ale celulelor măsurate față de uv; aceeași celulă cântărește de ~45 de ori mai mult."""
    return np.array([1.0 / (((p[0] - uv[0]) * COL) ** 2 + ((p[1] - uv[1]) * ROW) ** 2 + 25.0) for p, _ in data])


def predict_xy(data, uv, limit=MAX_FIX_MM):
    """Abaterea de punere (mm: x spre c2, y spre robot) estimată în uv, la fel ca `predict`, limitată la `limit` mm."""
    if not data:
        return np.zeros(2)
    w = idw(data, uv)
    e = (w[:, None] * np.array([e for _, e in data])).sum(0) / w.sum()
    n = float(np.linalg.norm(e))
    return e if n <= limit else e * limit / n


def predict(data, uv, limit=MAX_FIX_MM):
    """Abaterea laterală (mm) estimată în uv: media celulelor măsurate, ponderată cu 1/distanța² (fără extrapolare:
    pe 2026-10-29 un plan prin două celule a dat -9 mm pe r1c0 și a dus cubul pe linie), limitată la ±limit."""
    if not data:
        return 0.0
    w = idw(data, uv)
    e = float(w @ np.array([e for _, e in data]) / w.sum())
    return max(-limit, min(limit, e))


def base_angle(uv):
    """Unghiul bazei (grd) spre punctul uv, față de direcția spre foaie (+ = spre c0)."""
    dx, dy = uv[0] * COL - BASE_MM[0], uv[1] * ROW - BASE_MM[1]
    return math.degrees(math.atan2(-dx, -dy))


def predict_yaw0(data, uv):
    """Rotirea cu care ar ajunge cubul la comanda 0, în uv. Cubul păstrează cam rotirea brațului, deci o dreaptă în
    unghiul bazei prin celulele măsurate, când unghiurile lor diferă cu cel puțin 3 grd; altfel media ponderată."""
    if len(data) >= 2:
        th = np.array([base_angle(p) for p, _ in data])
        if th.max() - th.min() >= 3.0:
            k = np.linalg.lstsq(np.c_[np.ones(len(th)), th], np.array([e for _, e in data]), rcond=None)[0]
            return float(np.clip(k[0] + k[1] * base_angle(uv), -20.0, 20.0))
    return predict(data, uv, 45.0)


def corrected(uv, e_mm):
    """Punctul în care trebuie trimisă gheara ca să ajungă în uv, cu abaterea laterală e_mm."""
    lx, ly = lateral_dir(uv)
    return [uv[0] - e_mm * lx / COL, uv[1] - e_mm * ly / ROW]


def wrap45(a):
    """Rotirea unui cub (simetric la 90 grd), adusă în -45..45."""
    return (a + 45.0) % 90.0 - 45.0


def look(cube, tries=3):
    """Cubul din poza de start: două analize la rând care îl văd în același loc (±1,5 mm) și îi dau rotirea."""
    for _ in range(tries):
        (o1, _), (o2, img) = analyses(2)
        a, b = cube.seen(o1), cube.seen(o2)
        if a is None or b is None or b["yaw_deg"] is None:
            continue
        a = dict(a, uv_top=SHEET.to_ref(o1, a["uv_top"]))
        b = dict(b, uv_top=SHEET.to_ref(o2, b["uv_top"]))
        ua, ub = cube.base_uv(a), cube.base_uv(b)
        if math.hypot((ua[0] - ub[0]) * COL, (ua[1] - ub[1]) * ROW) <= 1.5:
            if a["yaw_deg"] is not None and abs(wrap45(a["yaw_deg"] - b["yaw_deg"])) <= 2.0:
                b = dict(b, yaw_deg=round(b["yaw_deg"] + wrap45(a["yaw_deg"] - b["yaw_deg"]) / 2.0, 2))
            return b, ub, o2, img
    return None


def status():
    return json.load(urllib.request.urlopen(f"{SERVICE}/status", timeout=10))["result"]


def grasp_text(st):
    """Unghiurile citite jos față de cele comandate și gheara după închidere."""
    g = st.get("last_grasp")
    if not g:
        return "fără citiri"
    d = ["?" if v is None else f"{v - t:+.0f}" for v, t in zip(g["bottom"][:5], g["sent"][:5])]
    # cu GRIP_CLOSED 140: goală citește 139, cubul de 29 mm o oprește pe la 133
    empty = g["closed"][5] is not None and g["closed"][5] >= st["grip"] - 2
    return (f"jos, citit - comandat: servo 1..5 [{', '.join(d)}] grd; gheara citită {g['closed'][5]} (comandată "
            f"{st['grip']}{', pare goală' if empty else ', strânge ceva'})")


def grab(cube, seen, uv, learned):
    """Pick pe cubul văzut în uv, apoi poza de start. Întoarce (starea, diagnoza, corecția): „prins”, „gol” (gheara
    s-a închis pe nimic: cubul e pe foaie) sau „neclar” (gheara strânge ceva, dar pe foaie e o celulă ocupată)."""
    e = predict(learned, uv)
    res = post("pick", {"location": LOCATION, "uv": corrected(uv, e), "yaw_deg": seen["yaw_deg"]})
    if res.get("result") is not True:
        sys.exit(f"pick refuzat: {res}")
    st = status()
    closed = ((st.get("last_grasp") or {}).get("closed") or [None] * 6)[5]
    holding = closed is not None and closed < st["grip"] - GRIP_EMPTY
    diag = grasp_text(st)
    post("beep", {"n": 2})
    start_pose()
    (obs, _), = analyses(1)
    left = [f"r{c['row']}c{c['col']}" for c in obs["cells"] if c["occupancy"] == "occupied"]
    if not holding:
        return "gol", diag, e
    return ("neclar" if left else "prins"), diag + (f"; pe foaie: {', '.join(left)} ocupat" if left else ""), e


GRAB = {"prins": "prins", "gol": "GHEARA GOALĂ, cubul a rămas pe foaie", "neclar": "NECLAR"}


def main():
    global SHEET
    SHEET = Sheet()
    sizes = [f"r{r}c{c} {(b[1] - b[0]) * COL:.1f}x{(b[3] - b[2]) * ROW:.1f}" for r in range(3) for c in range(3)
             for b in [SHEET.box(r, c)]]
    say("Liniile salvate din imaginea de referință (mm): " + ", ".join(sizes))
    route = [(int(s[0]), int(s[1])) for s in sys.argv[1:]]
    loc = load_app_config(ROOT / "ai/config").system.location(LOCATION)
    cube = Cube(loc, status()["cube_mm"])
    learned, learned_yaw, log = [], [], []
    learned_xy = [(uv, np.array(e)) for uv, e in PRIOR_XY]

    k_pairs = []  # k măsurat: (rotirea cubului) / (rotirea comandată), între două puneri pe aceeași celulă

    def k_now():
        return float(np.clip(np.median(k_pairs), *K_RANGE)) if k_pairs else K_WRIST

    def clamp_yaw(v):
        return max(-MAX_FIX_YAW, min(MAX_FIX_YAW, v))

    def aim(target):
        """Prima punere pe o celulă: unde și cu ce rotire, din tot ce s-a măsurat până acum (celulele apropiate)."""
        e = predict_xy(learned_xy, target)
        y0 = predict_yaw0([(uv, wrap45(yw - k_now() * cm)) for uv, cm, yw in learned_yaw], target)
        return [target[0] - e[0] / COL, target[1] - e[1] / ROW], clamp_yaw(-y0 / k_now()), e

    start_pose()
    for r, c in route:
        # 1. cubul și caroiajul, din poza de start
        found = look(cube)
        if found is None:
            sys.exit("Camera nu vede un singur cub, în același loc de două ori și cu rotirea lui: opresc.")
        seen, src, _, _ = found
        src_cell = (seen["row"], seen["col"])
        say(f"r{src_cell[0]}c{src_cell[1]} -> r{r}c{c}: cubul la uv ({src[0]:.3f}; {src[1]:.3f}), rotit {seen['yaw_deg']:+.1f} grd")
        # 2-3. pick, verificat din poza de start; o reîncercare
        for attempt in (1, 2):
            state, diag, e = grab(cube, seen, src, learned)
            say(f"  pick {attempt} (corecția laterală {-e:+.1f} mm): {diag} -> {GRAB[state]}")
            if state == "prins":
                break
            post("beep", {"n": 4})
            if state == "neclar":
                sys.exit("Gheara strânge ceva, dar pe foaie se vede un cub: opresc cu gheara închisă.")
            post("open_gripper")  # gheara s-a închis pe nimic
            found = look(cube)
            if attempt == 2 or found is None:
                sys.exit("Pick ratat: opresc (gheara deschisă)." + ("" if found else " Camera nu vede clar cubul."))
            seen, src, _, _ = found
            say(f"  cubul e acum la uv ({src[0]:.3f}; {src[1]:.3f}), rotit {seen['yaw_deg']:+.1f} grd: reîncerc")
        left, right, top, bottom = SHEET.box(r, c)
        target = ((left + right) / 2.0, (top + bottom) / 2.0)
        place_uv, yaw_cmd, e_dst = aim(target)
        say(f"  ținta r{r}c{c}: centrul din liniile salvate uv ({target[0]:.3f}; {target[1]:.3f}), celula "
            f"{(right - left) * COL:.1f} x {(bottom - top) * ROW:.1f} mm; place corectat cu {-e_dst[0]:+.1f} mm pe "
            f"coloane, {-e_dst[1]:+.1f} mm spre robot, rotire {yaw_cmd:+.1f} grd")
        for fix in range(MAX_FIXES + 1):
            # 4. place și verificarea
            res = post("place", {"location": LOCATION, "uv": place_uv, "yaw_deg": yaw_cmd})
            if res.get("result") is not True:
                sys.exit(f"place refuzat: {res} (cubul e încă în gheară)")
            start_pose()
            found = look(cube)
            if found is None:
                sys.exit("După place camera nu vede bine cubul (căzut pe linii?): opresc.")
            got, landed, _, _ = found
            fp = cube.footprint_mm(landed, got["yaw_deg"])
            margins = {"stânga": min(x for x, _ in fp) - left * COL, "dreapta": right * COL - max(x for x, _ in fp),
                       "sus": min(y for _, y in fp) - top * ROW, "jos": bottom * ROW - max(y for _, y in fp)}
            lx, ly = lateral_dir(target)
            ex, ey = (landed[0] - place_uv[0]) * COL, (landed[1] - place_uv[1]) * ROW  # față de unde a fost trimisă gheara
            e_true = ex * lx + ey * ly  # abaterea laterală a ghearei pe celula-țintă
            if fix and abs(yaw_cmd - prev_cmd) >= 5.0:  # aceeași celulă, altă rotire comandată: cât s-a rotit cubul
                k_pairs.append(wrap45(got["yaw_deg"] - prev_yaw) / (yaw_cmd - prev_cmd))
            prev_cmd, prev_yaw = yaw_cmd, got["yaw_deg"]
            off = math.hypot((landed[0] - target[0]) * COL, (landed[1] - target[1]) * ROW)
            learned.append((target, e_true))
            learned_xy.append((target, np.array([ex, ey])))
            learned_yaw.append((target, yaw_cmd, got["yaw_deg"]))
            straight = bool(abs(got["yaw_deg"]) <= YAW_TOL)  # bool: numpy.bool_ nu se scrie în JSON
            good = bool((got["row"], got["col"]) == (r, c) and off <= GOOD_OFF_MM
                        and min(margins.values()) >= GOOD_MARGIN_MM and straight)
            say(f"  pus{' (corecția ' + str(fix) + ')' if fix else ''}: {off:.1f} mm de centru, rotit {got['yaw_deg']:+.1f} grd "
                f"({'drept' if straight else 'STRÂMB'}, încheietura {yaw_cmd:+.1f} grd, k {k_now():.2f}); "
                "până la linii: " + ", ".join(f"{k} {v:+.1f}" for k, v in margins.items())
                + f" mm -> {'BINE' if good else 'DE CORECTAT'}; abaterea laterală a ghearei aici {e_true:+.1f} mm")
            log.append({"cell": f"r{r}c{c}", "from": f"r{src_cell[0]}c{src_cell[1]}", "fix": fix, "target_uv": target,
                        "sent_uv": place_uv, "yaw_cmd": yaw_cmd, "landed_uv": landed, "yaw": got["yaw_deg"],
                        "off_mm": off, "margins_mm": margins, "lateral_mm": e_true, "k": k_now(), "good": good})
            json.dump({"at": time.strftime("%Y-%m-%d %H:%M:%S"), "moves": log}, open(OUT, "w"), indent=1, ensure_ascii=False)
            post("beep", {"n": 1})
            if good or fix == MAX_FIXES:
                break
            # 5. corecția: din nou de unde a ajuns, cu abaterile învățate (acum dominate de celula asta)
            state, diag, _ = grab(cube, got, landed, learned)
            say(f"  corectez: pick din nou ({diag}) -> {GRAB[state]}")
            if state != "prins":
                post("beep", {"n": 4})
                if state == "neclar":
                    sys.exit("Gheara strânge ceva, dar pe foaie se vede un cub: opresc cu gheara închisă.")
                post("open_gripper")
                sys.exit("Pick-ul pentru corecție a ratat: opresc (gheara deschisă, cubul e pe foaie).")
            # pe aceeași celulă: poziția cu abaterea punerii de acum, rotirea cu un pas spre 0 (după k)
            e_dst = np.array([ex, ey])
            place_uv = [target[0] - ex / COL, target[1] - ey / ROW]
            if not straight:
                yaw_cmd = clamp_yaw(yaw_cmd - YAW_STEP * got["yaw_deg"] / k_now())
            say(f"  pun din nou corectat cu {-ex:+.1f} mm pe coloane, {-ey:+.1f} mm spre robot, încheietura {yaw_cmd:+.1f} grd")
    finals = [m for i, m in enumerate(log) if i + 1 == len(log) or log[i + 1]["fix"] == 0]
    say(f"Gata: {sum(m['good'] for m in finals)}/{len(finals)} mutări bine puse ({len(log) - len(finals)} corecții). "
        f"Rezultate: {OUT}")


if __name__ == "__main__":
    main()
