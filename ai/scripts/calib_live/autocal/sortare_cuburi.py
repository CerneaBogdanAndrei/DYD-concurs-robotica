"""Sortarea cuburilor: harta culorilor din caroiaj (pasul 1), apoi cuburile scoase de echipă în stânga, în dreapta și
sub caroiaj sunt puse înapoi pe celulele culorii lor (pașii 2-4).

Punctul de referință e centrul lui r1c1 (mijlocul caroiajului), văzut din poza de start (observe_small).

Pasul 1: echipa pune cuburile (cel mult 9) pe celule; două analize la rând din poza de start trebuie să dea aceleași
celule ocupate, cu aceleași culori. Harta se scrie în `harta_culori.json` (cu imaginea adnotată, `harta_culori.jpg`)
și rămâne până la sfârșitul tuturor pașilor; starea (celulele puse de aici încolo) pornește de la zero.

Pașii 2-4: din punctul de referință brațul pleacă în linie dreaptă spre zonă. În stânga și în dreapta se rotește baza
(camera merge în lungul lui r1 până la marginea caroiajului, 5,06 cm, și vede ~6 cm dincolo de ea; dacă nu vede niciun
cub sau unul iese din cadru, merge mai departe, ~10 cm); sub caroiaj brațul se strânge înapoi (12,35 cm spre robot).
Vederile sunt cele calibrate pe caroiajul gol (`vederi/caroiaj_gol`, `calib_vederi.py`), luate la fel ca atunci (din
poza de start); camera se reajustează la fiecare cadru pe liniile caroiajului care se văd. În zonă: cuburile găsite la
fel în două analize (fața de sus colorată, dusă în mm pe foaie); se ia cel mai apropiat de caroiaj.

Pick-ul are încheietura dreaptă: cu gheara înclinată, o încheietură rotită ține degetele la înălțimi diferite
(2026-10-30, 16 grd: vârfurile la 25 și 38 mm, dreaptă: 30 și 30), iar cubul se aliniază singur între degete. Doar un
cub rotit peste 30 grd față de degete face încheietura să se rotească, cel mult 10 grd. Dacă un deget ar atinge un cub
vecin (de pe caroiaj, din zonă sau o pată colorată care nu pare un cub), centrul ghearei se mută pe linia degetelor spre
spațiul liber, cel mult 8 mm și cât rămâne cubul între degetele deschise (regula B); altfel cubul rămâne pentru mai
târziu. Cubul prins e dus în punctul de referință (poza de start: acolo se verifică și caroiajul) și pus pe prima
celulă liberă cu culoarea lui, în ordinea rândurilor și coloanelor, exact ca la învățare (`traseu_cub.py`: ținta,
verificarea, corecțiile, învățarea în `invatat.json`); verificarea se uită doar la cubul nou de pe caroiaj. Se repetă
până nu mai e niciun cub în zonă. La sfârșit: după pasul 2 brațul trece în dreapta (10,12 cm), după pașii 3 și 4 revine
în punctul de referință. Starea e în `sortare_stare.json`, jurnalul punerilor în `sortare_cuburi.json`.

    ai/.venv/bin/python ai/scripts/calib_live/autocal/sortare_cuburi.py pas1
    ai/.venv/bin/python ai/scripts/calib_live/autocal/sortare_cuburi.py pas2   # stânga, apoi în dreapta
    ai/.venv/bin/python ai/scripts/calib_live/autocal/sortare_cuburi.py pas3   # dreapta, apoi în centru
    ai/.venv/bin/python ai/scripts/calib_live/autocal/sortare_cuburi.py pas4   # sub caroiaj, apoi în centru
    ... --doar-vede   # pașii 2-4: doar vederile, cuburile găsite și cum ar fi prinse, fără pick
"""
import argparse
import json
import math
import os
import shutil
import sys
import time

import cv2
import numpy as np

import foaie_la_loc
import traseu_cub
from traseu_cub import CLIPPED, COL, GOOD_MARGIN_MM, GOOD_OFF_MM, GRAB, GRIP_EMPTY, HELD_MAX_RATIO, HERE, K_RANGE, \
    K_WRIST, LOCATION, MAX_FIX_YAW, MAX_FIXES, PRIOR_XY, ROOT, ROW, YAW_TOL, Cube, Sheet, analyses, corrected, \
    grasp_text, lateral_dir, load_app_config, load_model, post, predict, predict_xy, predict_yaw0, save_model, say, \
    start_pose, status, wrap45
from vedere import CUBE_MM, DIST, K, Camera, cube_tops, lines, outside_gap_mm, visible_crossing

MAP = HERE / "harta_culori.json"
STATE = HERE / "sortare_stare.json"
OUT = HERE / "sortare_cuburi.json"
PREVIEW_DIR = ROOT / "ai/out/preview"
COLORS = {"red": "roșu", "yellow": "galben", "green": "verde", "blue": "albastru"}
CENTER_UV = (1.5, 1.5)  # punctul de referință: centrul lui r1c1, mijlocul caroiajului
FAINT = "faint_color_object"  # o urmă slabă pe hârtie (2026-10-30: pe r1c1, r1c2, r2c1), nu un cub
# puțină culoare pe celulă, fără un cub: 2026-10-30, verdele lăsat la ~5 mm în stânga lui r0 dădea pe r0c0 o bucată
# din fața lui (tăiată de marginea imaginii), cu centrul estimat în celulă
AMBIGUOUS = "ambiguous_colored_area"
NO_COLOR = ("edges_without_color", "dark_object_without_color", FAINT, AMBIGUOUS)  # celulă fără un cub colorat

VIEWS = json.load(open(HERE / "vederi/caroiaj_gol/calib_vederi.json"))["views"]
OBSERVE = VIEWS["centru"]["sent"]  # = observe_small
# pasul -> (zona, vederile în ordine: a doua doar dacă prima nu vede niciun cub sau unul iese din cadru)
ZONES = {"pas2": ("stânga", ["stanga11", "stanga20"]), "pas3": ("dreapta", ["dreapta11", "dreapta20"]),
         "pas4": ("sub caroiaj", ["jos3"])}
PATH = {"jos3": ["jos1", "jos2", "jos3"]}  # calea spre vedere, ca la calibrare (vederi_zone.py)
SAME_CUBE_MM = (3.0, 8.0)  # același cub în două analize: centrele la cel mult atât (întreg, tăiat de cadru)
MERGE_MM = 20.0  # același cub din două vederi (aceeași culoare): centrul unuia tăiat de cadru e mutat spre partea văzută
FACE_MM = (23.0, 36.0)  # fața de sus întreagă (29 mm) se măsoară între acestea; altfel e o pată care nu pare un cub
LINE_PX = (25.0, 8.0)  # liniile găsite se potrivesc cu cele proiectate la atâția px (cu camera de la calibrare, reajustată)
REFINE_MAX_PX = 4.0  # camera reajustată pe linii se folosește doar sub abaterea asta ...
REFINE_MAX_MM = 12.0  # ... și dacă nu mută foaia mai mult de atât (1 grd la bază ~ 5 mm)
SHEET_MOVED_MM = 4.0  # foaia (sau camera) mutată mai mult față de calibrarea vederilor: pick-ul ar fi greșit, se oprește
# gheara la pick (GRIP_OPEN 60, necalibrată în mm): deschiderea interioară ~46-55 mm (din înclinarea degetelor
# măsurată, 2026-10-30); pentru vecini contează cea mare, pentru cubul dintre degete cea mică
FINGER_GAP_MM = (46.0, 55.0)
FINGER_MM = 6.0  # grosimea unui deget (config.ts GRIP_FINGER_MM)
FINGER_W_MM = 16.0  # lățimea unui deget de-a lungul brațului (estimată, cu rezervă)
CLEAR_MM = 2.0  # cât rămâne cel puțin între un deget și un cub vecin
MAX_SHIFT_MM = 8.0  # cât se mută cel mult centrul ghearei față de cub
MAX_MISALIGN = 30.0  # cubul se aliniază singur între degete până la atâtea grade ...
MAX_WRIST = 10.0  # ... peste, încheietura se rotește cel mult atât (degetele ajung la înălțimi diferite)
PICK_OUTSIDE_CELLS = 3  # config.ts: pick-ul ia cuburi până la atâtea celule în afara caroiajului
GRIP_CLOSED = 140  # config.ts: gheara strânsă (după un pick)

SHEET = None  # Sheet(), încărcat în main


# ------------------------------------------------------------------ pasul 1


def held_strip(c):
    """Cubul din gheară, văzut din poza de start jos, peste r2, ca o fâșie, nu un cub de pe foaie (fața lui are ~29 x
    29 mm). 2026-10-30: ținut rotit după o corecție, era 27 x 11 mm, cu size_ratio 0,66 pe r2c1 (peste pragul de 0,6)."""
    sides = [c["metrics"].get(k) for k in ("side_w_mm", "side_h_mm")]
    return c["row"] == 2 and None not in sides and min(sides) < 18.0 and CLIPPED not in c.get("reasons", [])


def weak(c, lenient=False):
    """Celula are doar o urmă de culoare, nu un cub: o urmă slabă pe hârtie sau (`lenient`) puțină culoare ambiguă."""
    reasons = c.get("reasons", [])
    return c["occupancy"] != "occupied" and (FAINT in reasons or (lenient and AMBIGUOUS in reasons))


def spill(c, obs):
    """O celulă fără culoare lângă un cub de pe caroiaj: fața lui de sus sau cea laterală, văzute în perspectivă peste
    ea. 2026-10-30: verdele de pe r1c1, cu baza citită la 89 în loc de 90 grd, dădea pe r2c1 „obiect întunecat” (eticheta
    de pe fața laterală) și pe r1c2 contururi fără culoare, la fiecare analiză."""
    reasons = c.get("reasons", [])
    if c["occupancy"] == "occupied" or not reasons or any(r not in NO_COLOR for r in reasons):
        return False
    return any(on_paper(o) and o.get("color") in COLORS and abs(o["row"] - c["row"]) + abs(o["col"] - c["col"]) == 1
               for o in obs["cells"])


def cell_colors(obs, lenient=False):
    """{'r0c0': 'yellow', ...} pentru celulele ocupate; None dacă analiza nu e sigură (caroiajul negăsit, o celulă
    nesigură sau un cub cu o culoare necunoscută). O celulă cu doar o urmă slabă de culoare e goală; cu `lenient`, și
    una cu puțină culoare ambiguă (un cub de lângă caroiaj), cubul din gheară sau o celulă fără culoare lângă un cub."""
    if not obs["geometry"].get("ok"):
        return None
    out = {}
    for c in obs["cells"]:
        if c["occupancy"] == "empty" or weak(c, lenient) or (lenient and (held_strip(c) or spill(c, obs))):
            continue
        if c["occupancy"] != "occupied" or c.get("color") not in COLORS:
            return None
        out[f"r{c['row']}c{c['col']}"] = c["color"]
    return out


def table(cells):
    return "\n".join("  " + " | ".join(f"{COLORS.get(cells.get(f'r{r}c{c}'), '-'):8s}" for c in range(3))
                     for r in range(3))


def grid_colors(tries=5, lenient=False):
    """Celulele ocupate și culorile lor, la fel în două analize la rând din poza de start. -> (celule, analiza)"""
    for _ in range(tries):
        (o1, _), (o2, _) = analyses(2)
        a, b = cell_colors(o1, lenient), cell_colors(o2, lenient)
        if a is not None and a == b:
            return b, o2
    return None, None


def pas1():
    start_pose()
    cells, obs = grid_colors()
    if cells is None:
        sys.exit("Două analize la rând nu dau aceleași celule și culori (sau o celulă e nesigură): verificați cuburile.")
    shutil.copy(PREVIEW_DIR / "latest_annotated.jpg", MAP.with_suffix(".jpg"))
    json.dump({"at": time.strftime("%Y-%m-%d %H:%M:%S"), "center_uv": CENTER_UV, "cells": cells,
               "colors_ro": {k: COLORS[v] for k, v in cells.items()}, "image": MAP.with_suffix(".jpg").name},
              open(MAP, "w"), indent=1, ensure_ascii=False)
    save_state({"filled": {}, "skipped": []})
    faint = [f"r{c['row']}c{c['col']}" for c in obs["cells"] if c["occupancy"] != "occupied" and FAINT in c.get("reasons", [])]
    say(f"Harta culorilor ({len(cells)} cuburi), salvată în {MAP.name}:\n{table(cells)}"
        + (f"\n  (urme slabe, luate ca goale: {', '.join(faint)})" if faint else ""))


# ------------------------------------------------------------------ vederile din zone


def raw_frames(n):
    """`n` cadre brute (fără conturul desenat de previzualizare), diferite, făcute după ce brațul s-a oprit.
    Previzualizarea scrie atomic cadrul brut înaintea analizei lui: după o analiză nouă, cadrul citit e al ei sau unul
    și mai nou."""
    out, last = [], None
    while len(out) < n:
        analyses(1)
        path = PREVIEW_DIR / "latest_raw.jpg"
        stamp = os.stat(path).st_mtime_ns
        raw = cv2.imread(str(path))
        if raw is not None and stamp != last:
            out.append(raw)
            last = stamp
    return out


def match_lines(und, cam, tol):
    """Liniile caroiajului găsite în imaginea fără distorsiune: {(0, k): coloana x = 33k, (1, k): linia y = 33k}, fiecare
    cu linia detectată cea mai apropiată de proiecția ei cu `cam` (la cel mult `tol` px, aceeași direcție ±4 grd)."""
    found = {}
    for fam in lines(und):
        for det in fam:
            p, d, pts = det
            for axis in (0, 1):
                for k in range(4):
                    a = [COL * k, 0.0, 0.0] if axis == 0 else [0.0, ROW * k, 0.0]
                    b = [COL * k, 3 * ROW, 0.0] if axis == 0 else [3 * COL, ROW * k, 0.0]
                    pa, pb = cam.project([a, b])
                    length = float(np.linalg.norm(pb - pa))
                    e = (pb - pa) / length
                    if abs(d @ e) < math.cos(math.radians(4)):
                        continue
                    dist = float(np.median(np.abs((pts - pa) @ np.array([-e[1], e[0]]))))
                    t = (pts - pa) @ e
                    if dist < tol and np.mean((t > -20) & (t < length + 20)) > 0.6:
                        if (axis, k) not in found or dist < found[(axis, k)][0]:
                            found[(axis, k)] = (dist, det)
    return {key: det for key, (_, det) in found.items()}


def crossings(und, cam, tol):
    """Intersecțiile caroiajului văzute (mm pe foaie, px fără distorsiune), din liniile potrivite cu `cam`."""
    found = match_lines(und, cam, tol)
    obj, img = [], []
    for (ax, i), lc in found.items():
        for (ay, j), lr in found.items():
            if ax == 0 and ay == 1:
                x = visible_crossing(lc, lr)
                if x is not None:
                    obj.append((i * COL, j * ROW))
                    img.append(x)
    return obj, img


def refine(raws, prior):
    """Camera reajustată pe intersecțiile caroiajului care se văd în cadrele `raws` (aceeași poză; cuburile pot acoperi
    o parte din ele), în doi pași: liniile potrivite cu cele proiectate cu camera de la calibrare, apoi, mai strâns, cu
    cea reajustată. Cu doar 2-5 intersecții, camera de la calibrare doar se mută pe foaie cu abaterea lor medie (baza nu
    revine exact: ~1 grd mută foaia ~5 mm, aproape o translație). -> (camera, text, cât s-a mutat foaia față de
    camera de la calibrare, mm, sau None)."""
    unds = [cv2.undistort(raw, K, DIST) for raw in raws]
    cam, rms, n = prior, None, 0
    for tol in LINE_PX:
        obj, img = [], []
        for und in unds:
            o, i = crossings(und, cam, tol)
            obj += o
            img += i
        seen = len(set(obj))
        if seen < 6:
            if rms is None and seen >= 2:
                d = (np.array(obj) - cam.to_sheet(img, 0.0)).mean(0)
                spread = float(np.abs(np.array(obj) - cam.to_sheet(img, 0.0) - d).max())
                moved_mm = float(np.linalg.norm(d))
                if moved_mm <= REFINE_MAX_MM and spread <= 2.0:
                    moved = Camera(cam.rvec, -cam.R @ (cam.center + np.array([d[0], d[1], 0.0])))
                    return moved, f"camera de la calibrare mutată pe foaie cu {moved_mm:.1f} mm ({seen} intersecții)", moved_mm
            n = n or seen
            break
        cam, rms, _ = Camera.fit(obj, img, guess=cam)
        n = seen
    if rms is None:
        return prior, f"camera de la calibrare ({n} intersecții văzute)", None
    shift = float(np.linalg.norm(cam.to_sheet([[320, 240]], -CUBE_MM) - prior.to_sheet([[320, 240]], -CUBE_MM)))
    if rms > REFINE_MAX_PX or shift > REFINE_MAX_MM:
        return prior, f"camera de la calibrare (reajustată pe {n} intersecții ar fi {rms:.1f} px, {shift:.1f} mm)", (
            shift if rms <= REFINE_MAX_PX else None)
    return cam, f"camera reajustată pe {n} intersecții ({rms:.1f} px, foaia mutată {shift:.1f} mm)", shift


def is_face(c):
    """O față de cub: întreagă, cu laturile și umplerea unui pătrat de 29 mm, sau completată din laturile văzute."""
    return c["completed"] or (FACE_MM[0] <= min(c["sides_mm"]) and max(c["sides_mm"]) <= FACE_MM[1]
                              and c["fill"] >= 0.8)


def cubes_in(raw, cam):
    """Fețele colorate din afara caroiajului într-un cadru. -> (cuburi: fețe întregi, completate sau tăiate de cadru,
    celelalte pete colorate)"""
    cubes, blobs = [], []
    for c in cube_tops(raw, cam):
        if outside_gap_mm(c["center_mm"]) > 0:
            (cubes if c["clipped"] or is_face(c) else blobs).append(c)
    return cubes, blobs


def same_cubes(a, b, view):
    """Cuburile din două cadre, potrivite (aceeași culoare, centrele la cel mult SAME_CUBE_MM), cu centrul și rotirea
    mediate; None dacă nu se potrivesc toate."""
    pairs = []
    for ca in a:
        loose = lambda c: c["clipped"] or c["completed"]
        near = [cb for cb in b if cb["color"] == ca["color"] and math.dist(ca["center_mm"], cb["center_mm"])
                <= SAME_CUBE_MM[loose(ca) or loose(cb)]]
        if len(near) == 1:
            cb = near[0]
            pairs.append(dict(cb, center_mm=tuple((np.array(ca["center_mm"]) + cb["center_mm"]) / 2.0),
                              yaw_deg=cb["yaw_deg"] + wrap45(ca["yaw_deg"] - cb["yaw_deg"]) / 2.0,
                              clipped=ca["clipped"] or cb["clipped"], completed=ca["completed"] or cb["completed"],
                              view=view))
    return pairs if len(pairs) == len(a) == len(b) else None


def view_cubes(view):
    """Cuburile din afara caroiajului văzute din `view`, la fel în două cadre (camera reajustată pe amândouă).
    -> (cuburile, celelalte pete, text despre cameră), sau None. Se oprește dacă foaia (sau camera) s-a mutat peste
    SHEET_MOVED_MM față de calibrarea vederii: pick-urile ar fi greșite."""
    prior = Camera.from_dict(VIEWS[view]["camera"])
    for _ in range(3):
        raws = raw_frames(2)
        cam, info, moved = refine(raws, prior)
        if moved is not None and moved > SHEET_MOVED_MM:
            post("beep", {"n": 4})
            sys.exit(f"  {view}: {info}. Foaia (sau camera) s-a mutat cu {moved:.0f} mm față de calibrarea vederilor: "
                     "opresc înainte de pick.")
        (a, _), (b, blobs) = [cubes_in(raw, cam) for raw in raws]
        pairs = same_cubes(a, b, view)
        if pairs is not None:
            return pairs, blobs, info
    return None


def goto_view(view, at_start=False):
    """Din punctul de referință (home -> poza de start) în vederea `view`, ca la calibrare (vederi_zone.py)."""
    if not at_start:
        start_pose()
    for v in PATH.get(view, [view]):
        post("move_joints", {"angles": VIEWS[v]["sent"], "time_ms": 2000 if v.startswith("jos") else 1200})
    time.sleep(1.5)


def back_to_center(view):
    """Din vederea `view` înapoi în poza de start, pe drumul pe care a venit (baza înapoi / brațul întins la loc)."""
    for v in reversed(PATH.get(view, [view])[:-1]):
        post("move_joints", {"angles": VIEWS[v]["sent"], "time_ms": 1500})
    post("move_joints", {"angles": OBSERVE, "time_ms": 2000 if view.startswith("jos") else 1200})


def where(c):
    return f"{COLORS.get(c['color'], c['color'])} la ({c['center_mm'][0]:.0f}; {c['center_mm'][1]:.0f}) mm"


def search(views, at_start=False):
    """Cuburile din zonă, privite din toate vederile ei. Un cub văzut în mai multe (aceeași culoare, la cel mult
    MERGE_MM) se ia din cea în care e întreg și cât mai aproape de centrul imaginii, unde obiectivul e mai bine
    modelat (2026-10-30: un cub din colțul imaginii din stanga11 ieșea cu ~3 mm mai încolo decât din centrul lui
    stanga20). -> (cuburile, celelalte pete colorate, ultima vedere)"""
    rank = lambda c: (c["clipped"], c["completed"], c["off_center_px"])
    cubes, blobs, view = [], [], None
    for i, view in enumerate(views):
        if i == 0:
            goto_view(view, at_start)
        else:
            post("move_joints", {"angles": VIEWS[view]["sent"], "time_ms": 1200})
            time.sleep(1.5)
        found = view_cubes(view)
        if found is None:
            say(f"  {view}: două analize la rând nu dau aceleași cuburi (se mișcă ceva în cadru?)")
            continue
        new, other, info = found
        for c in new:
            same = [k for k, o in enumerate(cubes) if o["color"] == c["color"]
                    and math.dist(o["center_mm"], c["center_mm"]) <= MERGE_MM]
            if not same:
                cubes.append(c)
            elif rank(c) < rank(cubes[same[0]]):
                cubes[same[0]] = c
        blobs += other
        say(f"  {view}: {info}; " + (", ".join(
            f"{where(c)}{' (TĂIAT de cadru)' if c['clipped'] else ' (completat)' if c['completed'] else ''}, "
            f"{outside_gap_mm(c['center_mm']):.0f} mm de caroiaj, rotit {c['yaw_deg']:+.0f} grd" for c in new)
            or "niciun cub")
            + "".join(f"; pată {COLORS.get(b['color'], b['color'])} {b['sides_mm'][0]:.0f}x{b['sides_mm'][1]:.0f} mm "
                      f"la ({b['center_mm'][0]:.0f}; {b['center_mm'][1]:.0f}), nu pare un cub" for b in other))
    say("  " + (", ".join(f"{where(c)} din {c['view']}" for c in cubes) or "niciun cub în zonă"))
    return cubes, blobs, view


# ------------------------------------------------------------------ gheara printre cuburi


class RobotGrid:
    """Caroiajul în sistemul robotului, din calib.json (ca serviciul: cele mai mici pătrate pe [1, c, r]); pentru
    yaw_deg-ul cu care servo 5 stă la J5_CENTER (kinematics.wristForYaw: încheietura urmează unghiul bazei)."""

    def __init__(self):
        pts = json.load(open(ROOT / "calib.json"))["sites"][LOCATION]["points"]
        A = np.array([[1.0, int(k[3]), int(k[1])] for k in pts])
        P = np.array([v[:2] for v in pts.values()])
        self.O, self.ecol, self.erow = np.linalg.lstsq(A, P, rcond=None)[0]

    def xy(self, uv):
        return self.O + (uv[0] - 0.5) * self.ecol + (uv[1] - 0.5) * self.erow

    def straight_yaw(self, uv):
        """yaw_deg (pe foaie, -45..45) cu care încheietura stă dreaptă deasupra lui uv: direcția bazei (+ k x 90 grd),
        dusă pe foaie. Caroiajul e puțin oblic în sistemul robotului, deci dintre cele patru direcții se alege cea care
        cade în -45..45 pe foaie (wrap45 pe foaie ar da ~0,5 grd abatere)."""
        x, y = self.xy(uv)
        found = []
        for k in range(4):
            th = math.atan2(y, x) + k * math.pi / 2.0
            a, b = np.linalg.solve(np.c_[self.ecol, self.erow], [math.cos(th), math.sin(th)])
            found.append(math.degrees(math.atan2(b, a)))
        return min(found, key=abs)


def rect(center, sides, yaw_deg):
    return (tuple(map(float, center)), tuple(map(float, sides)), float(yaw_deg))


def fingers(center, yaw_deg):
    """Cele două degete (dreptunghiuri cv2) în jurul centrului ghearei; linia degetelor e la `yaw_deg` pe foaie."""
    line = np.array([math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))])
    off = (FINGER_GAP_MM[1] + FINGER_MM) / 2.0
    return [rect(np.asarray(center) + s * off * line, (FINGER_MM, FINGER_W_MM), yaw_deg) for s in (-1, 1)]


def overlaps(r1, r2):
    return cv2.rotatedRectangleIntersection(r1, r2)[0] != cv2.INTERSECT_NONE


def obstacle(o):
    """Un vecin, mărit cu CLEAR_MM pe fiecare parte: un cub (latura lui) sau o pată (dreptunghiul ei)."""
    sides = o["sides_mm"] if o.get("blob") else (CUBE_MM, CUBE_MM)
    return rect(o["center_mm"], (sides[0] + 2 * CLEAR_MM, sides[1] + 2 * CLEAR_MM), o.get("yaw_deg") or 0.0)


def grip_shift(center, yaw_deg, others, limit):
    """Cea mai mică mutare (mm, pe linia degetelor, cel mult `limit`) a centrului ghearei față de cub, cu care niciun
    deget nu atinge un vecin. None dacă nu există."""
    line = np.array([math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))])
    obstacles = [obstacle(o) for o in others]
    for s in sorted(np.arange(-math.floor(limit), math.floor(limit) + 0.5, 1.0), key=abs):
        if not any(overlaps(f, o) for f in fingers(np.asarray(center) + s * line, yaw_deg) for o in obstacles):
            return float(s)
    return None


def plan_pick(c, others, rg, learned):
    """Cum se prinde cubul `c`: rotirea încheieturii, mutarea ghearei printre vecini, punctul trimis.
    -> (planul, None) sau (None, de ce nu se poate)."""
    uv = (c["center_mm"][0] / COL, c["center_mm"][1] / ROW)
    if not all(-PICK_OUTSIDE_CELLS <= t <= 3 + PICK_OUTSIDE_CELLS for t in uv):
        return None, f"e la peste {PICK_OUTSIDE_CELLS} celule de caroiaj (serviciul nu ia de acolo)"
    straight = rg.straight_yaw(uv)
    turn = wrap45(c["yaw_deg"] - straight)  # cubul față de degete, cu încheietura dreaptă
    wrist = math.copysign(min(abs(turn) - MAX_MISALIGN, MAX_WRIST), turn) if abs(turn) > MAX_MISALIGN else 0.0
    rest = math.radians(turn - wrist)  # cât se aliniază singur
    extent = CUBE_MM / 2.0 * (abs(math.cos(rest)) + abs(math.sin(rest)))  # jumătatea cubului pe linia degetelor
    limit = min(MAX_SHIFT_MM, FINGER_GAP_MM[0] / 2.0 - extent - 1.0)
    shift = grip_shift(c["center_mm"], straight + wrist, others, max(limit, 0.0))
    if shift is None:
        return None, "un deget ar atinge un vecin oricum s-ar muta gheara"
    a = math.radians(straight + wrist)
    grip_uv = ((c["center_mm"][0] + shift * math.cos(a)) / COL, (c["center_mm"][1] + shift * math.sin(a)) / ROW)
    e = predict(learned, grip_uv)
    sent = corrected(grip_uv, e)
    return {"uv": sent, "yaw_deg": rg.straight_yaw(sent) + wrist, "wrist": wrist, "turn": turn, "shift": shift,
            "lateral": e}, None


# ------------------------------------------------------------------ punerea pe caroiaj (ca în traseu_cub.py)


def on_paper(c):
    """Celula e ocupată de un cub de pe foaie (cubul din gheară, văzut jos ca o fâșie, nu se numără)."""
    return (c["occupancy"] == "occupied" and c["metrics"].get("size_ratio", 1.0) >= HELD_MAX_RATIO
            and not held_strip(c))


def occupied(obs):
    """Celulele ocupate de un cub de pe foaie."""
    return {f"r{c['row']}c{c['col']}" for c in obs["cells"] if on_paper(c)}


def new_cube(obs, filled):
    """Cubul de pe o celulă care nu e printre cele puse deja; None dacă nu e exact unul (sau e tăiat altundeva decât
    sus pe r0 / în stânga pe c0, ca în traseu_cub.Cube.seen)."""
    cells = [c for c in obs["cells"] if on_paper(c) and c["uv_top"] and f"r{c['row']}c{c['col']}" not in filled]
    if len(cells) != 1:
        return None
    u, v = cells[0]["uv_top"]
    return None if CLIPPED in cells[0]["reasons"] and v >= 0.8 and u >= 0.8 else cells[0]


def look_new(cube, filled, tries=3):
    """Ca traseu_cub.look: cubul nou de pe caroiaj, de două ori în același loc (±1,5 mm), cu rotirea lui."""
    for _ in range(tries):
        (o1, _), (o2, img) = analyses(2)
        a, b = new_cube(o1, filled), new_cube(o2, filled)
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


def holding():
    st = status()
    closed = ((st.get("last_grasp") or {}).get("closed") or [None] * 6)[5]
    return closed is not None and closed < st["grip"] - GRIP_EMPTY, grasp_text(st)


def grab_at(cube, seen, uv, learned, filled):
    """Ca traseu_cub.grab, cu alte cuburi pe caroiaj: „neclar” dacă după pick se vede tot un cub nou pe caroiaj."""
    e = predict(learned, uv)
    res = post("pick", {"location": LOCATION, "uv": corrected(uv, e), "yaw_deg": seen["yaw_deg"]})
    if res.get("result") is not True:
        sys.exit(f"pick refuzat: {res}")
    held, diag = holding()
    post("beep", {"n": 2})
    start_pose()
    (obs, _), = analyses(1)
    left = sorted(occupied(obs) - set(filled))
    if not held:
        return "gol", diag, e
    return ("neclar" if left else "prins"), diag + (f"; pe foaie: {', '.join(left)} ocupat" if left else ""), e


class Placer:
    """Punerea pe o celulă, cu verificarea, corecțiile și învățarea din traseu_cub.main (forced = 0)."""

    def __init__(self, cube):
        self.cube = cube
        self.learned, self.learned_xy, self.learned_yaw, self.k_pairs = load_model()
        self.prior = [(uv, np.array(e)) for uv, e in PRIOR_XY]
        self.log = json.load(open(OUT))["moves"] if OUT.exists() else []

    def k_now(self):
        return float(np.clip(np.median(self.k_pairs), *K_RANGE)) if self.k_pairs else K_WRIST

    @staticmethod
    def clamp_yaw(v):
        return max(-MAX_FIX_YAW, min(MAX_FIX_YAW, v))

    def aim(self, target):
        e = predict_xy(self.prior + self.learned_xy, target)
        y0 = predict_yaw0([(uv, wrap45(yw - self.k_now() * cm)) for uv, cm, yw in self.learned_yaw], target)
        return [target[0] - e[0] / COL, target[1] - e[1] / ROW], self.clamp_yaw(-y0 / self.k_now()), e

    def place(self, r, c, filled, source):
        """Cubul din gheară pe celula (r, c); `filled` = celulele puse deja. -> (cubul de pe celulă, bine pus)."""
        cube = self.cube
        left, right, top, bottom = SHEET.box(r, c)
        target = ((left + right) / 2.0, (top + bottom) / 2.0)
        place_uv, yaw_cmd, e_dst = self.aim(target)
        say(f"  ținta r{r}c{c}: centrul din liniile salvate uv ({target[0]:.3f}; {target[1]:.3f}); place corectat cu "
            f"{-e_dst[0]:+.1f} mm pe coloane, {-e_dst[1]:+.1f} mm spre robot, rotire {yaw_cmd:+.1f} grd")
        prev_cmd = prev_yaw = None
        for fix in range(MAX_FIXES + 1):
            res = post("place", {"location": LOCATION, "uv": place_uv, "yaw_deg": yaw_cmd})
            if res.get("result") is not True:
                sys.exit(f"place refuzat: {res} (cubul e încă în gheară)")
            start_pose()
            found = look_new(cube, filled)
            if found is None:
                sys.exit("După place camera nu vede bine cubul nou (căzut pe linii? a mișcat alt cub?): opresc.")
            got, landed, _, _ = found
            fp = cube.footprint_mm(landed, got["yaw_deg"])
            margins = {"stânga": min(x for x, _ in fp) - left * COL, "dreapta": right * COL - max(x for x, _ in fp),
                       "sus": min(y for _, y in fp) - top * ROW, "jos": bottom * ROW - max(y for _, y in fp)}
            lx, ly = lateral_dir(target)
            ex, ey = (landed[0] - place_uv[0]) * COL, (landed[1] - place_uv[1]) * ROW
            e_true = ex * lx + ey * ly
            if fix and abs(yaw_cmd - prev_cmd) >= 3.0:
                self.k_pairs.append(wrap45(got["yaw_deg"] - prev_yaw) / (yaw_cmd - prev_cmd))
            prev_cmd, prev_yaw = yaw_cmd, got["yaw_deg"]
            off = math.hypot((landed[0] - target[0]) * COL, (landed[1] - target[1]) * ROW)
            self.learned.append((target, e_true))
            self.learned_xy.append((target, np.array([ex, ey])))
            self.learned_yaw.append((target, yaw_cmd, got["yaw_deg"]))
            straight = bool(abs(got["yaw_deg"]) <= YAW_TOL)
            good = bool((got["row"], got["col"]) == (r, c) and off <= GOOD_OFF_MM
                        and min(margins.values()) >= GOOD_MARGIN_MM and straight)
            say(f"  pus{' (corecția ' + str(fix) + ')' if fix else ''}: {off:.1f} mm de centru, rotit {got['yaw_deg']:+.1f} grd "
                f"({'drept' if straight else 'STRÂMB'}, încheietura {yaw_cmd:+.1f} grd, k {self.k_now():.2f}); "
                "până la linii: " + ", ".join(f"{k} {v:+.1f}" for k, v in margins.items())
                + f" mm -> {'BINE' if good else 'DE CORECTAT'}; abaterea laterală a ghearei aici {e_true:+.1f} mm")
            self.log.append({"cell": f"r{r}c{c}", "from": source, "fix": fix, "target_uv": target, "sent_uv": place_uv,
                             "yaw_cmd": yaw_cmd, "landed_uv": landed, "yaw": got["yaw_deg"], "off_mm": off,
                             "margins_mm": margins, "lateral_mm": e_true, "k": self.k_now(), "good": good})
            json.dump({"at": time.strftime("%Y-%m-%d %H:%M:%S"), "moves": self.log}, open(OUT, "w"), indent=1,
                      ensure_ascii=False)
            save_model(self.learned, self.learned_xy, self.learned_yaw, self.k_pairs)
            post("beep", {"n": 1})
            if good or fix == MAX_FIXES:
                return dict(got, landed_uv=landed), good
            state, diag, _ = grab_at(cube, got, landed, self.learned, filled)
            say(f"  corectez: pick din nou ({diag}) -> {GRAB[state]}")
            if state != "prins":
                post("beep", {"n": 4})
                if state == "neclar":
                    sys.exit("Gheara strânge ceva, dar pe caroiaj se vede tot cubul: opresc cu gheara închisă.")
                post("open_gripper")
                sys.exit("Pick-ul pentru corecție a ratat: opresc (gheara deschisă, cubul e pe foaie).")
            place_uv = [target[0] - ex / COL, target[1] - ey / ROW]
            if not straight:
                yaw_cmd = self.clamp_yaw(yaw_cmd - got["yaw_deg"] / self.k_now())
            say(f"  pun din nou corectat cu {-ex:+.1f} mm pe coloane, {-ey:+.1f} mm spre robot, încheietura {yaw_cmd:+.1f} grd")


# ------------------------------------------------------------------ pașii 2-4


def load_state():
    return json.load(open(STATE)) if STATE.exists() else {"filled": {}, "skipped": []}


def save_state(state):
    state["at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    json.dump(state, open(STATE, "w"), indent=1, ensure_ascii=False)


def free_cell(colors, color, filled):
    """Prima celulă liberă cu culoarea dată, în ordinea rândurilor și coloanelor."""
    for r in range(3):
        for c in range(3):
            if colors.get(f"r{r}c{c}") == color and f"r{r}c{c}" not in filled:
                return r, c
    return None


def grid_cubes(filled):
    """Cuburile de pe caroiaj (centrul măsurat la punere), ca vecini pentru degete."""
    return [{"center_mm": (f["landed_uv"][0] * COL, f["landed_uv"][1] * ROW), "yaw_deg": f.get("yaw") or 0.0}
            for f in filled.values()]


def check_grid(state, cube):
    """Caroiajul din poza de start față de stare: un cub care nu e printre cele puse (lăsat de echipă) intră în stare
    cu culoarea lui și rămâne acolo; o celulă pusă care nu se mai vede ocupată e doar semnalată (nu se mai pune nimic
    peste ea)."""
    cells, obs = grid_colors(lenient=True)
    if cells is None:
        sys.exit("Din poza de start două analize la rând nu dau aceleași celule ocupate: verificați caroiajul.")
    ref = json.load(open(traseu_cub.REF.with_suffix(".json")))
    dx, dy, ang = foaie_la_loc.offset(obs, cv2.getPerspectiveTransform(np.float32(ref["geometry"]["corners_px"]),
                                                                       traseu_cub.UV))
    say(f"  din poza de start, caroiajul față de imaginea de referință: {dx:+.1f} mm pe coloane, {dy:+.1f} mm spre "
        f"robot, rotit {ang:+.2f} grd")
    vague = [f"r{c['row']}c{c['col']}" for c in obs["cells"] if weak(c, True) and AMBIGUOUS in c.get("reasons", [])]
    if vague:
        say(f"  {', '.join(vague)}: puțină culoare, fără un cub (un cub de lângă caroiaj?): luate ca goale")
    near = [f"r{c['row']}c{c['col']}" for c in obs["cells"] if spill(c, obs) and not weak(c, True)]
    if near:
        say(f"  {', '.join(near)}: fără culoare, lângă un cub (fața lui văzută în perspectivă): luate ca goale")
    for c in obs["cells"]:
        key = f"r{c['row']}c{c['col']}"
        if key in cells and key not in state["filled"]:
            uv = cube.base_uv(dict(c, uv_top=SHEET.to_ref(obs, c["uv_top"]))) if c["uv_top"] else (c["col"] + 0.5,
                                                                                                  c["row"] + 0.5)
            state["filled"][key] = {"color": cells[key], "landed_uv": [float(uv[0]), float(uv[1])],
                                    "yaw": c.get("yaw_deg"), "good": None, "found_on_grid": True}
            say(f"  pe {key} e deja un cub {COLORS[cells[key]]}: rămâne acolo")
    gone = sorted(set(state["filled"]) - set(cells))
    if gone:
        say(f"  ATENȚIE: {', '.join(gone)} nu se mai văd ocupate (cub mutat?); nu pun nimic peste ele")
    save_state(state)


def zone(step, only_look=False, held=None):
    """Zona pasului `step`; `held` = celula (r, c) pe care trebuie pus întâi cubul care e deja în gheară (o rulare
    oprită după ce l-a prins)."""
    name, views = ZONES[step]
    colors = json.load(open(MAP))["cells"]
    state = load_state()
    cube = Cube(load_app_config(ROOT / "ai/config").system.location(LOCATION), status()["cube_mm"])
    rg = RobotGrid()
    placer = Placer(cube)
    say(f"Pasul {step[-1]}, {name}. Harta: " + ", ".join(f"{k} {COLORS[v]}" for k, v in sorted(colors.items()))
        + f"; puse deja: {', '.join(sorted(state['filled'])) or 'niciuna'}")
    if status()["grip"] == GRIP_CLOSED and held is None:
        sys.exit("Gheara e închisă: poate ține un cub (o rulare oprită la mijloc?). Scoateți-l / deschideți gheara.")
    if held is not None and (status()["grip"] != GRIP_CLOSED or not holding()[0]):
        sys.exit("--in-gheara: gheara nu ține niciun cub (deschisă sau goală la ultimul pick).")
    start_pose()
    check_grid(state, cube)
    if held is not None:
        r, cc = held
        key = f"r{r}c{cc}"
        if key in state["filled"] or key not in colors:
            sys.exit(f"{key} e deja pusă sau nu e pe hartă: cubul din gheară nu are unde merge.")
        got, good = placer.place(r, cc, state["filled"], "din gheară")
        state["filled"][key] = {"color": colors[key], "landed_uv": [float(v) for v in got["landed_uv"]],
                                "yaw": got["yaw_deg"], "good": good}
        save_state(state)
    fails, view, at_start = 0, views[0], True
    while True:
        cubes, blobs, view = search(views, at_start)
        at_start = False
        todo = [c for c in cubes if not c["clipped"]
                and not any(math.dist(c["center_mm"], s) < 15.0 for s in state["skipped"])]
        if not todo:
            say(f"  În {name} nu mai e niciun cub de pus" + (" (doar tăiate de cadru)" if any(
                c["clipped"] for c in cubes) else "") + ".")
            break
        plans = []
        for c in sorted(todo, key=lambda c: outside_gap_mm(c["center_mm"])):
            cell = free_cell(colors, c["color"], state["filled"])
            if cell is None:
                say(f"  {where(c)}: nicio celulă liberă cu culoarea lui pe hartă, îl las în zonă")
                if not only_look:
                    state["skipped"].append(list(c["center_mm"]))
                    save_state(state)
                continue
            others = [o for o in cubes if o is not c] + [dict(b, blob=True) for b in blobs] + grid_cubes(state["filled"])
            p, why = plan_pick(c, others, rg, placer.learned)
            if p is None:
                say(f"  {where(c)}: {why}, îl las pentru mai târziu")
                continue
            plans.append((c, cell, p))
            say(f"  {where(c)}, {outside_gap_mm(c['center_mm']):.0f} mm de caroiaj -> r{cell[0]}c{cell[1]}: rotit "
                f"{p['turn']:+.0f} grd față de degete, încheietura {p['wrist']:+.0f} grd, gheara mutată "
                f"{p['shift']:+.0f} mm pe linia degetelor, corecția laterală {-p['lateral']:+.1f} mm, uv "
                f"({p['uv'][0]:.2f}; {p['uv'][1]:.2f}), yaw {p['yaw_deg']:+.1f}")
            if not only_look:
                break
        if only_look:
            break
        if not plans:
            say(f"  În {name} niciun cub nu poate fi luat acum (vecini prea aproape): mutați-le puțin mai departe.")
            break
        c, (r, cc), p = plans[0]
        res = post("pick", {"location": LOCATION, "uv": p["uv"], "yaw_deg": p["yaw_deg"]})
        if res.get("result") is not True:
            say(f"  pick refuzat ({res.get('reason') or res.get('error')}): îl las în zonă")
            state["skipped"].append(list(c["center_mm"]))
            save_state(state)
            continue
        held, diag = holding()
        post("beep", {"n": 2})
        say(f"  pick: {diag} -> {'prins' if held else 'GHEARA GOALĂ'}")
        if not held:
            fails += 1
            post("beep", {"n": 4})
            post("open_gripper")
            if fails >= 2:
                sys.exit("Două pick-uri ratate la rând în zonă: opresc (gheara deschisă).")
            continue
        fails = 0
        # înapoi în punctul de referință; caroiajul trebuie să fie cum a fost lăsat
        start_pose()
        (obs, _), = analyses(1)
        extra = sorted(occupied(obs) - set(state["filled"]))
        if extra:
            post("beep", {"n": 4})
            sys.exit(f"Pe caroiaj apare un cub în plus ({', '.join(extra)}): opresc cu cubul în gheară.")
        got, good = placer.place(r, cc, state["filled"], f"{name} ({c['center_mm'][0]:.0f}; {c['center_mm'][1]:.0f})")
        at_start = True  # punerea se încheie cu verificarea din poza de start
        state["filled"][f"r{r}c{cc}"] = {"color": c["color"], "landed_uv": [float(v) for v in got["landed_uv"]],
                                         "yaw": got["yaw_deg"], "good": good,
                                         "from_mm": [float(v) for v in c["center_mm"]]}
        save_state(state)
    return view


def main():
    global SHEET
    ap = argparse.ArgumentParser()
    ap.add_argument("pas", choices=["pas1", "pas2", "pas3", "pas4"])
    ap.add_argument("--doar-vede", action="store_true", help="pașii 2-4: doar vederile, cuburile și cum ar fi prinse")
    ap.add_argument("--in-gheara", metavar="RC", help="un cub e deja în gheară: întâi se pune pe celula RC (ex. 00)")
    a = ap.parse_args()
    if a.pas == "pas1":
        pas1()
        return
    SHEET = traseu_cub.SHEET = Sheet()
    post("beep", {"n": 2})
    view = zone(a.pas, a.doar_vede, (int(a.in_gheara[0]), int(a.in_gheara[1])) if a.in_gheara else None)
    if a.pas == "pas2" and not a.doar_vede:  # din stânga în dreapta, în lungul lui r1 (10,12 cm)
        post("move_joints", {"angles": VIEWS[ZONES["pas3"][1][0]]["sent"], "time_ms": 2500})
        say("Brațul a trecut în dreapta; urmează pasul 3.")
    else:
        back_to_center(view)
        say("Brațul e în punctul de referință.")
    post("beep", {"n": 1})
    state = load_state()
    colors = json.load(open(MAP))["cells"]
    missing = [k for k in sorted(colors) if k not in state["filled"]]
    say("Pe caroiaj: " + (", ".join(
        f"{k} {COLORS.get(v['color'], v['color'])}" + ("" if v["good"] is not False else " (nu e BINE)")
        + ("" if colors.get(k) == v["color"] else " (ALTĂ CULOARE decât pe hartă)")
        for k, v in sorted(state["filled"].items())) or "nimic")
        + f"; rămase de pus după hartă: {', '.join(f'{k} {COLORS[colors[k]]}' for k in missing) or 'niciuna'}")


if __name__ == "__main__":
    main()
