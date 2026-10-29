#!/usr/bin/env python3
"""
Rulare concurs.

  python3 run.py s1                                   # Sarcina 1: copiaza aranjarea A -> B
  python3 run.py s2 --ordine rosu,galben,albastru,verde --pozitie 2,2
  python3 run.py scan A                               # doar recunoastere (debug)

Optiuni: --sursa/--dest (s1), --grila (s2), --afisare (ferestre OpenCV),
         --sim --img-A a.jpg --img-B b.jpg (fara brat/camera, pentru teste pe laptop)
"""
import argparse
import math
import os
import re
import time
import unicodedata

import cv2

import config as C
from dofbot import Arm, Unreachable
from grid import load_calib, grid_from_calib, require
from vision import Camera, refine_corners, warp, analyze, draw_overlay, format_layout

SHOW = False
T0 = time.time()


def log(msg):
    print(f"[{time.time() - T0:6.1f}s] {msg}", flush=True)


def save_dbg(name, img):
    os.makedirs(C.DEBUG_DIR, exist_ok=True)
    cv2.imwrite(os.path.join(C.DEBUG_DIR, f"{name}.jpg"), img)
    if SHOW:
        cv2.imshow(name, img)
        cv2.waitKey(1)


# ------------------------------------------------------------------ recunoastere
def scan(arm, cam, calib, g, tag):
    require(calib, g, "obs", "corners")
    arm.pose(calib[g]["obs"])
    time.sleep(0 if arm.sim else C.SETTLE_S)
    frame = cam.grab(tag=g)
    corners, auto = refine_corners(frame, calib[g]["corners"])
    w = warp(frame, corners)
    cells = analyze(w)
    raw = frame.copy()
    cv2.polylines(raw, [corners.astype("int32")], True, (0, 255, 0) if auto else (0, 165, 255), 2)
    save_dbg(f"{tag}_cadru", raw)
    save_dbg(f"{tag}_grila", draw_overlay(w, cells))
    log(f"scanare {g} ({'contur detectat' if auto else 'colturi din calibrare'}): {len(cells)} cuburi")
    print(format_layout(cells, f"--- caroiaj {g} ---"), flush=True)
    return cells


def dist_xy(p):
    return math.hypot(p[0], p[1])


# ------------------------------------------------------------------ sarcina 1
def sarcina1(arm, cam, calib, src, dst):
    gS, gD = grid_from_calib(calib, src), grid_from_calib(calib, dst)
    layout = scan(arm, cam, calib, src, "s1_sursa")
    if not layout:
        log("Nu am gasit niciun cub pe caroiajul sursa.")
        return
    ocupat = scan(arm, cam, calib, dst, "s1_dest_inainte")
    if ocupat:
        log(f"ATENTIE: caroiajul destinatie nu e gol: {sorted(ocupat)}")

    todo = list(layout.values())
    for incercare in range(2):
        # intai cuburile apropiate de baza (incheietura nu trece peste cuburi ramase)
        todo.sort(key=lambda d: dist_xy(gS.uv_to_xy(d["u"], d["v"])))
        for d in todo:
            x, y = gS.uv_to_xy(d["u"], d["v"])
            bx, by = gD.cell_xy(d["r"], d["c"])
            log(f"R{d['r']}C{d['c']} {d['culoare']}/{d['fata']}: ({x:.0f},{y:.0f}) -> ({bx:.0f},{by:.0f})")
            try:
                arm.pick(x, y, gS.z, gS.yaw_robot(d["yaw"]))
                arm.place(bx, by, gD.z, gD.yaw_robot(0))
            except Unreachable as e:
                log(f"  sarit: {e}")
        ramase = scan(arm, cam, calib, src, f"s1_sursa_dupa{incercare + 1}")
        todo = [d for k, d in ramase.items() if k in layout]
        if not todo:
            break
        log(f"Reincerc {len(todo)} cuburi ramase.")

    final = scan(arm, cam, calib, dst, "s1_dest_final")
    ok = 0
    for k, d in sorted(layout.items()):
        f = final.get(k)
        good = f is not None and f["culoare"] == d["culoare"] and f["fata"] == d["fata"]
        ok += good
        got = "gol" if f is None else f"{f['culoare']}/{f['fata']}"
        log(f"  R{k[0]}C{k[1]}: asteptat {d['culoare']}/{d['fata']}, gasit {got} -> {'OK' if good else 'GRESIT'}")
    extra = [k for k in final if k not in layout]
    if extra:
        log(f"  cuburi in plus pe destinatie: {extra}")
    log(f"Sarcina 1: {ok}/{len(layout)} corecte.")


# ------------------------------------------------------------------ sarcina 2
def normalize(s):
    s = unicodedata.normalize("NFD", s.lower())
    return "".join(ch for ch in s if unicodedata.category(ch) != "Mn")


def parse_ordine(s):
    out = []
    for tok in re.split(r"[,\s>→;-]+", normalize(s)):
        if not tok:
            continue
        if tok not in C.ALIAS:
            raise SystemExit(f"Culoare necunoscuta: '{tok}'. Folositi: rosu, galben, verde, albastru")
        out.append(C.ALIAS[tok])
    return out


def parse_pozitie(s):
    nums = [int(n) for n in re.findall(r"\d", s)]
    if len(nums) != 2 or not all(1 <= n <= 3 for n in nums):
        raise SystemExit("Pozitia se da ca linie,coloana, ex: 2,3 sau R2C3")
    return tuple(nums)


def sarcina2(arm, cam, calib, g, ordine, tinta):
    grid = grid_from_calib(calib, g)
    cells = scan(arm, cam, calib, g, "s2_initial")
    cul = {}
    for d in cells.values():
        cul.setdefault(d["culoare"], []).append(d)
    log("Culori identificate: " + ", ".join(f"{k} x{len(v)}" for k, v in sorted(cul.items())))
    log("Ordine turn (jos -> sus): " + " -> ".join(ordine) + f"; pozitie R{tinta[0]}C{tinta[1]}")

    tx, ty = grid.cell_xy(*tinta)
    yaw0 = grid.yaw_robot(0)
    disponibile = {k: d for k, d in cells.items() if k != tinta}
    nivel, start = 0, 0

    ocupant = cells.get(tinta)
    if ocupant is not None:
        if ordine and ocupant["culoare"] == ordine[0]:
            log("Cubul de baza e deja pe pozitia tinta.")
            nivel, start = 1, 1
        else:
            libere = [(r, c) for r in range(1, 4) for c in range(1, 4)
                      if (r, c) not in cells and (r, c) != tinta]
            if not libere:
                raise SystemExit("Pozitia tinta e ocupata si nu exista celula libera pentru mutare.")
            # cea mai departata celula libera de tinta
            libere.sort(key=lambda rc: -math.hypot(rc[0] - tinta[0], rc[1] - tinta[1]))
            dest = libere[0]
            log(f"Mut cubul {ocupant['culoare']} de pe tinta in R{dest[0]}C{dest[1]}.")
            arm.pick(*grid.uv_to_xy(ocupant["u"], ocupant["v"]), grid.z, grid.yaw_robot(ocupant["yaw"]))
            dx, dy = grid.cell_xy(*dest)
            arm.place(dx, dy, grid.z, yaw0)
            mutat = dict(ocupant, r=dest[0], c=dest[1], u=dest[1] - 0.5, v=dest[0] - 0.5, yaw=0.0)
            disponibile[dest] = mutat

    for culoare in ordine[start:]:
        cand = [d for d in disponibile.values() if d["culoare"] == culoare]
        if not cand:
            log(f"LIPSESTE un cub {culoare}; trec mai departe.")
            continue
        # cel mai departat de turn intai: gheara deschisa nu atinge turnul
        cand.sort(key=lambda d: -math.hypot(d["r"] - tinta[0], d["c"] - tinta[1]))
        d = cand[0]
        del disponibile[(d["r"], d["c"])]
        x, y = grid.uv_to_xy(d["u"], d["v"])
        log(f"Nivel {nivel + 1}: {culoare} din R{d['r']}C{d['c']}")
        arm.pick(x, y, grid.z, grid.yaw_robot(d["yaw"]))
        arm.place(tx, ty, grid.z + nivel * C.CUBE_MM, yaw0)
        nivel += 1
    log(f"Turn construit: {nivel} niveluri.")


# ------------------------------------------------------------------ main
def main():
    global SHOW
    ap = argparse.ArgumentParser(description="Concurs Dofbot")
    ap.add_argument("sarcina", choices=["s1", "s2", "scan"])
    ap.add_argument("grila_scan", nargs="?", default="A")
    ap.add_argument("--sursa", default="A")
    ap.add_argument("--dest", default="B")
    ap.add_argument("--grila", default="A", help="caroiajul pe care se construieste turnul")
    ap.add_argument("--ordine", default="rosu,galben,albastru,verde", help="de jos in sus")
    ap.add_argument("--pozitie", default="2,2", help="linie,coloana pentru turn")
    ap.add_argument("--afisare", action="store_true")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--img-A"); ap.add_argument("--img-B")
    args = ap.parse_args()
    SHOW = args.afisare and bool(os.environ.get("DISPLAY"))

    calib = load_calib()
    arm = Arm(sim=args.sim)
    cam = Camera({"A": args.img_A, "B": args.img_B} if args.sim else None)
    arm.beep(1)
    try:
        if args.sarcina == "scan":
            scan(arm, cam, calib, args.grila_scan, f"scan_{args.grila_scan}")
        elif args.sarcina == "s1":
            sarcina1(arm, cam, calib, args.sursa, args.dest)
        else:
            sarcina2(arm, cam, calib, args.grila, parse_ordine(args.ordine), parse_pozitie(args.pozitie))
    finally:
        if arm.grip != C.GRIP_OPEN:
            arm.gripper(C.GRIP_OPEN)
        arm.home()
        cam.release()
        arm.beep(3)
        log("Gata.")
        if SHOW:
            cv2.waitKey(0)


if __name__ == "__main__":
    main()
