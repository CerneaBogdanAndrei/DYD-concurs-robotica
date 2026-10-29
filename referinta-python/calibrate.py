#!/usr/bin/env python3
"""
Calibrare (se face o singura data, in laborator, cu caroiajele fixate pe masa).

  python3 calibrate.py poza A        # poza de observare + click pe cele 4 colturi ale caroiajului A
  python3 calibrate.py poza B
  python3 calibrate.py celule A      # invatare manuala: gheara pe 4 celule -> pozitia caroiajului
  python3 calibrate.py celule B
  python3 calibrate.py culori A      # previzualizare live a recunoasterii (ajustare praguri HSV)
  python3 calibrate.py test A 2 3    # bratul coboara pe celula R2C3 (verificare precizie)
  python3 calibrate.py citeste       # afiseaza unghiurile curente (cu cuplul oprit)

Pentru poza/culori e nevoie de ecran (monitor sau VNC) pe Raspberry Pi.
"""
import argparse
import sys
import time

import cv2
import numpy as np

import config as C
from dofbot import Arm, fk
from grid import Grid, load_calib, save_calib, require, grid_from_calib
from vision import Camera, refine_corners, warp, analyze, draw_overlay, format_layout

LABELS_COLTURI = ["coltul exterior R1C1", "coltul exterior R1C3",
                  "coltul exterior R3C3", "coltul exterior R3C1"]


def teach(arm, mesaj):
    """Opreste cuplul, utilizatorul pozitioneaza bratul cu mana, citeste unghiurile."""
    arm.torque(False)
    input(f"\n>>> {mesaj}\n    Apasati Enter cand e gata (tineti bratul nemiscat)... ")
    s = arm.read()
    arm.torque(True)
    if any(v is None for v in s[:5]):
        print("Citire esuata, reincercati.", s)
        return teach(arm, mesaj)
    arm.write6(s, 300)  # tine pozitia
    return s


def click_corners(img):
    pts = []

    def cb(ev, x, y, *_):
        if ev == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append([x, y])

    cv2.namedWindow("calib")
    cv2.setMouseCallback("calib", cb)
    while len(pts) < 4:
        vis = img.copy()
        for i, p in enumerate(pts):
            cv2.circle(vis, tuple(p), 5, (0, 0, 255), -1)
            cv2.putText(vis, str(i + 1), (p[0] + 6, p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(vis, f"Click: {LABELS_COLTURI[len(pts)]}  (u=anulare, Esc=iesire)",
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        cv2.imshow("calib", vis)
        k = cv2.waitKey(20) & 0xFF
        if k == 27:
            sys.exit("anulat")
        if k == ord("u") and pts:
            pts.pop()
    return pts


def cmd_poza(args):
    arm, cam = Arm(), Camera()
    calib = load_calib()
    s = teach(arm, f"Asezati bratul astfel incat camera sa vada TOT caroiajul {args.grila}, "
                   "cat mai de sus si cat mai perpendicular.")
    obs = [int(v) for v in s[:5]]
    arm.pose(obs, 500)
    time.sleep(C.SETTLE_S)
    frame = cam.grab()
    pts = click_corners(frame)
    w = warp(frame, pts)
    cv2.imshow("rectificat (verificati: R1C1 stanga-sus)", draw_overlay(w, analyze(w)))
    print("Verificati imaginea rectificata. Orice tasta = salvare, Esc = renuntare")
    if (cv2.waitKey(0) & 0xFF) == 27:
        sys.exit("nesalvat")
    calib.setdefault(args.grila, {})
    calib[args.grila]["obs"] = obs
    calib[args.grila]["corners"] = pts
    save_calib(calib)
    print(f"Salvat: poza de observare {obs} si colturile pentru {args.grila}.")


def cmd_celule(args):
    arm = Arm()
    calib = load_calib()
    labels = args.celule or ["r1c1", "r1c3", "r3c1", "r3c3"]
    pts = {}
    for lab in labels:
        s = teach(arm, f"Puneti un cub in celula {lab.upper()} a caroiajului {args.grila} (centrat). "
                       "Aduceti gheara DESCHISA in jurul lui, la jumatatea inaltimii cubului, "
                       "cat mai VERTICALA.")
        x, y, z, p = fk(s)
        pts[lab] = [x, y, z]
        print(f"    {lab}: servo={s[:5]} -> x={x:.1f} y={y:.1f} z={z:.1f} mm, inclinare {p:.0f} grd")
        if not (-15 < z < 60):
            print("    ATENTIE: z ar trebui sa fie ~ jumatate din latura cubului. "
                  "Verificati L1/L4 din config.py.")
        arm.pose(C.HOME, 1000)
    g, res = Grid.from_points(pts)
    print("\n" + g.info())
    for lab, err, _ in res:
        print(f"    eroare de potrivire {lab}: {err:.1f} mm")
    calib.setdefault(args.grila, {})["points"] = pts
    save_calib(calib)
    print(f"Salvat caroiajul {args.grila}.")


def cmd_culori(args):
    arm, cam = Arm(), Camera()
    calib = load_calib()
    require(calib, args.grila, "obs", "corners")
    arm.pose(calib[args.grila]["obs"])
    time.sleep(C.SETTLE_S)
    state = {"hsv": None, "pos": (0, 0)}

    def cb(ev, x, y, *_):
        state["pos"] = (x, y)

    cv2.namedWindow("culori")
    cv2.setMouseCallback("culori", cb)
    print("Mutati mouse-ul peste cuburi pentru valorile HSV. q = iesire")
    while True:
        frame = cam.grab(n=1, flush=1)
        corners, ok = refine_corners(frame, calib[args.grila]["corners"])
        w = warp(frame, corners)
        cells = analyze(w)
        vis = draw_overlay(w, cells)
        x, y = state["pos"]
        if 0 <= x < w.shape[1] and 0 <= y < w.shape[0]:
            h, s, v = cv2.cvtColor(w, cv2.COLOR_BGR2HSV)[y, x]
            cv2.putText(vis, f"HSV {h} {s} {v}  {'auto' if ok else 'calib'}", (5, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(vis, f"HSV {h} {s} {v}  {'auto' if ok else 'calib'}", (5, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow("culori", vis)
        if (cv2.waitKey(30) & 0xFF) == ord("q"):
            break
    print(format_layout(cells))


def cmd_test(args):
    arm = Arm()
    calib = load_calib()
    g = grid_from_calib(calib, args.grila)
    x, y = g.cell_xy(args.r, args.c)
    yaw = g.yaw_robot(0)
    print(f"R{args.r}C{args.c}: x={x:.1f} y={y:.1f} z={g.z:.1f}")
    arm.gripper(C.GRIP_OPEN)
    arm.goto(x, y, g.z + C.HOVER_DZ, yaw)
    arm.goto(x, y, g.z, yaw, t_ms=C.T_FINE * 2)
    input("Gheara trebuie sa fie centrata pe celula si paralela cu liniile. Enter = sus ")
    arm.goto(x, y, g.z + C.HOVER_DZ, yaw, t_ms=C.T_FINE)
    arm.home()


def cmd_citeste(args):
    arm = Arm()
    arm.torque(False)
    print("Cuplu oprit. Miscati bratul; Ctrl+C pentru iesire.")
    try:
        while True:
            s = arm.read()
            if None not in s[:4]:
                x, y, z, p = fk(s)
                print(f"\rservo {s}  ->  x={x:6.1f} y={y:6.1f} z={z:6.1f} pitch={p:5.0f}   ", end="")
            time.sleep(0.3)
    except KeyboardInterrupt:
        arm.torque(True)
        print()


def main():
    ap = argparse.ArgumentParser(description="Calibrare Dofbot")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("poza"); p.add_argument("grila", choices=["A", "B"])
    p = sub.add_parser("celule"); p.add_argument("grila", choices=["A", "B"])
    p.add_argument("--celule", nargs="+", help="ex: r1c1 r1c3 r3c1 r3c3 r2c2")
    p = sub.add_parser("culori"); p.add_argument("grila", choices=["A", "B"])
    p = sub.add_parser("test"); p.add_argument("grila", choices=["A", "B"])
    p.add_argument("r", type=int); p.add_argument("c", type=int)
    sub.add_parser("citeste")
    args = ap.parse_args()
    {"poza": cmd_poza, "celule": cmd_celule, "culori": cmd_culori,
     "test": cmd_test, "citeste": cmd_citeste}[args.cmd](args)


if __name__ == "__main__":
    main()
