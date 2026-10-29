"""
Modelul unui caroiaj 3x3 in sistemul robotului + citire/scriere calib.json.

Coordonate "grila": u = coloana continua (0..3), v = linia continua (0..3).
Celula (r, c) (1..3) are centrul la u = c - 0.5, v = r - 0.5.
"""
import json
import math
import os

import numpy as np

import config as C


def parse_label(lab):
    """'r1c3' -> (1, 3)"""
    lab = lab.lower().replace(" ", "")
    return int(lab[1]), int(lab[3])


class Grid:
    def __init__(self, O, ecol, erow, z):
        self.O = np.array(O, float)       # centrul celulei R1C1 (x, y)
        self.ecol = np.array(ecol, float) # deplasarea de la o coloana la urmatoarea
        self.erow = np.array(erow, float) # deplasarea de la o linie la urmatoarea
        self.z = float(z)                 # inaltimea de prindere a unui cub de pe masa

    @classmethod
    def from_points(cls, pts):
        """pts: {"r1c1": [x, y, z], ...} (minim 3 celule necoliniare)."""
        A, X, Y, Z = [], [], [], []
        for lab, p in pts.items():
            r, c = parse_label(lab)
            A.append([1.0, c - 1.0, r - 1.0])
            X.append(p[0]); Y.append(p[1]); Z.append(p[2])
        A = np.array(A)
        if len(A) < 3 or np.linalg.matrix_rank(A) < 3:
            raise ValueError("trebuie cel putin 3 celule necoliniare (ex. r1c1, r1c3, r3c1)")
        sx = np.linalg.lstsq(A, np.array(X), rcond=None)[0]
        sy = np.linalg.lstsq(A, np.array(Y), rcond=None)[0]
        g = cls((sx[0], sy[0]), (sx[1], sy[1]), (sx[2], sy[2]), float(np.mean(Z)) + C.GRASP_DZ)
        res = []
        for lab, p in pts.items():
            r, c = parse_label(lab)
            q = g.cell_xy(r, c)
            res.append((lab, math.hypot(q[0] - p[0], q[1] - p[1]), p[2]))
        return g, res

    def uv_to_xy(self, u, v):
        return self.O + (u - 0.5) * self.ecol + (v - 0.5) * self.erow

    def cell_xy(self, r, c):
        return self.uv_to_xy(c - 0.5, r - 0.5)

    def yaw_robot(self, yaw_img_deg):
        """Unghi masurat in imaginea rectificata (fata de axa coloanelor) -> unghi robot."""
        a = math.radians(yaw_img_deg)
        v = math.cos(a) * self.ecol + math.sin(a) * self.erow
        return math.degrees(math.atan2(v[1], v[0]))

    def info(self):
        pc, pr = np.linalg.norm(self.ecol), np.linalg.norm(self.erow)
        cosang = float(np.dot(self.ecol, self.erow) / (pc * pr))
        return (f"pas coloane {pc:.1f} mm, pas linii {pr:.1f} mm, "
                f"unghi intre axe {math.degrees(math.acos(max(-1, min(1, cosang)))):.1f} grd, "
                f"z prindere {self.z:.1f} mm")


# ------------------------------------------------------------------ calib.json
def load_calib(path=C.CALIB_FILE):
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return json.load(f)


def save_calib(calib, path=C.CALIB_FILE):
    with open(path, "w") as f:
        json.dump(calib, f, indent=2)


def grid_from_calib(calib, name):
    try:
        pts = calib[name]["points"]
    except KeyError:
        raise SystemExit(f"Caroiajul {name} nu are celule calibrate. Rulati: python3 calibrate.py celule {name}")
    return Grid.from_points(pts)[0]


def require(calib, name, *keys):
    for k in keys:
        if k not in calib.get(name, {}):
            cmd = {"obs": "poza", "corners": "poza", "points": "celule"}[k]
            raise SystemExit(f"Lipseste calibrarea '{k}' pentru {name}. Rulati: python3 calibrate.py {cmd} {name}")
