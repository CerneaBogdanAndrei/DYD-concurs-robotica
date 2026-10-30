"""Vederile sortării (stânga, dreapta, sub caroiaj): camera calibrată pe liniile caroiajului care se văd, iar cuburile
găsite după fața de sus colorată și duse în milimetrii foii.

Obiectivul e cel din `mutare_foaie.py` (F, C, K1); punctele se lucrează fără distorsiune, cu aceeași matrice K.
(2026-10-30: f, centrul, k1, k2 reestimate din cele 8 vederi se potriveau mai bine pe linii, f ~900-955 px, dar
dădeau un cub văzut din două vederi mai împrăștiat, 0,7-2,5 mm, față de 0,4-1,2 mm cu acesta.)
Foaia: x spre c2 de la marginea stângă a caroiajului, y spre robot de la marginea depărtată, z = x × y (în masă):
deasupra foii z e negativ, fața de sus a unui cub e la z = -29 mm.
"""
import math
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from mutare_foaie import C, COL, F, K1, ROOT, ROW  # noqa: E402

sys.path.insert(0, str(ROOT / "ai"))
from brain.configuration.loader import load_app_config  # noqa: E402
from brain.perception.color import HueClassifier  # noqa: E402

K = np.array([[F, 0, C[0]], [0, F, C[1]], [0, 0, 1.0]])
DIST = np.array([K1, 0, 0, 0, 0.0])
CUBE_MM = 29.0
LINE_DARK = 40  # cât de mult mai închisă decât hârtia din jur e o linie (black-hat, nivele de gri)
HUE = HueClassifier(load_app_config(ROOT / "ai/config").colors)


def undistort_points(px):
    return cv2.undistortPoints(np.float64(px).reshape(-1, 1, 2), K, DIST, P=K).reshape(-1, 2)


def distort_points(px):
    """Inversul lui `undistort_points`: pixeli fără distorsiune -> imaginea brută (k1, k2 radiale)."""
    p = np.float64(px).reshape(-1, 2)
    f, c = K[0, 0], K[:2, 2]
    n = (p - c) / f
    r2 = (n ** 2).sum(1, keepdims=True)
    return n * (1 + DIST[0] * r2 + DIST[1] * r2 ** 2) * f + c


# ------------------------------------------------------------------ liniile caroiajului


def _line_mask(und):
    g = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)
    bh = cv2.morphologyEx(g, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11)))
    sat = cv2.cvtColor(und, cv2.COLOR_BGR2HSV)[..., 1]
    return ((bh > LINE_DARK) & (sat < 80)).astype(np.uint8) * 255


def _fit(pts):
    """Dreapta prin puncte: (punct, direcție unitară)."""
    vx, vy, x0, y0 = cv2.fitLine(np.float32(pts), cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(4)
    return np.array([x0, y0]), np.array([vx, vy])


def lines(und):
    """Liniile drepte închise la culoare din imaginea fără distorsiune, în două familii după direcție.
    -> (familia A, familia B): liste de (punct, direcție, puncte), fiecare cu ~90 grd între ele."""
    mask = _line_mask(und)
    segs = cv2.HoughLinesP(mask, 1, np.pi / 360, 50, minLineLength=50, maxLineGap=10)
    if segs is None:
        return [], []
    segs = segs.reshape(-1, 4).astype(np.float64)
    ang = np.degrees(np.arctan2(segs[:, 3] - segs[:, 1], segs[:, 2] - segs[:, 0])) % 180.0
    ys, xs = np.nonzero(mask)
    pix = np.stack([xs, ys], 1).astype(np.float64)
    # direcția dominantă (unghiul dublat, ca 0 și 180 să fie la fel), ponderată cu lungimea
    w = np.hypot(segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1])
    a2 = np.radians(2 * ang)
    best = None
    for k in range(180):  # caută direcția d care împarte segmentele în d și d + 90 cel mai bine
        d = math.radians(2 * k)
        score = (w * np.cos(a2 - d) ** 2).sum()
        if best is None or score > best[0]:
            best = (score, k)
    fam = [[], []]
    for s, a in zip(segs, ang):
        diff = abs((a - best[1] + 90) % 180 - 90)
        if diff < 25:
            fam[0].append(s)
        elif diff > 65:
            fam[1].append(s)
    out = []
    for group in fam:
        merged = []  # (punct, direcție) pentru fiecare linie, din segmentele apropiate
        for s in sorted(group, key=lambda s: -math.hypot(s[2] - s[0], s[3] - s[1])):
            p, q = s[:2], s[2:]
            for m in merged:
                n = np.array([-m[1][1], m[1][0]])
                if abs((p - m[0]) @ n) < 8 and abs((q - m[0]) @ n) < 8:
                    break
            else:
                d = (q - p) / np.linalg.norm(q - p)
                merged.append((p, d))
        fitted = []
        for p, d in merged:
            n = np.array([-d[1], d[0]])
            near = pix[np.abs((pix - p) @ n) < 4]
            if len(near) < 40:
                continue
            p, d = _fit(near)
            n = np.array([-d[1], d[0]])
            near = pix[np.abs((pix - p) @ n) < 3]
            p, d = _fit(near)
            if not any(abs((p - f[0]) @ np.array([-f[1][1], f[1][0]])) < 6 for f in fitted):
                fitted.append((p, d, near))
        out.append(fitted)
    return out[0], out[1]


def crossing(l1, l2):
    (p, d), (q, e) = l1[:2], l2[:2]
    t = np.linalg.solve(np.c_[d, -e], q - p)[0]
    return p + t * d


def visible_crossing(l1, l2, gap=25.0):
    """Intersecția, doar dacă ambele linii au puncte detectate la cel mult `gap` px de ea."""
    x = crossing(l1, l2)
    if all(np.min(np.linalg.norm(l[2] - x, axis=1)) <= gap for l in (l1, l2)):
        return x
    return None


# ------------------------------------------------------------------ camera


class Camera:
    """Poza camerei față de foaie (rvec, tvec: foaie -> cameră), pe puncte fără distorsiune."""

    def __init__(self, rvec, tvec):
        self.rvec, self.tvec = np.float64(rvec).reshape(3), np.float64(tvec).reshape(3)
        self.R = cv2.Rodrigues(self.rvec)[0]
        self.center = -self.R.T @ self.tvec

    @classmethod
    def fit(cls, obj_mm, img_px, guess=None):
        obj = np.c_[np.float64(obj_mm), np.zeros(len(obj_mm))]
        img = np.float64(img_px)
        if guess is not None:
            ok, rvec, tvec = cv2.solvePnP(obj, img, K, None, guess.rvec.copy(), guess.tvec.copy(), True,
                                          cv2.SOLVEPNP_ITERATIVE)
        else:
            ok, rvec, tvec = cv2.solvePnP(obj, img, K, None, flags=cv2.SOLVEPNP_IPPE)
        cam = cls(rvec, tvec)
        res = np.linalg.norm(cam.project(obj) - img, axis=1)
        return cam, float(np.sqrt((res ** 2).mean())), res

    def project(self, pts3):
        return cv2.projectPoints(np.float64(pts3).reshape(-1, 3), self.rvec, self.tvec, K, None)[0].reshape(-1, 2)

    def to_sheet(self, px, z=0.0):
        """Pixeli fără distorsiune -> punctele din planul z al foii (mm)."""
        p = np.float64(px).reshape(-1, 2)
        rays = (self.R.T @ np.c_[(p - K[:2, 2]) / K[0, 0], np.ones(len(p))].T).T
        t = (z - self.center[2]) / rays[:, 2]
        return (self.center + t[:, None] * rays)[:, :2]

    @property
    def height(self):
        return float(-self.center[2])

    @property
    def nadir(self):
        return self.center[:2].copy()

    def as_dict(self):
        return {"rvec": self.rvec.tolist(), "tvec": self.tvec.tolist()}

    @classmethod
    def from_dict(cls, d):
        return cls(d["rvec"], d["tvec"])


# ------------------------------------------------------------------ cuburile


FACE_PX_MM = 2.0  # rezoluția planului fețelor: pixeli pe mm
FACE_VAL_MIN = 35  # fața verde e închisă (V ~58), aproape de pragul viziunii (50): aici ceva mai jos
HUE_GAP = 6  # nuanțele dintre intervalele culorilor (verdele închis ajunge la H 90, intervalul lui se oprește la 88)


def _hue_lut():
    """Clasa fiecărei nuanțe, cu golurile dintre intervale umplute până la HUE_GAP de cel mai apropiat interval."""
    lut = HUE._lut.copy()
    for h in range(180):
        if lut[h] >= 0:
            continue
        best = None
        for k in range(1, HUE_GAP + 1):
            for hh in ((h - k) % 180, (h + k) % 180):
                if HUE._lut[hh] >= 0:
                    best = HUE._lut[hh]
                    break
            if best is not None:
                break
        if best is not None:
            lut[h] = best
    return lut


LUT = _hue_lut()


def cube_tops(raw, cam, min_area=1500, open_mm=8.0):
    """Fețele de sus colorate din imaginea brută, în planul fețelor (z = -29 mm).

    Fiecare pată de culoare se proiectează pe planul fețelor (FACE_PX_MM pixeli pe mm); acolo o închidere umple
    golurile și o deschidere cu un disc de `open_mm` taie fâșiile subțiri (marginea colorată a unei laturi, văzută
    din lateral). -> listă de dicționare: culoare (votul pe nuanțele viziunii), centru și laturi (mm), rotire (grd,
    față de axele foii, -45..45), umplere, tăiată (atinge marginea imaginii), conturul (px în imaginea brută)."""
    hsv = cv2.cvtColor(raw, cv2.COLOR_BGR2HSV)
    colored = (hsv[..., 1] >= HUE.config.saturation_min) & (hsv[..., 2] >= FACE_VAL_MIN)
    classes = np.where(colored, LUT[hsv[..., 0].astype(np.intp)], -1)
    h, w = raw.shape[:2]
    out = []
    for idx in range(len(HUE.colors)):
        m = (classes == idx).astype(np.uint8) * 255
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, stats, _ = cv2.connectedComponentsWithStats(cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8)))
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] < min_area:
                continue
            cnt = cv2.findContours((lab == i).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
            cnt = max(cnt, key=cv2.contourArea).reshape(-1, 2)
            mm = cam.to_sheet(undistort_points(cnt), -CUBE_MM)
            x0, y0 = mm.min(0) - 10.0
            x1, y1 = mm.max(0) + 10.0
            if x1 - x0 > 150 or y1 - y0 > 150:  # nu e o față de cub (sau camera e greșită)
                continue
            xs = np.arange(x0, x1, 1.0 / FACE_PX_MM)
            ys = np.arange(y0, y1, 1.0 / FACE_PX_MM)
            gx, gy = np.meshgrid(xs, ys)
            pts = np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, -CUBE_MM)]
            px = distort_points(cam.project(pts)).reshape(gx.shape + (2,)).astype(np.float32)
            inside = ((px[..., 0] >= 0) & (px[..., 0] <= w - 1) & (px[..., 1] >= 0) & (px[..., 1] <= h - 1))
            face = cv2.remap(((lab == i) & (m > 0)).astype(np.uint8) * 255, px[..., 0], px[..., 1],
                             cv2.INTER_NEAREST, borderValue=0)
            k = lambda mm_: cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (int(mm_ * FACE_PX_MM) | 1,) * 2)
            face = cv2.morphologyEx(face, cv2.MORPH_CLOSE, k(3.0))
            filled = np.zeros_like(face)
            cv2.drawContours(filled, cv2.findContours(face, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], -1, 255, -1)
            face = cv2.morphologyEx(filled, cv2.MORPH_OPEN, k(open_mm))
            fc = cv2.findContours(face, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
            if not fc:
                continue
            fc = max(fc, key=cv2.contourArea)
            (cx, cy), (sw, sh), a = cv2.minAreaRect(fc)
            area = cv2.contourArea(fc)
            region = np.zeros_like(face)
            cv2.drawContours(region, [fc], -1, 255, -1)
            border = cv2.dilate(region, k(2.0)) > 0
            clipped = bool((border & ~inside).any())
            # culoarea: votul viziunii pe pixelii feței (nuanțele originale, fără golurile umplute)
            sel = cv2.remap(hsv, px[..., 0], px[..., 1], cv2.INTER_NEAREST)[region > 0]
            vote = HUE.vote(sel[(sel[:, 1] >= HUE.config.saturation_min) & (sel[:, 2] >= FACE_VAL_MIN)], 50)
            to_mm = lambda v: v / FACE_PX_MM
            face_px = distort_points(cam.project(np.c_[x0 + to_mm(fc[:, 0, 0]), y0 + to_mm(fc[:, 0, 1]),
                                                       np.full(len(fc), -CUBE_MM)]))
            out.append({"color": vote.color.value if vote.color.value != "unknown" else HUE.colors[idx].value,
                        "vote": round(vote.dominance, 2),
                        "center_mm": (float(x0 + to_mm(cx)), float(y0 + to_mm(cy))),
                        "sides_mm": (float(to_mm(sw)), float(to_mm(sh))),
                        "yaw_deg": float((a + 45.0) % 90.0 - 45.0),
                        "fill": float(area / max(sw * sh, 1e-6)), "clipped": clipped,
                        "contour_px": face_px.tolist()})
    return out


def outside_gap_mm(center):
    """Cât de departe e centrul unui cub de marginea caroiajului (0 dacă e pe caroiaj)."""
    x, y = center
    dx = max(0.0, -x, x - 3 * COL)
    dy = max(0.0, -y, y - 3 * ROW)
    return math.hypot(dx, dy)
