"""
Viziune: captura, rectificarea caroiajului, detectia cuburilor si a culorii fetei de sus.

Ideea pentru cuburile Yahboom: fata de sus e fie plina (o culoare), fie o imagine pe
fond alb cu RAMA colorata. Culoarea se decide din rama (inelul exterior al fetei),
nu din interior, ca desenul (doza portocalie, mar rosu etc.) sa nu pacaleasca.
"""
import math

import cv2
import numpy as np

import config as C


class Camera:
    def __init__(self, sim_images=None):
        self.sim_images = sim_images or {}
        self.cap = None
        if not sim_images:
            self.cap = cv2.VideoCapture(C.CAM_INDEX)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, C.CAM_W)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, C.CAM_H)
            if not self.cap.isOpened():
                raise RuntimeError("Camera nu poate fi deschisa (verificati CAM_INDEX / cablul USB)")

    def grab(self, tag=None, n=C.FRAMES_AVG, flush=4):
        if self.sim_images:
            img = cv2.imread(self.sim_images[tag])
            if img is None:
                raise RuntimeError(f"nu pot citi imaginea simulata pentru {tag}")
            return img
        for _ in range(flush):          # aruncam cadrele vechi din buffer
            self.cap.grab()
        acc, k = None, 0
        for _ in range(n):
            ok, f = self.cap.read()
            if not ok:
                continue
            acc = f.astype(np.float32) if acc is None else acc + f
            k += 1
        if k == 0:
            raise RuntimeError("camera nu returneaza cadre")
        return (acc / k).astype(np.uint8)

    def release(self):
        if self.cap is not None:
            self.cap.release()


# ------------------------------------------------------------------ caroiaj
def refine_corners(frame, corners):
    """Cauta conturul exterior al caroiajului langa colturile calibrate.
    Compenseaza micile erori de repozitionare ale bratului. -> (colturi, gasit?)"""
    ref = np.float32(corners)
    ref_area = cv2.contourArea(ref)
    gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    edges = cv2.dilate(cv2.Canny(gray, 50, 150), np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    best, best_err = None, 1e9
    for c in cnts:
        a = cv2.contourArea(c)
        if a < 0.75 * ref_area or a > 1.3 * ref_area:
            continue
        approx = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True)
        if len(approx) != 4:
            continue
        q = approx.reshape(4, 2).astype(np.float32)
        d = np.linalg.norm(ref[:, None, :] - q[None, :, :], axis=2)
        idx = d.argmin(axis=1)
        if len(set(idx.tolist())) < 4:
            continue
        dd = d[np.arange(4), idx]
        if dd.max() > C.REFINE_MAX_PX:
            continue
        if dd.sum() < best_err:
            best_err, best = dd.sum(), q[idx]
    if best is None:
        return ref, False
    return best, True


def warp(frame, corners):
    """corners in ordinea: colt R1C1, colt R1C3, colt R3C3, colt R3C1 (colturile exterioare)."""
    W = C.WARP
    dst = np.float32([[0, 0], [W, 0], [W, W], [0, W]])
    M = cv2.getPerspectiveTransform(np.float32(corners), dst)
    return cv2.warpPerspective(frame, M, (W, W))


# ------------------------------------------------------------------ culori
def color_masks(bgr):
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
    masks = {}
    for name, ranges in C.CULORI.items():
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        masks[name] = m
    return masks


def _box_angle(box):
    p0, p1 = box[0], box[1]
    a = math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))
    return (a + 45.0) % 90.0 - 45.0


def analyze(warped):
    """-> {(r, c): {...}} pentru celulele ocupate."""
    W = warped.shape[0]
    cell = W / 3.0
    masks = color_masks(warped)
    anym = np.zeros(warped.shape[:2], np.uint8)
    for m in masks.values():
        anym |= m
    anym = cv2.morphologyEx(anym, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    anyc = cv2.morphologyEx(anym, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)

    k_par = (C.CAM_HEIGHT_MM - C.CUBE_MM) / C.CAM_HEIGHT_MM  # corectie de paralaxa
    out = {}
    for r in range(3):
        for c in range(3):
            x0, y0 = int(round(c * cell)), int(round(r * cell))
            x1, y1 = int(round((c + 1) * cell)), int(round((r + 1) * cell))
            roi = anyc[y0:y1, x0:x1]
            if cv2.countNonZero(roi) < C.MIN_FRACTIE * roi.size:
                continue
            cnts, _ = cv2.findContours(roi.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnt = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(cnt) < C.MIN_FRACTIE * roi.size:
                continue
            cnt = cnt + np.array([[x0, y0]])
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            (cx, cy), _, _ = rect

            full = np.zeros(anym.shape, np.uint8)
            cv2.fillPoly(full, [np.int32(np.round(box))], 255)
            ibox = (box - [cx, cy]) * C.INNER_SCALE + [cx, cy]
            inner = np.zeros(anym.shape, np.uint8)
            cv2.fillPoly(inner, [np.int32(np.round(ibox))], 255)
            ring = cv2.subtract(full, inner)
            ring_n = max(1, cv2.countNonZero(ring))
            inner_n = max(1, cv2.countNonZero(inner))

            scores = {n: cv2.countNonZero(cv2.bitwise_and(m, ring)) / ring_n for n, m in masks.items()}
            culoare = max(scores, key=scores.get)
            if scores[culoare] < C.MIN_SCOR_CULOARE:
                continue
            fr_in = cv2.countNonZero(cv2.bitwise_and(masks[culoare], inner)) / inner_n
            fata = "plina" if fr_in > C.FATA_PLINA_PRAG else "imagine"

            u, v = cx / cell, cy / cell
            u = 1.5 + (u - 1.5) * k_par
            v = 1.5 + (v - 1.5) * k_par
            out[(r + 1, c + 1)] = {
                "r": r + 1, "c": c + 1, "culoare": culoare, "fata": fata,
                "u": float(u), "v": float(v), "yaw": float(_box_angle(box)),
                "scor": round(float(scores[culoare]), 2), "box": box.tolist(),
            }
    return out


# ------------------------------------------------------------------ afisare
def draw_overlay(warped, cells):
    img = warped.copy()
    W = img.shape[0]
    for i in range(4):
        p = int(round(i * W / 3))
        cv2.line(img, (p, 0), (p, W), (90, 90, 90), 1)
        cv2.line(img, (0, p), (W, p), (90, 90, 90), 1)
    for d in cells.values():
        col = C.CULOARE_BGR.get(d["culoare"], (255, 255, 255))
        cv2.polylines(img, [np.int32(np.round(d["box"]))], True, (0, 0, 0), 3)
        cv2.polylines(img, [np.int32(np.round(d["box"]))], True, col, 2)
        x, y = int(d["u"] * W / 3), int(d["v"] * W / 3)
        cv2.circle(img, (x, y), 4, (0, 0, 0), -1)
        txt = f"{d['culoare'][:3].upper()} {'P' if d['fata'] == 'plina' else 'I'}"
        cv2.putText(img, txt, (x - 28, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(img, txt, (x - 28, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return img


def format_layout(cells, titlu=""):
    rows = [titlu] if titlu else []
    rows.append("        C1              C2              C3")
    for r in range(1, 4):
        line = f"R{r}  "
        for c in range(1, 4):
            d = cells.get((r, c))
            s = "  -  " if d is None else f"{d['culoare']}({'plina' if d['fata'] == 'plina' else 'img'})"
            line += f"{s:<16}"
        rows.append(line)
    return "\n".join(rows)
