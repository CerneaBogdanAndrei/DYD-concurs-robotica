"""Liniile caroiajului 3x3 măsurate în imagine, cu precizie sub-pixel (pornind de la colțurile exterioare).

Fiecare linie (4 verticale, 4 orizontale) e urmărită pe profilul întunecat perpendicular pe ea și ajustată cu o
parabolă; intersecțiile (4 x 4) vin din liniile ajustate. O linie acoperită de un cub (prea puține puncte) lipsește,
iar intersecțiile ei sunt `None`. Dimensiunile celulelor în mm sunt măsurate față de conturul exterior, considerat
`cols x cell_mm` pe `rows x cell_mm` (omografia celor 4 colțuri): arată unde sunt liniile interioare.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

_UV = np.float32([[0, 0], [3, 0], [3, 3], [0, 3]])


def _bilinear(g: np.ndarray, pts: np.ndarray) -> np.ndarray:
    x, y = pts[..., 0], pts[..., 1]
    h, w = g.shape
    x0 = np.clip(np.floor(x).astype(int), 0, w - 2)
    y0 = np.clip(np.floor(y).astype(int), 0, h - 2)
    fx, fy = x - x0, y - y0
    return (g[y0, x0] * (1 - fx) * (1 - fy) + g[y0, x0 + 1] * fx * (1 - fy)
            + g[y0 + 1, x0] * (1 - fx) * fy + g[y0 + 1, x0 + 1] * fx * fy)


def measure_lines(image_bgr: np.ndarray, corners_px: Sequence[Sequence[float]], half: float = 10.0,
                  min_points: int = 15) -> Dict[str, object]:
    """-> {"points": 4x4 de (x, y) sau None (rând = linia v, coloană = linia u), "rms": {("u"|"v", k): px}}."""
    g = cv2.GaussianBlur(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY).astype(np.float64), (3, 3), 0)
    H = cv2.getPerspectiveTransform(_UV, np.float32(corners_px))

    def proj(uv: np.ndarray) -> np.ndarray:
        return cv2.perspectiveTransform(np.float64(uv).reshape(-1, 1, 2), H).reshape(-1, 2)

    s = np.arange(-half, half + 0.01, 0.5)
    fits: Dict[Tuple[str, int], np.ndarray] = {}
    rms: Dict[Tuple[str, int], float] = {}
    for vertical in (True, False):
        for k in range(4):
            ts = np.linspace(0.06, 2.94, 60)
            ts = ts[np.abs(ts - np.round(ts)) > 0.12]  # departe de intersecții
            uv = np.stack([np.full_like(ts, k), ts], 1) if vertical else np.stack([ts, np.full_like(ts, k)], 1)
            p, p2 = proj(uv), proj(uv + (np.array([0, 0.01]) if vertical else np.array([0.01, 0])))
            d = p2 - p
            d /= np.linalg.norm(d, axis=1, keepdims=True)
            n = np.stack([-d[:, 1], d[:, 0]], 1)
            prof = _bilinear(g, p[:, None, :] + s[None, :, None] * n[:, None, :])
            base = np.median(np.concatenate([prof[:, :4], prof[:, -4:]], 1), axis=1, keepdims=True)
            dark = np.clip(base - prof, 0, None)
            peak = dark.max(1)
            w = np.where(dark >= 0.3 * peak[:, None], dark, 0)
            off = (w * s).sum(1) / np.maximum(w.sum(1), 1e-9)
            pts = (p + off[:, None] * n)[peak > 25]
            if len(pts) < min_points:
                continue
            a, b = (0, 1) if vertical else (1, 0)  # verticală: x = f(y); orizontală: y = f(x)
            coef = np.polyfit(pts[:, b], pts[:, a], 2)
            res = pts[:, a] - np.polyval(coef, pts[:, b])
            keep = np.abs(res) < max(3 * res.std(), 0.5)
            if keep.sum() < min_points:
                continue
            coef = np.polyfit(pts[keep, b], pts[keep, a], 2)
            res = pts[keep, a] - np.polyval(coef, pts[keep, b])
            key = ("u" if vertical else "v", k)
            fits[key] = coef
            rms[key] = float(np.sqrt((res ** 2).mean()))
    points: List[List[Optional[Tuple[float, float]]]] = [[None] * 4 for _ in range(4)]
    for j in range(4):
        for i in range(4):
            if ("u", i) not in fits or ("v", j) not in fits:
                continue
            x, y = proj(np.array([[i, j]]))[0]
            for _ in range(20):
                x = np.polyval(fits[("u", i)], y)
                y = np.polyval(fits[("v", j)], x)
            points[j][i] = (float(x), float(y))
    return {"points": points, "rms": rms}


def cell_sizes_mm(points: List[List[Optional[Tuple[float, float]]]], cell_mm: float
                  ) -> Optional[List[List[Optional[Tuple[float, float]]]]]:
    """Lățimea x înălțimea fiecărei celule (mm, media laturilor opuse), față de conturul exterior de 3 x cell_mm."""
    outer = [points[0][0], points[0][3], points[3][3], points[3][0]]
    if any(p is None for p in outer):
        return None
    H = cv2.getPerspectiveTransform(np.float32(outer), np.float32(_UV * cell_mm))

    def mm(p: Tuple[float, float]) -> np.ndarray:
        return cv2.perspectiveTransform(np.float64([[p]]), H).reshape(2)

    out: List[List[Optional[Tuple[float, float]]]] = [[None] * 3 for _ in range(3)]
    for r in range(3):
        for c in range(3):
            q = [points[r][c], points[r][c + 1], points[r + 1][c + 1], points[r + 1][c]]
            if any(p is None for p in q):
                continue
            a, b, cc, d = (mm(p) for p in q)
            out[r][c] = ((np.linalg.norm(b - a) + np.linalg.norm(cc - d)) / 2.0,
                         (np.linalg.norm(d - a) + np.linalg.norm(cc - b)) / 2.0)
    return out
