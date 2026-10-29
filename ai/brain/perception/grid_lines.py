"""Detecția caroiajului după liniile negre (metoda principală, fără markeri).

1. Masca pixelilor întunecați și necolorați (liniile).
2. Patrulater grosier: înfășurătoarea convexă a componentei principale + fragmentele alungite din jur
   (cuburile de pe margine pot rupe conturul în bucăți).
3. Rafinare iterativă: imaginea măștii e redresată cu margine, iar în profilele de proiecție
   (pe coloane și pe rânduri) se caută `cols+1` / `rows+1` linii echidistante. Intersecțiile lor dau
   puncte de control pentru o omografie prin cele mai mici pătrate. Profilele agregă fiecare linie pe
   toată lungimea ei, deci un colț sau o porțiune acoperită de un cub nu strică estimarea.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from brain.configuration.models import LineDetectionConfig
from brain.perception.geometry import dark_mask, order_corners

REASON_NOT_FOUND = "grid_not_found"
REASON_LINES_NOT_FOUND = "grid_lines_not_found"

PROFILE_UNIT_PX = 50  # px per celulă în imaginea de rafinare
REFINE_ITERATIONS = 3
GROWTH_PAD = 0.15  # fracție din latura componentei principale


@dataclass(frozen=True)
class GridDetection:
    """`image_points` (px) corespund lui `grid_points` (unități de celulă: coloană, rând).

    Punctele sunt intersecțiile centrelor liniilor: (0,0) = colțul stânga-sus al grilei.
    """

    ok: bool
    image_points: Optional[np.ndarray]
    grid_points: Optional[np.ndarray]
    reasons: Tuple[str, ...]
    metrics: Dict[str, float] = field(default_factory=dict)


def detect_grid_lines(
    image: np.ndarray, config: LineDetectionConfig, rows: int, cols: int, roi: Optional[np.ndarray] = None
) -> List[GridDetection]:
    """Candidații pentru grila țintă; validarea (în pipeline) alege candidatul corect.

    Dacă în cadru se văd mai multe șabloane (aceeași foaie A4): `roi` (patrulater în px, de ex. colțurile
    așteptate din calibrare, dilatate) restrânge căutarea; fără `roi`, sămânța e componenta cea mai
    apropiată de centrul imaginii (poza de observare centrează șablonul țintă). Fragmentele se adună
    în două moduri (iterativ — grile aglomerate, rupte de cuburi; o singură trecere — șabloane vecine
    apropiate), iar fiecare variantă e rafinată separat.
    """
    height, width = image.shape[:2]
    mask = dark_mask(image, min(height, width) * config.adaptive_block_fraction, config.adaptive_c, config.dark_chroma_max)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    if roi is not None:
        region = np.zeros_like(mask)
        cv2.fillConvexPoly(region, np.rint(np.asarray(roi, dtype=np.float64)).astype(np.int32).reshape(-1, 2), 255)
        mask = cv2.bitwise_and(mask, region)
    candidates: List[GridDetection] = []
    seen: List[np.ndarray] = []
    starts = []
    for iterative in (True, False):
        coarse = _coarse_quad(mask, config, iterative)
        if coarse is None:
            continue
        quad, hull = coarse
        starts.append((quad, hull))
        # Dreptunghiul minim al înfășurătoarei: robust când mai multe colțuri sunt acoperite de cuburi.
        starts.append((order_corners(cv2.boxPoints(cv2.minAreaRect(hull)).astype(np.float64)), hull))
    for quad, hull in starts:
        if any(float(np.max(np.linalg.norm(quad - other, axis=1))) < 3.0 for other in seen):
            continue
        seen.append(quad)
        # Rafinarea vede doar regiunea candidatului (înfășurătoarea mărită cu 20%), nu și restul foii.
        center = hull.reshape(-1, 2).mean(axis=0)
        region = np.zeros_like(mask)
        grown = (hull.reshape(-1, 2) - center) * 1.2 + center
        cv2.fillConvexPoly(region, np.rint(grown).astype(np.int32), 255)
        candidate_mask = cv2.bitwise_and(mask, region)
        refined = _refine_by_profiles(candidate_mask, quad, rows, cols)
        if refined is None:
            candidates.append(GridDetection(False, None, None, (REASON_LINES_NOT_FOUND,)))
            continue
        # A doua trecere pornește din colțurile rafinate: de la un punct de pornire îndepărtat (dreptunghiul
        # minim poate fi la zeci de px de grilă) prima trecere nu converge complet. Rămâne reziduul mai mic.
        polished = _refine_by_profiles(candidate_mask, corners_from_points(refined[0], rows, cols), rows, cols)
        if polished is not None and polished[2] < refined[2]:
            refined = polished
        image_points, grid_points, residual = refined
        candidates.append(GridDetection(True, image_points, grid_points, (), {"line_residual_cells": round(residual, 4)}))
    if not candidates:
        return [GridDetection(False, None, None, (REASON_NOT_FOUND,))]
    return candidates


def _component_points(labels: np.ndarray, stats: np.ndarray, label: int) -> np.ndarray:
    x, y = int(stats[label, cv2.CC_STAT_LEFT]), int(stats[label, cv2.CC_STAT_TOP])
    w, h = int(stats[label, cv2.CC_STAT_WIDTH]), int(stats[label, cv2.CC_STAT_HEIGHT])
    ys, xs = np.nonzero(labels[y:y + h, x:x + w] == label)
    return np.column_stack([xs + x, ys + y]).astype(np.float32)


def _coarse_quad(
    mask: np.ndarray, config: LineDetectionConfig, iterative: bool = True
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    height, width = mask.shape
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    min_area = config.min_component_fraction * height * width
    large = [label for label in range(1, count) if stats[label, cv2.CC_STAT_AREA] >= min_area]
    if not large:
        return None
    extents = {
        label: int(stats[label, cv2.CC_STAT_WIDTH]) * int(stats[label, cv2.CC_STAT_HEIGHT]) for label in large
    }
    biggest = max(extents.values())
    # Sămânța: componenta mare cea mai apropiată de centrul imaginii (nu doar cea mai mare: pe foaia A4
    # se pot vedea și celelalte șabloane, iar poza de observare centrează șablonul țintă).
    center = np.array([width / 2.0, height / 2.0])

    def distance(label: int) -> float:
        x, y = stats[label, cv2.CC_STAT_LEFT], stats[label, cv2.CC_STAT_TOP]
        w, h = stats[label, cv2.CC_STAT_WIDTH], stats[label, cv2.CC_STAT_HEIGHT]
        return float(np.linalg.norm(np.array([x + w / 2.0, y + h / 2.0]) - center))

    main = min((label for label in large if extents[label] >= 0.3 * biggest), key=distance)
    # Cuburile pot rupe grila în mai multe bucăți (cruci, L-uri, segmente). Uniunea crește iterativ cu
    # componentele apropiate care nu sunt compacte; formele compacte sunt pictogramele de pe cuburi.
    small_min = max(15.0, 0.05 * min_area)
    pending = {}
    for label in range(1, count):
        if label == main or stats[label, cv2.CC_STAT_AREA] < small_min:
            continue
        pts = _component_points(labels, stats, label)
        (_, _), (rw, rh), _ = cv2.minAreaRect(pts)
        elongation = max(rw, rh) / max(min(rw, rh), 1.0)
        fill = len(pts) / max(rw * rh, 1.0)
        if elongation < 2.0 and fill > 0.4:
            continue
        pending[label] = pts
    points = [_component_points(labels, stats, main)]
    box = [float(stats[main, cv2.CC_STAT_LEFT]), float(stats[main, cv2.CC_STAT_TOP])]
    box += [box[0] + float(stats[main, cv2.CC_STAT_WIDTH]), box[1] + float(stats[main, cv2.CC_STAT_HEIGHT])]
    if iterative:
        # Grilă ruptă de cuburi în mai multe bucăți: uniunea crește cât timp găsește fragmente apropiate.
        grown = True
        while grown and pending:
            grown = False
            pad = 0.3 * max(box[2] - box[0], box[3] - box[1])
            for label in list(pending):
                x, y = float(stats[label, cv2.CC_STAT_LEFT]), float(stats[label, cv2.CC_STAT_TOP])
                w, h = float(stats[label, cv2.CC_STAT_WIDTH]), float(stats[label, cv2.CC_STAT_HEIGHT])
                if x > box[2] + pad or y > box[3] + pad or x + w < box[0] - pad or y + h < box[1] - pad:
                    continue
                points.append(pending.pop(label))
                box = [min(box[0], x), min(box[1], y), max(box[2], x + w), max(box[3], y + h)]
                grown = True
    else:
        # O singură trecere față de cutia componentei principale: un șablon vecin apropiat rămâne în afară.
        pad = GROWTH_PAD * max(box[2] - box[0], box[3] - box[1])
        for label, pts in pending.items():
            x, y = float(stats[label, cv2.CC_STAT_LEFT]), float(stats[label, cv2.CC_STAT_TOP])
            w, h = float(stats[label, cv2.CC_STAT_WIDTH]), float(stats[label, cv2.CC_STAT_HEIGHT])
            cx, cy = x + w / 2.0, y + h / 2.0
            if box[0] - pad <= cx <= box[2] + pad and box[1] - pad <= cy <= box[3] + pad:
                points.append(pts)
    hull = cv2.convexHull(np.vstack(points))
    quad = _enclosing_quad(hull)
    if quad is None:
        quad = cv2.boxPoints(cv2.minAreaRect(hull)).astype(np.float64)
    return order_corners(quad), hull


def _intersect(p1: np.ndarray, d1: np.ndarray, p2: np.ndarray, d2: np.ndarray) -> Optional[np.ndarray]:
    """Intersecția dreptelor p1 + s·d1 și p2 + t·d2; `None` dacă sunt (aproape) paralele."""
    det = d1[0] * (-d2[1]) + d2[0] * d1[1]
    if abs(det) < 1e-9 * max(1.0, float(np.linalg.norm(d1) * np.linalg.norm(d2))):
        return None
    rhs = p2 - p1
    s = (rhs[0] * (-d2[1]) + d2[0] * rhs[1]) / det
    return p1 + s * d1


def _enclosing_quad(hull: np.ndarray) -> Optional[np.ndarray]:
    """Patrulater care înconjoară înfășurătoarea, obținut eliminând pe rând latura a cărei eliminare
    (prin prelungirea vecinelor până se intersectează) adaugă cea mai mică arie.

    Un colț teșit de o ocluzie e o latură scurtă între două laturi lungi: eliminarea ei reface colțul.
    """
    perimeter = cv2.arcLength(hull, True)
    if perimeter <= 0:
        return None
    poly = [p for p in cv2.approxPolyDP(hull, 0.005 * perimeter, True).reshape(-1, 2).astype(np.float64)]
    while len(poly) > 4:
        n = len(poly)
        best_area = float("inf")
        best: Optional[Tuple[int, np.ndarray]] = None
        for i in range(n):
            p_prev, p_a, p_b, p_next = poly[i - 1], poly[i], poly[(i + 1) % n], poly[(i + 2) % n]
            d_a = p_a - p_prev
            d_b = p_b - p_next
            point = _intersect(p_a, d_a, p_b, d_b)
            if point is None:
                continue
            # Intersecția trebuie să fie în prelungirea ambelor laturi vecine (în exterior).
            if np.dot(point - p_a, d_a) < 0 or np.dot(point - p_b, d_b) < 0:
                continue
            area = 0.5 * abs((p_a[0] - point[0]) * (p_b[1] - point[1]) - (p_b[0] - point[0]) * (p_a[1] - point[1]))
            if area < best_area:
                best_area, best = area, (i, point)
        if best is None:
            return None
        index, point = best
        if index + 1 < len(poly):
            poly[index:index + 2] = [point]
        else:  # latura dintre ultimul și primul vârf
            poly = [point] + poly[1:-1]
    if len(poly) < 4:
        return None
    return np.array(poly)


def _refine_by_profiles(
    mask: np.ndarray, quad: np.ndarray, rows: int, cols: int
) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
    unit = PROFILE_UNIT_PX
    grid_w, grid_h = cols * unit, rows * unit
    off_x, off_y = grid_w, grid_h  # marginea: o grilă întreagă pe fiecare parte
    size = (grid_w + 2 * off_x, grid_h + 2 * off_y)
    target_quad = np.array(
        [[off_x, off_y], [off_x + grid_w, off_y], [off_x + grid_w, off_y + grid_h], [off_x, off_y + grid_h]],
        dtype=np.float32,
    )
    homography = cv2.getPerspectiveTransform(quad.astype(np.float32), target_quad)
    grid_units = np.array([(i, j) for j in range(rows + 1) for i in range(cols + 1)], dtype=np.float64)
    targets = np.column_stack([off_x + grid_units[:, 0] * unit, off_y + grid_units[:, 1] * unit])
    residual = float("inf")
    band: Optional[Tuple[slice, slice]] = None
    for iteration in range(REFINE_ITERATIONS):
        warped = cv2.warpPerspective(mask, homography, size, flags=cv2.INTER_NEAREST, borderValue=0) > 0
        rows_slice, cols_slice = band if band is not None else (slice(None), slice(None))
        strict = iteration > 0
        xs = _find_line_positions(warped[rows_slice, :].mean(axis=0), cols, unit, off_x, strict)
        ys = _find_line_positions(warped[:, cols_slice].mean(axis=1), rows, unit, off_y, strict)
        if xs is None or ys is None:
            return None
        x_targets = [off_x + i * unit for i in range(cols + 1)]
        y_targets = [off_y + j * unit for j in range(rows + 1)]
        fitted = _fit_line_correction(warped, xs, ys, x_targets, y_targets, unit, wide=iteration == 0)
        if fitted is None:
            return None
        correction, residual = fitted
        homography = correction @ homography
        homography /= homography[2, 2]
        pad = unit // 4
        band = (slice(off_y - pad, off_y + grid_h + pad), slice(off_x - pad, off_x + grid_w + pad))
    image_points = cv2.perspectiveTransform(targets.reshape(-1, 1, 2), np.linalg.inv(homography)).reshape(-1, 2)
    return image_points, grid_units, residual / unit


def _fit_line_correction(
    warped: np.ndarray,
    xs: Sequence[float],
    ys: Sequence[float],
    x_targets: Sequence[float],
    y_targets: Sequence[float],
    unit: float,
    wide: bool,
) -> Optional[Tuple[np.ndarray, float]]:
    """Omografia de corecție (canvas → canvas) care aduce pixelii fiecărei linii pe poziția ei țintă.

    Un pixel al liniei verticale i dă ecuația x' = X_i, adică liniar în parametri (ca la DLT):
    h11·x + h12·y + h13 − X_i·(h31·x + h32·y) = X_i. Analog pentru liniile orizontale. Sistemul se
    rezolvă prin cele mai mici pătrate cu ponderi Huber (robust la pixeli străini: pictograme, umbre).
    Întoarce corecția și reziduul RMS (px canvas) al pixelilor de linie.
    """
    yy, xx = np.nonzero(warped)
    px = xx.astype(np.float64)
    py = yy.astype(np.float64)
    half_band = (0.2 if wide else 0.1) * unit
    pad = 0.15 * unit
    x_lo, x_hi = min(xs) - pad, max(xs) + pad
    y_lo, y_hi = min(ys) - pad, max(ys) + pad
    rows_eq: List[np.ndarray] = []
    rhs: List[np.ndarray] = []
    center = np.array([(x_lo + x_hi) / 2.0, (y_lo + y_hi) / 2.0])
    scale = 1.0 / unit
    for position, target in zip(xs, x_targets):
        sel = (np.abs(px - position) <= half_band) & (py >= y_lo) & (py <= y_hi)
        if sel.sum() < 10:
            continue
        x = (px[sel] - center[0]) * scale
        y = (py[sel] - center[1]) * scale
        t = (target - center[0]) * scale
        zeros = np.zeros_like(x)
        rows_eq.append(np.column_stack([x, y, np.ones_like(x), zeros, zeros, zeros, -t * x, -t * y]))
        rhs.append(np.full_like(x, t))
    vertical_eqs = len(rows_eq)
    for position, target in zip(ys, y_targets):
        sel = (np.abs(py - position) <= half_band) & (px >= x_lo) & (px <= x_hi)
        if sel.sum() < 10:
            continue
        x = (px[sel] - center[0]) * scale
        y = (py[sel] - center[1]) * scale
        t = (target - center[1]) * scale
        zeros = np.zeros_like(x)
        rows_eq.append(np.column_stack([zeros, zeros, zeros, x, y, np.ones_like(x), -t * x, -t * y]))
        rhs.append(np.full_like(x, t))
    # Fiecare familie are nevoie de cel puțin două linii pentru a fixa perspectiva pe acea axă.
    if vertical_eqs < 2 or len(rows_eq) - vertical_eqs < 2:
        return None
    a = np.vstack(rows_eq)
    b = np.concatenate(rhs)
    weights = np.ones_like(b)
    huber = 1.5 * scale  # ~ jumătate din grosimea liniei, în unități normalizate
    params = None
    for _ in range(4):
        sw = np.sqrt(weights)
        params, *_ = np.linalg.lstsq(a * sw[:, None], b * sw, rcond=None)
        residuals = np.abs(a @ params - b)
        weights = np.where(residuals <= huber, 1.0, huber / np.maximum(residuals, 1e-12))
    assert params is not None
    norm = np.array([[scale, 0, -scale * center[0]], [0, scale, -scale * center[1]], [0, 0, 1]])
    h_norm = np.array([[params[0], params[1], params[2]], [params[3], params[4], params[5]], [params[6], params[7], 1.0]])
    correction = np.linalg.inv(norm) @ h_norm @ norm
    if not np.all(np.isfinite(correction)) or abs(np.linalg.det(correction)) < 1e-12:
        return None
    rms = float(np.sqrt(np.mean((a @ params - b) ** 2))) / scale
    return correction, rms


def _find_line_positions(
    profile: np.ndarray, n: int, unit: float, start_prior: float, strict: bool
) -> Optional[List[float]]:
    """Pozițiile a `n+1` linii echidistante în profil.

    Oricare două vârfuri pot fi liniile i < j ale grilei; restul pozițiilor se verifică. E tolerată o
    singură linie lipsă (inclusiv de capăt, atunci e extrapolată). Priorul (start ≈ `start_prior`,
    pas ≈ `unit`) departajează tiparele cu scor egal; `strict` îl transformă în constrângere.
    """
    smooth = np.convolve(profile, np.ones(5) / 5.0, mode="same")
    floor = max(0.03, 0.15 * float(smooth.max()))
    candidates: List[Tuple[float, float]] = []
    for i in range(1, len(smooth) - 1):
        if smooth[i] >= floor and smooth[i] >= smooth[i - 1] and smooth[i] > smooth[i + 1]:
            lo, hi = max(0, i - 4), min(len(profile), i + 5)
            weights = profile[lo:hi]
            position = float((np.arange(lo, hi) * weights).sum() / weights.sum()) if weights.sum() > 0 else float(i)
            if candidates and position - candidates[-1][0] < 4.0:
                if smooth[i] > candidates[-1][1]:
                    candidates[-1] = (position, float(smooth[i]))
                continue
            candidates.append((position, float(smooth[i])))
    if len(candidates) < 2:
        return None
    positions = np.array([c[0] for c in candidates])
    heights = np.array([c[1] for c in candidates])
    mean_height = float(heights.mean())
    best: Optional[Tuple[float, List[float]]] = None
    for a_index in range(len(candidates)):
        for b_index in range(a_index + 1, len(candidates)):
            for i in range(n):
                for j in range(i + 1, n + 1):
                    step = (positions[b_index] - positions[a_index]) / (j - i)
                    start = positions[a_index] - i * step
                    if strict:
                        if abs(step - unit) > 0.15 * unit or abs(start - start_prior) > 0.3 * unit:
                            continue
                    elif not 0.3 * unit <= step <= 3.2 * unit:
                        continue
                    tolerance = max(2.0, 0.12 * step)
                    matched: List[float] = []
                    score = 0.0
                    missing = 0
                    for k in range(n + 1):
                        expected = start + k * step
                        nearest = int(np.argmin(np.abs(positions - expected)))
                        if abs(positions[nearest] - expected) <= tolerance:
                            matched.append(float(positions[nearest]))
                            score += float(heights[nearest])
                        else:
                            matched.append(float(expected))
                            missing += 1
                    if missing > 1:
                        continue
                    score -= missing * 0.5 * mean_height
                    score -= 0.3 * mean_height * (abs(start - start_prior) + n * abs(step - unit)) / unit
                    if best is None or score > best[0]:
                        best = (score, matched)
    return best[1] if best is not None else None


def corners_from_points(image_points: Sequence[Sequence[float]], rows: int, cols: int) -> np.ndarray:
    """Colțurile TL, TR, BR, BL din punctele ordonate row-major ale intersecțiilor."""
    pts = np.asarray(image_points, dtype=np.float64).reshape(rows + 1, cols + 1, 2)
    return np.array([pts[0, 0], pts[0, cols], pts[rows, cols], pts[rows, 0]])
