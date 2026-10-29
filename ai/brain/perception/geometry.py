"""Colțuri, validarea patrulaterului, omografia și scorul de structură al grilei."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from brain.configuration.models import GeometryThresholds
from brain.perception.photometry import chroma

REASON_NOT_CONVEX = "quad_not_convex"
REASON_TOO_SMALL = "grid_too_small"
REASON_TOUCHES_BORDER = "grid_touches_border"
REASON_PERSPECTIVE = "perspective_too_strong"
REASON_ASPECT = "aspect_out_of_range"
REASON_ANGLE = "angle_out_of_range"
REASON_ILL_CONDITIONED = "homography_ill_conditioned"
REASON_STRUCTURE = "grid_structure_weak"
REASON_LINE_UNSUPPORTED = "grid_line_unsupported"
REASON_UNVERIFIABLE = "grid_structure_unverifiable"


def order_corners(points: np.ndarray) -> np.ndarray:
    """Ordonează 4 puncte TL, TR, BR, BL (sens orar pe ecran). Presupune rotație a grilei sub ±45°."""
    pts = np.asarray(points, dtype=np.float64).reshape(4, 2)
    center = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    pts = pts[np.argsort(angles)]
    start = int(np.argmin(pts.sum(axis=1)))
    return np.roll(pts, -start, axis=0)


@dataclass(frozen=True)
class QuadMetrics:
    area: float
    sides: Tuple[float, float, float, float]
    opposite_ratio: float
    adjacent_ratio: float
    angles_deg: Tuple[float, float, float, float]
    convex: bool


def quad_metrics(corners: np.ndarray) -> QuadMetrics:
    pts = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    sides = tuple(float(np.linalg.norm(pts[(i + 1) % 4] - pts[i])) for i in range(4))
    eps = 1e-9
    opposite = max(max(sides[0], sides[2]) / max(min(sides[0], sides[2]), eps),
                   max(sides[1], sides[3]) / max(min(sides[1], sides[3]), eps))
    mean_a = (sides[0] + sides[2]) / 2.0
    mean_b = (sides[1] + sides[3]) / 2.0
    adjacent = max(mean_a, mean_b) / max(min(mean_a, mean_b), eps)
    angles = []
    crosses = []
    for i in range(4):
        prev_vec = pts[i - 1] - pts[i]
        next_vec = pts[(i + 1) % 4] - pts[i]
        denom = max(np.linalg.norm(prev_vec) * np.linalg.norm(next_vec), eps)
        cos_a = float(np.clip(np.dot(prev_vec, next_vec) / denom, -1.0, 1.0))
        angles.append(math.degrees(math.acos(cos_a)))
        edge = pts[(i + 1) % 4] - pts[i]
        nxt = pts[(i + 2) % 4] - pts[(i + 1) % 4]
        crosses.append(edge[0] * nxt[1] - edge[1] * nxt[0])
    convex = all(c > 0 for c in crosses) or all(c < 0 for c in crosses)
    area = float(abs(cv2.contourArea(pts.astype(np.float32))))
    return QuadMetrics(area, sides, float(opposite), float(adjacent), tuple(angles), convex)  # type: ignore[arg-type]


def validate_quad(corners: np.ndarray, image_shape: Sequence[int], thresholds: GeometryThresholds) -> List[str]:
    """Motivele pentru care patrulaterul nu e o grilă plauzibilă; listă goală = valid."""
    metrics = quad_metrics(corners)
    height, width = int(image_shape[0]), int(image_shape[1])
    reasons: List[str] = []
    if not metrics.convex or metrics.area <= 1.0:
        return [REASON_NOT_CONVEX]
    if metrics.area < thresholds.min_area_fraction * width * height:
        reasons.append(REASON_TOO_SMALL)
    margin = thresholds.border_margin_px
    pts = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    if np.any(pts[:, 0] < margin) or np.any(pts[:, 1] < margin) or np.any(pts[:, 0] > width - 1 - margin) or np.any(
        pts[:, 1] > height - 1 - margin
    ):
        reasons.append(REASON_TOUCHES_BORDER)
    if metrics.opposite_ratio > thresholds.max_opposite_side_ratio:
        reasons.append(REASON_PERSPECTIVE)
    if metrics.adjacent_ratio > thresholds.max_adjacent_side_ratio:
        reasons.append(REASON_ASPECT)
    if any(a < thresholds.min_interior_angle_deg or a > thresholds.max_interior_angle_deg for a in metrics.angles_deg):
        reasons.append(REASON_ANGLE)
    return reasons


def fit_homography(src: np.ndarray, dst: np.ndarray) -> Optional[np.ndarray]:
    """Omografie exactă pentru 4 puncte, cele mai mici pătrate pentru mai multe; `None` dacă e degenerată."""
    src_pts = np.asarray(src, dtype=np.float64).reshape(-1, 2)
    dst_pts = np.asarray(dst, dtype=np.float64).reshape(-1, 2)
    if src_pts.shape[0] < 4 or src_pts.shape != dst_pts.shape:
        return None
    if src_pts.shape[0] == 4:
        homography = cv2.getPerspectiveTransform(src_pts.astype(np.float32), dst_pts.astype(np.float32)).astype(np.float64)
    else:
        homography, _ = cv2.findHomography(src_pts, dst_pts, 0)
    if homography is None or not np.all(np.isfinite(homography)) or abs(np.linalg.det(homography)) < 1e-12:
        return None
    return homography


def _normalization(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    center = pts.mean(axis=0)
    mean_dist = np.mean(np.linalg.norm(pts - center, axis=1))
    scale = math.sqrt(2.0) / max(mean_dist, 1e-9)
    return np.array([[scale, 0, -scale * center[0]], [0, scale, -scale * center[1]], [0, 0, 1]], dtype=np.float64)


def normalized_condition_number(homography: np.ndarray, src: np.ndarray, dst: np.ndarray) -> float:
    """Numărul de condiționare după normalizarea Hartley (independent de rezoluție); mare = degenerat."""
    t_src = _normalization(src)
    t_dst = _normalization(dst)
    hn = t_dst @ homography @ np.linalg.inv(t_src)
    if abs(hn[2, 2]) < 1e-12:
        return float("inf")
    hn = hn / hn[2, 2]
    return float(np.linalg.cond(hn))


def transform_points(homography: np.ndarray, points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, homography).reshape(-1, 2)


def warp(image: np.ndarray, homography: np.ndarray, size: Tuple[int, int]) -> np.ndarray:
    return cv2.warpPerspective(image, homography, size, flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)


def odd_block_size(value: float) -> int:
    size = max(3, int(round(value)))
    return size if size % 2 == 1 else size + 1


def dark_mask(image: np.ndarray, block_size: int, offset: float, chroma_max: float) -> np.ndarray:
    """Pixeli mult mai întunecați decât media locală și necolorați (liniile negre ale grilei).

    Canalul V = max(B,G,R) face cuburile saturate luminoase, deci nu sunt confundate cu liniile.
    """
    value = image.max(axis=2)
    dark = cv2.adaptiveThreshold(
        value, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, odd_block_size(block_size), float(offset)
    )
    return np.where(chroma(image) <= chroma_max, dark, 0).astype(np.uint8)


@dataclass(frozen=True)
class StructureReport:
    """`score` = acoperire medie pe liniile așteptate minus acoperirea între linii, în [0, 1].

    `line_supports`: pentru fiecare linie măsurabilă, fracția din lungimea ei neacoperită de cuburi
    pe care apare tuș. O grilă estimată greșit are cel puțin o linie care trece prin masă goală.
    """

    score: float
    line_supports: Tuple[float, ...]
    lines_expected: int

    @property
    def min_line_support(self) -> float:
        return min(self.line_supports) if self.line_supports else 0.0


def _local_level(gray: np.ndarray, candidates: np.ndarray, tiles: int) -> np.ndarray:
    """Nivelul local al mesei: mediana pixelilor candidați pe dale, interpolată la rezoluția imaginii."""
    height, width = gray.shape
    fallback = float(np.median(gray[candidates])) if candidates.any() else float(np.median(gray))
    grid = np.full((tiles, tiles), fallback, dtype=np.float32)
    for i in range(tiles):
        for j in range(tiles):
            ys = slice(i * height // tiles, (i + 1) * height // tiles)
            xs = slice(j * width // tiles, (j + 1) * width // tiles)
            values = gray[ys, xs][candidates[ys, xs]]
            if values.size >= 30:
                grid[i, j] = float(np.median(values))
    return cv2.resize(grid, (width, height), interpolation=cv2.INTER_LINEAR)


def structure_score(
    image: np.ndarray,
    homography: np.ndarray,
    rows: int,
    cols: int,
    cell_px: int,
    offset: float,
    chroma_max: float,
    saturation_min: int,
    value_min: int,
) -> StructureReport:
    """Cât de bine apar liniile grilei la pozițiile prezise de omografie.

    O poziție de pe o linie se verifică doar dacă ambele părți ale liniei sunt masă vizibilă; acolo
    trebuie să existe tuș. Masa = alb necolorat, la nivelul local de luminozitate (umbrele și fețele
    laterale sunt mai întunecate), în afara cuburilor (blob-urile colorate cu găurile umplute, ca
    interiorul alb al fețelor cu pictogramă să nu treacă drept masă). Redresarea se face cu margine,
    ca liniile de contur să aibă și partea exterioară vizibilă.
    """
    margin = int(round(0.2 * cell_px))
    shift = np.array([[1.0, 0.0, margin], [0.0, 1.0, margin], [0.0, 0.0, 1.0]])
    width, height = cols * cell_px + 2 * margin, rows * cell_px + 2 * margin
    rectified = warp(image, shift @ homography, (width, height))
    dark = dark_mask(rectified, cell_px * 0.6, offset, chroma_max) > 0
    gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(rectified, cv2.COLOR_BGR2HSV)
    colored = ((hsv[..., 1] >= saturation_min) & (hsv[..., 2] >= value_min)).astype(np.uint8) * 255
    contours = cv2.findContours(colored, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cubes = np.zeros_like(colored)
    cv2.drawContours(cubes, contours, -1, 255, thickness=-1)
    cubes = cv2.dilate(cubes, np.ones((5, 5), np.uint8)) > 0
    candidates = (chroma(rectified) <= 45) & ~dark & ~cubes
    level = _local_level(gray, candidates, tiles=6)
    table = candidates & (gray >= 0.9 * level) & (gray <= 1.1 * level)
    half = max(3, int(round(cell_px * 0.05)))
    side = half + max(3, int(round(cell_px * 0.04)))

    def support(position: int, vertical: bool, span: Tuple[int, int]) -> Optional[float]:
        lo, hi = position - half, position + half + 1
        a, b = span
        if vertical:
            hit = dark[a:b, lo:hi].any(axis=1)
            valid = table[a:b, position - side] & table[a:b, position + side]
        else:
            hit = dark[lo:hi, a:b].any(axis=0)
            valid = table[position - side, a:b] & table[position + side, a:b]
        if valid.sum() < 0.1 * valid.size:
            return None
        return float(hit[valid].mean())

    x_span = (margin, margin + cols * cell_px)
    y_span = (margin, margin + rows * cell_px)
    on: List[float] = []
    off: List[float] = []
    for k in range(cols + 1):
        value = support(margin + k * cell_px, True, y_span)
        if value is not None:
            on.append(value)
    for k in range(rows + 1):
        value = support(margin + k * cell_px, False, x_span)
        if value is not None:
            on.append(value)
    for k in range(cols):
        value = support(margin + int((k + 0.5) * cell_px), True, y_span)
        if value is not None:
            off.append(value)
    for k in range(rows):
        value = support(margin + int((k + 0.5) * cell_px), False, x_span)
        if value is not None:
            off.append(value)
    expected = rows + cols + 2
    if not on:
        return StructureReport(0.0, (), expected)
    score = float(np.mean(on)) - (float(np.mean(off)) if off else 0.0)
    return StructureReport(float(min(1.0, max(0.0, score))), tuple(on), expected)
