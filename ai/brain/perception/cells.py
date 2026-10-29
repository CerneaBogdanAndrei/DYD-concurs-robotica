"""Analiza celulelor în imaginea redresată: prezență, culoare, yaw, tipul feței superioare.

Regiunile colorate (componente conexe) se atribuie celulei care le conține centroidul, deci un
cub deplasat de paralaxă spre celula vecină rămâne în celula corectă. O regiune prea mare
(cuburi lipite în imagine) se separă după nucleele transformatei de distanță, cu încredere redusă.
O celulă fără culoare e declarată goală doar dacă nu are nici muchii, nici obiecte întunecate:
altfel e `unknown` (un cub decolorat nu trebuie raportat drept absent).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from brain.configuration.models import CellAnalysisConfig
from brain.domain.cube import Color, Occupancy, TopFace, normalize_yaw_deg
from brain.domain.grid import GridCoord, iter_coords
from brain.domain.observation import CellObservation
from brain.perception.color import HueClassifier
from brain.perception.photometry import chroma

REASON_EDGES_WITHOUT_COLOR = "edges_without_color"
REASON_DARK_OBJECT = "dark_object_without_color"
REASON_FAINT_COLOR = "faint_color_object"
REASON_AMBIGUOUS_AREA = "ambiguous_colored_area"
REASON_ROI_OCCLUDED = "roi_occluded"
REASON_OFF_CENTER = "off_center"
REASON_SEPARATED = "merged_blob_separated"
REASON_SPLIT = "split_blob"
REASON_OVERSIZED = "oversized_blob"
REASON_YAW_UNRELIABLE = "yaw_unreliable"
REASON_CLIPPED = "blob_clipped_by_image"

OVERSIZED_AREA_FACTOR = 2.2
OVERSIZED_EXTENT_FACTOR = 1.35
WINDOW_MARGIN = 6


@dataclass(frozen=True)
class _Part:
    ys: np.ndarray
    xs: np.ndarray
    split: str  # "" | "separated" | "cut"


@dataclass(frozen=True)
class CellResult:
    observation: CellObservation
    box: Optional[np.ndarray] = None  # minAreaRect în px redresați, pentru adnotare
    centroid: Optional[Tuple[float, float]] = None


def _clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return float(min(high, max(low, value)))


def _fill_holes(mask: np.ndarray) -> np.ndarray:
    """Umple găurile (pictograma albă din mijlocul unei fețe cu bordură colorată)."""
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    filled = np.zeros_like(mask)
    cv2.drawContours(filled, contours, -1, 255, thickness=-1)
    return filled


def dominant_orientation(filled: np.ndarray, exclude: Optional[np.ndarray] = None) -> Tuple[float, float]:
    """Orientarea dominantă a conturului (mod 90°) și concentrarea ei în [0, 1].

    Normalele conturului unui pătrat rotit cu α au unghiurile α + k·90°, deci media circulară a lui
    4θ (ponderată cu gradientul) dă 4α. Fețele laterale vizibile oblic adaugă puține muchii cu alte
    orientări, deci estimarea rămâne stabilă; concentrarea scade când forma nu e un pătrat.
    Întoarce unghiul în convenția imaginii (axa y în jos).
    """
    smooth = cv2.GaussianBlur(filled.astype(np.float32) / 255.0, (0, 0), 1.5)
    gx = cv2.Sobel(smooth, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(smooth, cv2.CV_32F, 0, 1, ksize=3)
    weight = cv2.magnitude(gx, gy)
    if exclude is not None:
        weight = np.where(exclude, 0.0, weight)
    total = float(weight.sum())
    if total <= 1e-9:
        return 0.0, 0.0
    angle = np.arctan2(gy, gx) * 4.0
    c = float((weight * np.cos(angle)).sum())
    s = float((weight * np.sin(angle)).sum())
    return math.degrees(math.atan2(s, c) / 4.0), math.hypot(c, s) / total


class CellAnalyzer:
    def __init__(self, config: CellAnalysisConfig, classifier: HueClassifier, cell_px: int, cube_px: float) -> None:
        self.config = config
        self.classifier = classifier
        self.cell_px = int(cell_px)
        self.cube_px = float(cube_px)
        self.cube_area = self.cube_px ** 2
        self._margin = 0

    def analyze(self, rectified: np.ndarray, rows: int, cols: int, margin: int = 0) -> List[CellResult]:
        """`margin`: px de imagine redresată în jurul grilei (cuburile de pe margine pot depăși conturul)."""
        cfg = self.config
        self._margin = int(margin)
        hsv = cv2.cvtColor(rectified, cv2.COLOR_BGR2HSV)
        mask = self.classifier.colored_mask(hsv)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # Componentele se calculează separat pe clase de nuanță: cuburi vecine de culori diferite nu se
        # contopesc niciodată; pentru aceeași culoare rămâne separarea după nuclee.
        classes = np.where(mask > 0, self.classifier.hue_classes(hsv), -2)
        parts: Dict[Tuple[int, int], List[_Part]] = {}
        for value in np.unique(classes):
            if value == -2:
                continue
            class_mask = np.where(classes == value, 255, 0).astype(np.uint8)
            for key, found in self._assign_parts(class_mask, rows, cols).items():
                parts.setdefault(key, []).extend(found)

        owner = np.full(mask.shape, -1, dtype=np.int32)
        for (row, col), cell_parts in parts.items():
            for part in cell_parts:
                owner[part.ys, part.xs] = row * cols + col

        gray = cv2.GaussianBlur(cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY), (3, 3), 0)
        gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
        edges = cv2.magnitude(gx, gy) > cfg.edge_gradient_threshold
        neutral = chroma(rectified) <= 50

        results: List[CellResult] = []
        for coord in iter_coords(rows, cols):
            index = coord.row * cols + coord.col
            foreign = ((owner >= 0) & (owner != index)).astype(np.uint8)
            foreign = cv2.dilate(foreign, np.ones((7, 7), np.uint8)) > 0
            cell_parts = parts.get((coord.row, coord.col), [])
            results.append(self._analyze_cell(coord, cell_parts, hsv, gray, edges, neutral, foreign, owner, index))
        return results

    def _assign_parts(self, mask: np.ndarray, rows: int, cols: int) -> Dict[Tuple[int, int], List[_Part]]:
        cfg = self.config
        cell = self.cell_px
        margin = self._margin
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        parts: Dict[Tuple[int, int], List[_Part]] = {}

        def add(ys: np.ndarray, xs: np.ndarray, split: str) -> None:
            row, col = int((ys.mean() - margin) // cell), int((xs.mean() - margin) // cell)
            if 0 <= row < rows and 0 <= col < cols:
                parts.setdefault((row, col), []).append(_Part(ys, xs, split))

        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < cfg.min_component_px:
                continue
            ys, xs = np.nonzero(labels == label)
            extent = max(int(stats[label, cv2.CC_STAT_WIDTH]), int(stats[label, cv2.CC_STAT_HEIGHT]))
            if area <= OVERSIZED_AREA_FACTOR * self.cube_area and extent <= OVERSIZED_EXTENT_FACTOR * cell:
                add(ys, xs, "")
                continue
            separated = self._separate(ys, xs)
            if separated is not None:
                for part_ys, part_xs in separated:
                    if len(part_ys) >= cfg.min_component_px:
                        add(part_ys, part_xs, "separated")
                continue
            # Ultima soluție: tăiere după granițele celulelor (forma rezultată nu mai e fiabilă pentru yaw).
            cell_rows, cell_cols = (ys - margin) // cell, (xs - margin) // cell
            for row in range(rows):
                for col in range(cols):
                    selected = (cell_rows == row) & (cell_cols == col)
                    if int(selected.sum()) >= cfg.min_component_px:
                        parts.setdefault((row, col), []).append(_Part(ys[selected], xs[selected], "cut"))
        return parts

    def _separate(self, ys: np.ndarray, xs: np.ndarray) -> Optional[List[Tuple[np.ndarray, np.ndarray]]]:
        """Separă cuburi lipite: nucleele transformatei de distanță, apoi fiecare pixel la nucleul cel mai apropiat."""
        y0, x0 = int(ys.min()), int(xs.min())
        local = np.zeros((int(ys.max()) - y0 + 1, int(xs.max()) - x0 + 1), dtype=np.uint8)
        local[ys - y0, xs - x0] = 255
        # Închiderea repară inelele rupte, ca fețele cu pictogramă să devină pătrate pline înainte de nuclee.
        local = cv2.morphologyEx(cv2.copyMakeBorder(local, 3, 3, 3, 3, cv2.BORDER_CONSTANT, value=0),
                                 cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))[3:-3, 3:-3]
        filled = _fill_holes(local)
        dist = cv2.distanceTransform(filled, cv2.DIST_L2, 5)
        min_core = max(4, int(0.01 * self.cube_area))
        cores = None
        n_cores = 0
        core_labels = None
        # Pragul crește până când nucleele cuburilor lipite se despart (gâtul dintre ele poate fi lat).
        for threshold in (0.5, 0.6, 0.7, 0.8):
            candidate = (dist > threshold * float(dist.max())).astype(np.uint8)
            count, labels, stats, _ = cv2.connectedComponentsWithStats(candidate)
            big = [label for label in range(1, count) if stats[label, cv2.CC_STAT_AREA] >= min_core]
            if len(big) >= 2:
                cores = np.isin(labels, big).astype(np.uint8)
                n_cores, core_labels = cv2.connectedComponents(cores)
                break
        if cores is None or core_labels is None or n_cores - 1 < 2:
            return None
        sources = np.where(cores > 0, 0, 1).astype(np.uint8)
        _, nearest = cv2.distanceTransformWithLabels(sources, cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_CCOMP)
        result = []
        pixel_labels = nearest[ys - y0, xs - x0]
        for core in range(1, n_cores):
            core_pixels = np.argwhere(core_labels == core)
            label = nearest[core_pixels[0, 0], core_pixels[0, 1]]
            selected = pixel_labels == label
            if selected.any():
                result.append((ys[selected], xs[selected]))
        return result if len(result) >= 2 else None

    def _analyze_cell(
        self,
        coord: GridCoord,
        parts: List[_Part],
        hsv: np.ndarray,
        gray: np.ndarray,
        edges: np.ndarray,
        neutral: np.ndarray,
        foreign: np.ndarray,
        owner: np.ndarray,
        index: int,
    ) -> CellResult:
        cfg = self.config
        cell = self.cell_px
        x0, y0 = self._margin + coord.col * cell, self._margin + coord.row * cell
        area = int(sum(len(p.ys) for p in parts))
        fraction = area / self.cube_area

        margin = cfg.roi_margin_px
        roi = (slice(y0 + margin, y0 + cell - margin), slice(x0 + margin, x0 + cell - margin))
        valid = ~foreign[roi]
        metrics: Dict[str, float] = {"colored_fraction": round(fraction, 4)}
        enough_roi = valid.sum() >= 0.2 * valid.size
        edge_density = float(edges[roi][valid].mean()) if enough_roi else float("nan")
        if enough_roi:
            metrics["edge_density"] = round(edge_density, 4)

        if fraction < cfg.occupied_min_fraction:
            if fraction > cfg.empty_max_fraction:
                return CellResult(CellObservation(coord, Occupancy.UNKNOWN, 0.0, reasons=(REASON_AMBIGUOUS_AREA,), metrics=metrics))
            if not enough_roi:
                return CellResult(CellObservation(coord, Occupancy.UNKNOWN, 0.0, reasons=(REASON_ROI_OCCLUDED,), metrics=metrics))
            roi_gray = gray[roi]
            table_level = float(np.median(roi_gray[valid]))
            dark = (roi_gray < table_level - cfg.dark_object_contrast) & neutral[roi] & valid
            dark_px = int(dark.sum())
            metrics["dark_px"] = float(dark_px)
            if dark_px >= cfg.dark_object_min_px:
                return CellResult(CellObservation(coord, Occupancy.UNKNOWN, 0.0, reasons=(REASON_DARK_OBJECT,), metrics=metrics))
            # Un cub decolorat (saturație peste masa albă, dar sub pragul de clasificare) poate umple ROI-ul
            # fără muchii; umbrele au saturație mică și nu declanșează verificarea.
            faint = float((hsv[roi][..., 1][valid] >= cfg.faint_saturation_min).mean())
            metrics["faint_fraction"] = round(faint, 4)
            if faint >= cfg.faint_fraction_max:
                return CellResult(CellObservation(coord, Occupancy.UNKNOWN, 0.0, reasons=(REASON_FAINT_COLOR,), metrics=metrics))
            if edge_density > cfg.edge_density_max:
                return CellResult(
                    CellObservation(coord, Occupancy.UNKNOWN, 0.0, reasons=(REASON_EDGES_WITHOUT_COLOR,), metrics=metrics)
                )
            confidence = 1.0 - 0.25 * fraction / max(cfg.empty_max_fraction, 1e-9) - 0.25 * edge_density / max(
                cfg.edge_density_max, 1e-9
            )
            return CellResult(CellObservation(coord, Occupancy.EMPTY, round(_clamp(confidence, 0.5), 4), metrics=metrics))

        return self._occupied(coord, parts, hsv, area, fraction, metrics, owner, index)

    def _occupied(
        self,
        coord: GridCoord,
        parts: List[_Part],
        hsv: np.ndarray,
        area: int,
        fraction: float,
        metrics: Dict[str, float],
        owner: np.ndarray,
        index: int,
    ) -> CellResult:
        cfg = self.config
        cell = self.cell_px
        reasons: List[str] = []
        ys = np.concatenate([p.ys for p in parts])
        xs = np.concatenate([p.xs for p in parts])
        splits = {p.split for p in parts} - {""}
        points = np.column_stack([xs, ys]).astype(np.float32)
        hull = cv2.convexHull(points)
        hull_area = max(float(cv2.contourArea(hull)), 1.0)
        rect = cv2.minAreaRect(hull)
        box = cv2.boxPoints(rect)
        side = math.sqrt(hull_area)

        # Fereastră locală în jurul blob-ului (pentru viteză).
        height, width = hsv.shape[:2]
        wy0, wx0 = max(0, int(ys.min()) - WINDOW_MARGIN), max(0, int(xs.min()) - WINDOW_MARGIN)
        wy1, wx1 = min(height, int(ys.max()) + WINDOW_MARGIN + 1), min(width, int(xs.max()) + WINDOW_MARGIN + 1)
        blob = np.zeros((wy1 - wy0, wx1 - wx0), dtype=np.uint8)
        blob[ys - wy0, xs - wx0] = 255
        # Fața cubului e convexă: înfășurătoarea umple și inelele cu goluri (pixeli de margine fără clasă).
        filled = np.zeros_like(blob)
        local_hull = np.rint(hull.reshape(-1, 2) - np.array([wx0, wy0], dtype=np.float32)).astype(np.int32)
        cv2.fillConvexPoly(filled, local_hull, 255)

        # Culoarea: doar inelul exterior al feței, ca pictograma din centru să nu influențeze votul.
        ring_px = max(2, int(round(cfg.ring_width_fraction * side)))
        inner = cv2.erode(filled, np.ones((2 * ring_px + 1, 2 * ring_px + 1), np.uint8))
        ring = (filled > 0) & (inner == 0) & (blob > 0)
        vote = self.classifier.vote(hsv[wy0:wy1, wx0:wx1][ring], cfg.min_color_pixels)
        reasons.extend(vote.reasons)

        # Yaw: orientarea dominantă a conturului; marginile tăiate artificial (lângă alte blob-uri) se exclud.
        others = ((owner[wy0:wy1, wx0:wx1] >= 0) & (owner[wy0:wy1, wx0:wx1] != index)).astype(np.uint8)
        exclude = cv2.dilate(others, np.ones((7, 7), np.uint8)) > 0 if splits else np.zeros(blob.shape, dtype=bool)
        # Un blob tăiat de marginea imaginii are o latură artificială: marginea nu participă la orientare.
        clipped = ys.min() <= 1 or xs.min() <= 1 or ys.max() >= height - 2 or xs.max() >= width - 2
        if clipped:
            border = np.zeros(blob.shape, dtype=bool)
            border[: max(0, 4 - wy0), :] = True
            border[:, : max(0, 4 - wx0)] = True
            border[max(0, height - 4 - wy0):, :] = True
            border[:, max(0, width - 4 - wx0):] = True
            exclude = exclude | border
        theta, concentration = dominant_orientation(filled, exclude)
        yaw = normalize_yaw_deg(-theta)  # axa y a imaginii e în jos: sensul antiorar pe ecran are semn opus
        size_ratio = side / self.cube_px
        yaw_conf = _clamp((concentration - 0.5) / 0.4)
        if clipped:
            yaw_conf *= 0.5
            reasons.append(REASON_CLIPPED)
        if not 0.75 <= size_ratio <= 1.5:
            yaw_conf *= 0.4
        if "cut" in splits:
            yaw_conf *= 0.3
        yaw_value: Optional[float] = round(normalize_yaw_deg(round(yaw, 2)), 2)
        if yaw_conf < cfg.yaw_min_confidence:
            yaw_value, yaw_conf = None, 0.0
            reasons.append(REASON_YAW_UNRELIABLE)

        fill_ratio = area / max(float((filled > 0).sum()), 1.0)
        if fill_ratio >= cfg.top_face_full_min_fill:
            top_face = TopFace.FULL
        elif fill_ratio <= cfg.top_face_icon_max_fill:
            top_face = TopFace.ICON
        else:
            top_face = TopFace.UNKNOWN

        occ_conf = _clamp(0.5 + 0.5 * (fraction - cfg.occupied_min_fraction) / cfg.occupied_min_fraction, 0.5)
        cx, cy = float(xs.mean()), float(ys.mean())
        offset = math.hypot(cx - self._margin - (coord.col + 0.5) * cell, cy - self._margin - (coord.row + 0.5) * cell)
        if offset > cfg.off_center_px:
            occ_conf *= 0.7
            reasons.append(REASON_OFF_CENTER)
        if "separated" in splits:
            occ_conf *= 0.9
            reasons.append(REASON_SEPARATED)
        if "cut" in splits:
            occ_conf *= 0.7
            reasons.append(REASON_SPLIT)
        if fraction > OVERSIZED_AREA_FACTOR:
            occ_conf *= 0.6
            reasons.append(REASON_OVERSIZED)

        metrics.update(
            {
                "offset_px": round(offset, 2),
                "orientation_concentration": round(concentration, 3),
                "face_fill": round(fill_ratio, 3),
                "size_ratio": round(size_ratio, 3),
                "color_dominance": round(vote.dominance, 3),
                "color_pixels": float(vote.pixel_count),
            }
        )
        uv_top = (round((cx - self._margin) / cell, 4), round((cy - self._margin) / cell, 4))
        observation = CellObservation(
            coord=coord,
            occupancy=Occupancy.OCCUPIED,
            occupancy_confidence=round(occ_conf, 4),
            color=vote.color,
            color_confidence=round(vote.confidence, 4) if vote.color is not Color.UNKNOWN else 0.0,
            yaw_deg=yaw_value,
            yaw_confidence=round(yaw_conf, 4),
            top_face=top_face,
            reasons=tuple(reasons),
            metrics=metrics,
            uv_top=uv_top,
        )
        return CellResult(observation, box=box, centroid=(cx, cy))
