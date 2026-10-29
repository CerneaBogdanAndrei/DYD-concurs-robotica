"""Pipeline-ul de viziune: cadre → calitate → agregare → grilă → redresare → celule → `GridObservation`.

Dacă orice etapă nu poate fi estimată sigur, observația e marcată neacționabilă, fără celule,
iar motivele sunt raportate. Nicio comandă de manipulare nu trebuie generată dintr-o astfel de observație.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from brain.configuration.models import ColorsConfig, LocationConfig, SystemConfig
from brain.domain.cube import Occupancy
from brain.domain.errors import PerceptionInputError
from brain.domain.grasp import GAP_METRIC_KEYS, clearances, footprint_from_uv
from brain.domain.observation import FrameQualitySummary, GeometrySummary, GridObservation
from brain.domain.scene_state import utc_now_iso
from brain.perception import geometry as geo
from brain.perception.cells import CellAnalyzer, CellResult
from brain.perception.color import HueClassifier
from brain.perception.grid_aruco import detect_grid_markers
from brain.perception.grid_lines import detect_grid_lines
from brain.perception.lens import Undistorter
from brain.perception.photometry import (
    FrameQuality,
    aggregate_frames,
    apply_gains,
    assess_frame,
    estimate_white_balance,
    validate_frames,
    white_reference_mask,
)

REASON_TOO_FEW_FRAMES = "too_few_valid_frames"
MIN_LINES_MEASURED = 2
RECTIFIED_MARGIN_CELLS = 0.3  # minimul; cu camera oblică, cât cere paralaxa (`rectified_margin_cells`)
MARGIN_YAW_DEG = 15.0  # fața de sus a unui cub rotit până la atât încape în margine
RESIDUAL_TIE_FACTOR = 1.1  # potriviri ale aceleiași grile cu reziduuri la sub 10% una de alta sunt echivalente


def rectified_margin_cells(loc: LocationConfig, cube_mm: float) -> float:
    """Marginea redresării, în celule: cât iese peste contur fața de sus a unui cub de pe o celulă de margine.

    Fața de sus e mărită cu H/(H-h) și împinsă departe de nadir; cu nadirul în afara foii (camera de lângă
    bază, înclinată), fața cubului de pe rândul cel mai depărtat trece cu mult peste linia de contur.
    """
    scale = loc.camera_height_mm / (loc.camera_height_mm - cube_mm)
    yaw = np.radians(MARGIN_YAW_DEG)
    half = cube_mm / loc.cell_mm / 2.0 * float(np.cos(yaw) + np.sin(yaw))
    nu, nv = loc.nadir_uv
    over = 0.0
    for r in range(loc.rows):
        for c in range(loc.cols):
            for du in (-half, half):
                for dv in (-half, half):
                    u = nu + (c + 0.5 + du - nu) * scale
                    v = nv + (r + 0.5 + dv - nv) * scale
                    over = max(over, -u, u - loc.cols, -v, v - loc.rows)
    return max(RECTIFIED_MARGIN_CELLS, over + 0.05)


@dataclass
class _Geometry:
    ok: bool
    reasons: List[str]
    homography: Optional[np.ndarray] = None
    grid_corners: Optional[np.ndarray] = None
    structure: Optional[float] = None
    metrics: dict = field(default_factory=dict)


@dataclass
class VisionResult:
    """Observația + artefactele intermediare necesare pentru adnotare și depanare."""

    observation: GridObservation
    aggregated: Optional[np.ndarray] = None
    rectified: Optional[np.ndarray] = None  # orientare logică, după balansul de alb, cu margine
    rectified_margin: int = 0  # px în jurul grilei în `rectified`
    homography: Optional[np.ndarray] = None  # imagine agregată → redresată (cadrul camerei, fără margine)
    quarter_turns: int = 0
    cell_results: List[CellResult] = field(default_factory=list)
    frame_qualities: List[FrameQuality] = field(default_factory=list)
    white_balance_gains: Optional[Tuple[float, float, float]] = None
    px_per_mm: Optional[float] = None  # scara imaginii redresate (pe masă)
    cube_mm: Optional[float] = None


def measure_cubes(
    results: List[CellResult], analyzer: CellAnalyzer, loc: LocationConfig, cube_mm: float, cell_px: int
) -> List[CellResult]:
    """Măsurători în mm pentru controlul ghearei, adăugate în `metrics` fiecărui cub văzut:
    - `side_w_mm`, `side_h_mm`: laturile feței de sus (dreptunghiul minim), aduse la scara mesei (fără paralaxă);
    - `gap_a_neg_mm` ... `gap_b_pos_mm`: spațiul liber până la cel mai apropiat vecin, pe axele cubului
      (lipsește dacă pe direcția aceea nu e nimic). Bazele vecinilor: cuburi unice pe masă, latura nominală.
    """
    scale = cell_px / loc.cell_mm * analyzer.top_scale  # px redresați per mm, pe fața de sus a unui cub
    prints = {}
    for result in results:
        obs = result.observation
        if obs.occupancy is Occupancy.OCCUPIED and obs.uv_top is not None:
            prints[obs.coord] = footprint_from_uv(analyzer.base_uv(obs.uv_top), obs.yaw_deg, loc.cell_mm, cube_mm)
    measured = []
    for result in results:
        obs = result.observation
        footprint = prints.get(obs.coord)
        if footprint is None:
            measured.append(result)
            continue
        metrics = dict(obs.metrics)
        if result.box is not None:
            edges = np.linalg.norm(np.roll(result.box, -1, axis=0) - result.box, axis=1)
            metrics["side_w_mm"] = round(float(edges[0]) / scale, 1)
            metrics["side_h_mm"] = round(float(edges[1]) / scale, 1)
        gaps = clearances(footprint, [p for coord, p in prints.items() if coord != obs.coord]).to_dict()
        for side, key in GAP_METRIC_KEYS:
            if gaps[side] is not None and gaps[side] < loc.cell_mm:  # mai departe de o celulă nu ajunge niciun deget
                metrics[key] = float(gaps[side])
        measured.append(CellResult(replace(obs, metrics=metrics), box=result.box, centroid=result.centroid))
    return measured


class VisionPipeline:
    def __init__(self, system: SystemConfig, colors: ColorsConfig, method: Optional[str] = None) -> None:
        self.system = system
        self.colors = colors
        self.method = method or system.vision.detection_method
        if self.method not in ("lines", "aruco"):
            raise PerceptionInputError("metodă de detecție necunoscută: {}".format(self.method))
        if self.method == "aruco" and system.vision.aruco is None:
            raise PerceptionInputError("metoda aruco cere secțiunea vision.aruco în {}".format(system.source_file))
        self.classifier = HueClassifier(colors)
        self._analyzers: Dict[str, CellAnalyzer] = {}
        # Obiectivul cu unghi larg curbează liniile: cadrele se îndreaptă înainte de orice (dacă există lens.yaml).
        self._undistort = Undistorter(system.lens) if system.lens is not None else None

    def _analyzer(self, location: str) -> CellAnalyzer:
        """Câte un analizor per șablon: raportul cub/celulă diferă între șabloanele mic, mediu și mare.

        Poziția camerei (punctul de sub ea și înălțimea) dă geometria vederii oblice: fața de sus a unui cub e
        mărită cu H/(H-h) și împinsă departe de nadir; analizorul atribuie cubul după bază.
        """
        if location not in self._analyzers:
            loc = self.system.location(location)
            height = loc.camera_height_mm
            self._analyzers[location] = CellAnalyzer(
                self.system.vision.cells, self.classifier, self.system.vision.rectified_cell_px,
                self.system.cube_px(location), nadir_uv=loc.nadir_uv,
                top_scale=height / (height - self.system.cube.size_mm),
            )
        return self._analyzers[location]

    def analyze(self, frames: Sequence[np.ndarray], location: str, captured_at: Optional[str] = None) -> VisionResult:
        start = time.perf_counter()
        try:
            loc = self.system.location(location)
        except KeyError as exc:
            raise PerceptionInputError(str(exc.args[0])) from None
        frames = validate_frames(frames)
        if self._undistort is not None:
            frames = self._undistort.undistort_all(frames)
        vision = self.system.vision
        captured = captured_at or utc_now_iso()

        qualities = [assess_frame(frame, vision.quality) for frame in frames]
        valid = [frame for frame, quality in zip(frames, qualities) if quality.ok]
        if len(valid) < vision.min_valid_frames:
            flags = sorted({flag for q in qualities for flag in q.flags})
            aggregated = aggregate_frames(frames)
            return self._reject(start, loc, captured, [REASON_TOO_FEW_FRAMES] + flags, qualities, len(valid), aggregated)

        aggregated = aggregate_frames(valid)
        aggregate_quality = assess_frame(aggregated, vision.quality)
        if not aggregate_quality.ok:
            return self._reject(start, loc, captured, list(aggregate_quality.flags), qualities, len(valid), aggregated)

        located = self._locate_grid(aggregated, loc)
        if not located.ok or located.homography is None:
            return self._reject(start, loc, captured, located.reasons, qualities, len(valid), aggregated, located)

        cell_px = vision.rectified_cell_px
        # Marginea în jurul grilei: cuburile din celulele de pe contur pot depăși linia (paralaxă, rotație).
        margin = int(round(rectified_margin_cells(loc, self.system.cube.size_mm) * cell_px))
        shift = np.array([[1.0, 0.0, margin], [0.0, 1.0, margin], [0.0, 0.0, 1.0]])
        size = (loc.cols * cell_px + 2 * margin, loc.rows * cell_px + 2 * margin)
        rectified = geo.warp(aggregated, shift @ located.homography, size)
        # Ce acoperă cadrul camerei: o față tăiată de marginea cadrului nu are voie să pară întreagă.
        inside = cv2.warpPerspective(np.full(aggregated.shape[:2], 255, np.uint8), shift @ located.homography, size,
                                     flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        gains = None
        if vision.white_balance.enabled:
            reference = white_reference_mask(rectified)
            gains = estimate_white_balance(
                rectified, reference, vision.white_balance.max_gain, vision.white_balance.min_reference_pixels
            )
            if gains is not None:
                rectified = apply_gains(rectified, gains)
        turns = loc.orientation_quarter_turns
        logical = np.ascontiguousarray(np.rot90(rectified, k=turns)) if turns else rectified
        if turns:
            inside = np.ascontiguousarray(np.rot90(inside, k=turns))
        analyzer = self._analyzer(loc.name)
        cell_results = measure_cubes(analyzer.analyze(logical, loc.rows, loc.cols, margin, inside), analyzer, loc,
                                     self.system.cube.size_mm, cell_px)

        observation = GridObservation(
            location=loc.name,
            rows=loc.rows,
            cols=loc.cols,
            captured_at=captured,
            actionable=True,
            cells=tuple(result.observation for result in cell_results),
            reasons=(),
            geometry=self._geometry_summary(located),
            quality=self._quality_summary(qualities, len(valid), aggregate_quality),
            processing_ms=round((time.perf_counter() - start) * 1000.0, 2),
        )
        return VisionResult(
            observation=observation,
            aggregated=aggregated,
            rectified=logical,
            rectified_margin=margin,
            homography=located.homography,
            quarter_turns=turns,
            cell_results=cell_results,
            frame_qualities=qualities,
            white_balance_gains=gains,
            px_per_mm=cell_px / loc.cell_mm,
            cube_mm=self.system.cube.size_mm,
        )

    def _locate_grid(self, image: np.ndarray, loc: LocationConfig) -> _Geometry:
        vision = self.system.vision
        cell_px = vision.rectified_cell_px
        if self.method == "lines":
            detections = detect_grid_lines(image, vision.lines, loc.rows, loc.cols, roi=self._roi(loc))
            evaluated = []
            for detection in detections:
                if not detection.ok or detection.image_points is None or detection.grid_points is None:
                    evaluated.append(_Geometry(False, list(detection.reasons)))
                    continue
                evaluated.append(
                    self._evaluate(image, loc, detection.image_points, detection.grid_points * cell_px, dict(detection.metrics))
                )
            return self._select(evaluated, image.shape, loc)
        aruco = vision.aruco
        assert aruco is not None
        markers = detect_grid_markers(image, aruco)
        if not markers.ok or markers.centers is None:
            return _Geometry(False, list(markers.reasons))
        scale = self.system.px_per_mm(loc.name)
        dst = np.array([[x * scale, y * scale] for x, y in aruco.marker_centers_mm], dtype=np.float64)
        return self._evaluate(image, loc, markers.centers, dst, {})

    def _select(self, evaluated: List[_Geometry], shape: Sequence[int], loc: LocationConfig) -> _Geometry:
        """Dintre candidații valizi: pentru fiecare grilă distinctă, potrivirea cea mai bună; apoi grila cea
        mai apropiată de colțurile așteptate sau, fără ele, de centrul imaginii."""
        valid = [g for g in evaluated if g.ok and g.grid_corners is not None]
        if not valid:
            with_geometry = [g for g in evaluated if g.grid_corners is not None]
            return (with_geometry or evaluated)[0]
        # Candidații aceleiași grile (colțuri la sub un sfert de celulă) diferă doar prin precizie; din fiecare
        # grup rămâne potrivirea cea mai bună. Centralitatea alege doar între șabloane diferite.
        groups: List[List[_Geometry]] = []
        for g in valid:
            sides = np.linalg.norm(np.roll(g.grid_corners, -1, axis=0) - g.grid_corners, axis=1)
            same_grid_px = 0.25 * float(sides.mean()) / max(loc.rows, loc.cols)
            for group in groups:
                if float(np.max(np.linalg.norm(g.grid_corners - group[0].grid_corners, axis=1))) <= same_grid_px:
                    group.append(g)
                    break
            else:
                groups.append([g])
        distinct = [self._best_fit(group) for group in groups]
        if loc.expected_corners_px is not None:
            expected = np.array(loc.expected_corners_px, dtype=np.float64)
            return min(distinct, key=lambda g: float(np.max(np.linalg.norm(g.grid_corners - expected, axis=1))))
        center = np.array([shape[1] / 2.0, shape[0] / 2.0])
        return min(distinct, key=lambda g: float(np.linalg.norm(g.grid_corners.mean(axis=0) - center)))

    @staticmethod
    def _best_fit(group: List[_Geometry]) -> _Geometry:
        """Reziduurile aflate la cel mult 10% de minim sunt echivalente (diferența e zgomot). Între ele decide
        scorul de structură, apoi ordinea de generare (pornirea din înfășurătoare, înaintea dreptunghiului minim)."""

        def residual(g: _Geometry) -> float:
            return float(g.metrics.get("line_residual_cells", float("inf")))

        best = min(residual(g) for g in group)
        near = [g for g in group if residual(g) <= RESIDUAL_TIE_FACTOR * best]
        return max(near, key=lambda g: g.structure or 0.0)  # la egalitate, max păstrează primul (ordinea de generare)

    def _evaluate(
        self, image: np.ndarray, loc: LocationConfig, src: np.ndarray, dst: np.ndarray, metrics: Dict[str, float]
    ) -> _Geometry:
        vision = self.system.vision
        thresholds = vision.geometry
        cell_px = vision.rectified_cell_px
        width, height = loc.cols * cell_px, loc.rows * cell_px
        homography = geo.fit_homography(src, dst)
        if homography is None:
            return _Geometry(False, [geo.REASON_NOT_CONVEX], metrics=metrics)
        # Colțurile grilei (centrele liniilor de contur) în imagine.
        rect_corners = np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype=np.float64)
        grid_corners = geo.transform_points(np.linalg.inv(homography), rect_corners)
        reasons = geo.validate_quad(grid_corners, image.shape, thresholds)
        if geo.REASON_NOT_CONVEX in reasons:
            return _Geometry(False, reasons, metrics=metrics)
        condition = geo.normalized_condition_number(homography, grid_corners, rect_corners)
        metrics["condition_number"] = round(condition, 3)
        if not np.isfinite(condition) or condition > thresholds.max_condition_number:
            reasons.append(geo.REASON_ILL_CONDITIONED)
            return _Geometry(False, reasons, grid_corners=grid_corners, metrics=metrics)

        report = geo.structure_score(
            image,
            homography,
            loc.rows,
            loc.cols,
            cell_px,
            vision.lines.adaptive_c,
            vision.lines.dark_chroma_max,
            self.colors.saturation_min,
            self.colors.value_min,
        )
        metrics["structure_score"] = round(report.score, 4)
        metrics["min_line_support"] = round(report.min_line_support, 4)
        metrics["lines_measured"] = float(len(report.line_supports))
        enforce = self.method == "lines" or (vision.aruco is not None and vision.aruco.require_line_structure)
        if enforce:
            # O linie nemăsurabilă (acoperită de cuburi) nu e o dovadă împotriva geometriei; o linie
            # măsurată fără tuș este. Fără nicio măsurătoare, geometria nu poate fi verificată.
            if len(report.line_supports) < MIN_LINES_MEASURED:
                reasons.append(geo.REASON_UNVERIFIABLE)
            elif report.min_line_support < thresholds.min_line_support:
                reasons.append(geo.REASON_LINE_UNSUPPORTED)
            if report.score < thresholds.min_structure_score:
                reasons.append(geo.REASON_STRUCTURE)
        return _Geometry(not reasons, reasons, homography, grid_corners, report.score, metrics)

    @staticmethod
    def _roi(loc: LocationConfig) -> Optional[np.ndarray]:
        """Regiunea de căutare din colțurile așteptate (calibrare), dilatată cu 25% în jurul centrului."""
        if loc.expected_corners_px is None:
            return None
        corners = np.array(loc.expected_corners_px, dtype=np.float64)
        center = corners.mean(axis=0)
        return (corners - center) * 1.25 + center

    def _geometry_summary(self, located: _Geometry) -> GeometrySummary:
        corners: Tuple[Tuple[float, float], ...] = ()
        if located.grid_corners is not None:
            corners = tuple((round(float(x), 2), round(float(y), 2)) for x, y in located.grid_corners)
        return GeometrySummary(
            method=self.method,
            ok=located.ok,
            corners_px=corners,
            structure_score=None if located.structure is None else round(located.structure, 4),
            metrics=dict(located.metrics),
        )

    @staticmethod
    def _quality_summary(
        qualities: List[FrameQuality], valid_count: int, aggregate: Optional[FrameQuality] = None
    ) -> FrameQualitySummary:
        reference = aggregate if aggregate is not None else qualities[0]
        flags = sorted({flag for q in qualities for flag in q.flags} | set(reference.flags))
        return FrameQualitySummary(
            frames_total=len(qualities),
            frames_valid=valid_count,
            flags=tuple(flags),
            mean_brightness=round(reference.mean, 2),
            contrast=round(reference.contrast, 2),
            sharpness=round(reference.sharpness, 2),
        )

    def _reject(
        self,
        start: float,
        loc: LocationConfig,
        captured: str,
        reasons: List[str],
        qualities: List[FrameQuality],
        valid_count: int,
        aggregated: Optional[np.ndarray],
        located: Optional[_Geometry] = None,
    ) -> VisionResult:
        geometry = (
            self._geometry_summary(located) if located is not None else GeometrySummary(method=self.method, ok=False)
        )
        observation = GridObservation(
            location=loc.name,
            rows=loc.rows,
            cols=loc.cols,
            captured_at=captured,
            actionable=False,
            cells=(),
            reasons=tuple(dict.fromkeys(reasons)) or ("unknown_failure",),
            geometry=geometry,
            quality=self._quality_summary(qualities, valid_count),
            processing_ms=round((time.perf_counter() - start) * 1000.0, 2),
        )
        return VisionResult(
            observation=observation,
            aggregated=aggregated,
            homography=located.homography if located is not None else None,
            frame_qualities=qualities,
        )
