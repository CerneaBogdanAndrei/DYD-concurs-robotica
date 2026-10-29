"""Selftest-ul Etapei 0: scene sintetice reproductibile → pipeline → comparație cu adevărul de referință.

Pragurile se aplică doar pe date sintetice și nu spun nimic despre performanța pe camera reală.
"""

from __future__ import annotations

import math
import time
from collections import Counter
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from brain.configuration.loader import AppConfig
from brain.domain.cube import Color, Occupancy, yaw_difference_deg
from brain.domain.grid import GridCoord
from brain.perception.annotate import render_annotation
from brain.perception.image_io import write_image
from brain.perception.pipeline import VisionPipeline, VisionResult
from brain.simulation.scene_gen import AMBIGUOUS, KIND_POSITIVE, NEGATIVE_KINDS, SceneTruth, generate_scene

HIGH_CONFIDENCE = 0.8


@dataclass(frozen=True)
class SelftestThresholds:
    grid_detection_min: float = 0.99
    occupancy_accuracy_min: float = 0.99
    color_accuracy_min: float = 0.98
    top_face_accuracy_min: float = 0.95
    yaw_within_5deg_min: float = 0.95
    high_confidence_error_max: float = 0.01
    corner_tolerance_px: float = 4.0


@dataclass
class _Counts:
    positive_scenes: int = 0
    grid_ok: int = 0
    grid_wrong: int = 0
    grid_missed: int = 0
    cells: int = 0
    occupancy_correct: int = 0
    occupancy_unknown: int = 0
    color_total: int = 0
    color_correct: int = 0
    top_face_correct: int = 0
    yaw_total: int = 0
    yaw_within_5: int = 0
    yaw_errors: List[float] = field(default_factory=list)
    high_conf_predictions: int = 0
    high_conf_errors: int = 0
    rejection_scenes: int = 0
    rejection_correct: int = 0
    occluded_scenes: int = 0
    occluded_ok: int = 0
    ambiguous_cells: int = 0
    ambiguous_ok: int = 0
    processing_ms: List[float] = field(default_factory=list)


def _schedule(seed: int, scenes: int) -> List[Dict[str, Any]]:
    """Amestec determinist de tipuri de scene (≈70% pozitive)."""
    plan: List[Dict[str, Any]] = []
    weights = [
        ({"kind": KIND_POSITIVE, "aruco": False}, 60),
        ({"kind": KIND_POSITIVE, "aruco": True}, 10),
        ({"kind": "occluded_corner", "aruco": False}, 6),
        ({"kind": "ambiguous_color", "aruco": False}, 8),
        ({"kind": "missing_marker", "aruco": True}, 4),
    ] + [({"kind": kind, "aruco": False}, 2) for kind in NEGATIVE_KINDS]
    total = sum(w for _, w in weights)
    for spec, weight in weights:
        plan.extend([dict(spec)] * int(round(scenes * weight / total)))
    while len(plan) < scenes:
        plan.append({"kind": KIND_POSITIVE, "aruco": False})
    plan = plan[:scenes]
    order = np.random.default_rng(seed).permutation(len(plan))
    return [dict(plan[i], seed=seed * 100003 + index) for index, i in enumerate(order)]


def _corner_error(result: VisionResult, truth: SceneTruth) -> float:
    if not result.observation.geometry.corners_px or truth.grid_corners_px is None:
        return float("inf")
    detected = np.array(result.observation.geometry.corners_px)
    expected = np.array(truth.grid_corners_px)
    return float(np.max(np.linalg.norm(detected - expected, axis=1)))


def _evaluate_cells(result: VisionResult, truth: SceneTruth, counts: _Counts, failures: List[str]) -> None:
    for cell in result.observation.cells:
        gt = truth.cube_at(cell.coord.row, cell.coord.col)
        counts.cells += 1
        predicted_occupied = cell.occupancy is Occupancy.OCCUPIED
        if cell.occupancy is Occupancy.UNKNOWN:
            counts.occupancy_unknown += 1
        correct_occupancy = (gt is not None and predicted_occupied) or (gt is None and cell.occupancy is Occupancy.EMPTY)
        counts.occupancy_correct += int(correct_occupancy)
        if cell.occupancy is not Occupancy.UNKNOWN and cell.occupancy_confidence > HIGH_CONFIDENCE:
            counts.high_conf_predictions += 1
            if not correct_occupancy:
                counts.high_conf_errors += 1
                failures.append("seed {} {}: ocupare greșită cu încredere {:.2f}".format(truth.seed, cell.coord, cell.occupancy_confidence))
        if gt is None or not predicted_occupied:
            continue
        counts.color_total += 1
        color_ok = cell.color.value == gt.color
        counts.color_correct += int(color_ok)
        counts.top_face_correct += int(cell.top_face.value == gt.top_face)
        if cell.color is not Color.UNKNOWN and cell.color_confidence > HIGH_CONFIDENCE:
            counts.high_conf_predictions += 1
            if not color_ok:
                counts.high_conf_errors += 1
                failures.append("seed {} {}: culoare {} în loc de {} cu încredere {:.2f}".format(
                    truth.seed, cell.coord, cell.color.value, gt.color, cell.color_confidence))
        if abs(gt.yaw_deg) <= 40.0:
            counts.yaw_total += 1
            if cell.yaw_deg is not None:
                error = yaw_difference_deg(cell.yaw_deg, gt.yaw_deg)
                counts.yaw_errors.append(error)
                counts.yaw_within_5 += int(error <= 5.0)


def _rate(numerator: int, denominator: int) -> Optional[float]:
    return None if denominator == 0 else numerator / denominator


def run_stage0_selftest(
    config: AppConfig,
    seed: int = 42,
    scenes: int = 200,
    difficulty: str = "nominal",
    frames: int = 5,
    out_dir: Optional[Path] = None,
    save_failures: int = 10,
    thresholds: Optional[SelftestThresholds] = None,
    location: str = "medium",
) -> Dict[str, Any]:
    thresholds = thresholds or SelftestThresholds()
    system, colors = config.system, config.colors
    aruco_config = system.vision.aruco
    pipelines = {"lines": VisionPipeline(system, colors, method="lines")}
    if aruco_config is not None:
        pipelines["aruco"] = VisionPipeline(system, colors, method="aruco")
    loc = system.location(location)
    counts = _Counts()
    failures: List[str] = []
    saved = 0
    kinds = Counter()
    started = time.perf_counter()
    for spec in _schedule(seed, scenes):
        use_aruco = bool(spec["aruco"]) and aruco_config is not None
        if spec["aruco"] and not use_aruco:
            continue
        kinds[spec["kind"] + (" (aruco)" if use_aruco else "")] += 1
        scene = generate_scene(
            spec["seed"], system.grid_geometry(location), kind=spec["kind"], difficulty=difficulty, frames=frames,
            rows=loc.rows, cols=loc.cols, aruco=aruco_config if use_aruco else None,
        )
        result = pipelines["aruco" if use_aruco else "lines"].analyze(scene.frames, location)
        obs = result.observation
        truth = scene.truth
        counts.processing_ms.append(obs.processing_ms)
        failed_before = len(failures)
        if truth.kind == KIND_POSITIVE:
            counts.positive_scenes += 1
            if not obs.actionable:
                counts.grid_missed += 1
                failures.append("seed {} ({}): grilă nedetectată: {}".format(truth.seed, truth.kind, ",".join(obs.reasons)))
            elif _corner_error(result, truth) > thresholds.corner_tolerance_px:
                counts.grid_wrong += 1
                failures.append("seed {}: geometrie GREȘITĂ acceptată (eroare colț {:.1f} px)".format(
                    truth.seed, _corner_error(result, truth)))
            else:
                counts.grid_ok += 1
                _evaluate_cells(result, truth, counts, failures)
        elif truth.expect_actionable is False:
            counts.rejection_scenes += 1
            if obs.actionable:
                failures.append("seed {} ({}): scenă negativă acceptată".format(truth.seed, truth.kind))
            else:
                counts.rejection_correct += 1
        elif truth.kind == "occluded_corner":
            counts.occluded_scenes += 1
            if obs.actionable and _corner_error(result, truth) > thresholds.corner_tolerance_px:
                failures.append("seed {} (occluded_corner): geometrie greșită acceptată ({:.1f} px)".format(
                    truth.seed, _corner_error(result, truth)))
            else:
                counts.occluded_ok += 1
        elif truth.kind == "ambiguous_color" and obs.actionable:
            for cube in truth.cubes:
                if cube.color != AMBIGUOUS:
                    continue
                counts.ambiguous_cells += 1
                cell = obs.cell(GridCoord(cube.row, cube.col))
                confident_wrong_color = cell.color is not Color.UNKNOWN and cell.color_confidence > HIGH_CONFIDENCE
                confident_empty = cell.occupancy is Occupancy.EMPTY and cell.occupancy_confidence > HIGH_CONFIDENCE
                if confident_wrong_color or confident_empty:
                    failures.append("seed {} {}: cub ambiguu ({}) clasificat sigur ca {} / {}".format(
                        truth.seed, (cube.row, cube.col), cube.note, cell.occupancy.value, cell.color.value))
                else:
                    counts.ambiguous_ok += 1
        if out_dir is not None and len(failures) > failed_before and saved < save_failures:
            write_image(Path(out_dir) / "failure_{}.jpg".format(truth.seed), render_annotation(result))
            saved += 1

    yaw_errors = counts.yaw_errors
    metrics = {
        "grid_detection_rate": _rate(counts.grid_ok, counts.positive_scenes),
        "grid_wrong_accepted": counts.grid_wrong,
        "occupancy_accuracy": _rate(counts.occupancy_correct, counts.cells),
        "occupancy_unknown_rate": _rate(counts.occupancy_unknown, counts.cells),
        "color_accuracy": _rate(counts.color_correct, counts.color_total),
        "top_face_accuracy": _rate(counts.top_face_correct, counts.color_total),
        "yaw_within_5deg": _rate(counts.yaw_within_5, counts.yaw_total),
        "yaw_error_median_deg": float(np.median(yaw_errors)) if yaw_errors else None,
        "yaw_error_p95_deg": float(np.percentile(yaw_errors, 95)) if yaw_errors else None,
        "high_confidence_error_rate": _rate(counts.high_conf_errors, counts.high_conf_predictions),
        "negative_rejection_rate": _rate(counts.rejection_correct, counts.rejection_scenes),
        "occluded_corner_safe_rate": _rate(counts.occluded_ok, counts.occluded_scenes),
        "ambiguous_safe_rate": _rate(counts.ambiguous_ok, counts.ambiguous_cells),
        "processing_ms_median": float(np.median(counts.processing_ms)) if counts.processing_ms else None,
        "processing_ms_max": float(np.max(counts.processing_ms)) if counts.processing_ms else None,
    }

    def at_least(value: Optional[float], minimum: float) -> bool:
        return value is not None and value >= minimum

    def at_most(value: Optional[float], maximum: float) -> bool:
        return value is not None and value <= maximum

    checks = {
        "grid_detection": at_least(metrics["grid_detection_rate"], thresholds.grid_detection_min),
        "no_wrong_geometry": counts.grid_wrong == 0,
        "occupancy": at_least(metrics["occupancy_accuracy"], thresholds.occupancy_accuracy_min),
        "color": at_least(metrics["color_accuracy"], thresholds.color_accuracy_min),
        "top_face": at_least(metrics["top_face_accuracy"], thresholds.top_face_accuracy_min),
        "yaw": at_least(metrics["yaw_within_5deg"], thresholds.yaw_within_5deg_min),
        "high_confidence_errors": at_most(metrics["high_confidence_error_rate"], thresholds.high_confidence_error_max),
        "negatives_rejected": metrics["negative_rejection_rate"] in (None, 1.0),
        "occluded_corner_safe": metrics["occluded_corner_safe_rate"] in (None, 1.0),
        "ambiguous_safe": metrics["ambiguous_safe_rate"] in (None, 1.0),
    }
    return {
        "seed": seed,
        "scenes": scenes,
        "location": location,
        "difficulty": difficulty,
        "frames_per_scene": frames,
        "kinds": dict(sorted(kinds.items())),
        "thresholds": asdict(thresholds),
        "metrics": {k: (round(v, 5) if isinstance(v, float) and math.isfinite(v) else v) for k, v in metrics.items()},
        "checks": checks,
        "passed": all(checks.values()),
        "failures": failures,
        "duration_s": round(time.perf_counter() - started, 2),
        "note": "Rezultate pe scene sintetice; nu validează camera reală.",
    }
