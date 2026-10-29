"""`python -m brain camera preview`: captură continuă + analiză periodică, publicate în `PreviewHub`.

Captura și analiza sunt separate: pe Pi analiza poate dura aproape o secundă, iar fluxul live (30 fps) nu
trebuie să se blocheze. Analiza rulează implicit într-un PROCES separat: în același proces Python ar concura
cu firul de captură (GIL) și ar scădea cadrele pe secundă; dacă procesul nu poate porni, rămâne un fir.
Analiza folosește ultimele `frames_to_aggregate` cadre, ca `vision analyze`.

Peste fluxul live se desenează conturul cuburilor din ultima analiză (laturile în mm și cel mai mic spațiu
până la un vecin). Execuția din pagină folosește aceeași sesiune (`SessionPerception`): camera e deja deschisă,
iar cadrele observate sunt doar cele capturate după cererea de observare (adică după `settle_s`).
"""

from __future__ import annotations

import collections
import concurrent.futures
import json
import os
import threading
import time
from pathlib import Path
from typing import Any, Deque, Dict, List, NamedTuple, Optional, Tuple

import cv2
import numpy as np

from brain.adapters.preview_server import PreviewHub
from brain.configuration.models import PreviewConfig, SystemConfig
from brain.domain.cube import KNOWN_COLORS, Occupancy
from brain.domain.errors import PerceptionInputError
from brain.domain.grasp import GAP_METRIC_KEYS, Footprint, clearances_from_metrics, plan_grasp
from brain.domain.observation import CellObservation, GridObservation
from brain.domain.scene_state import utc_now_iso
from brain.memory.codec import observation_to_dict
from brain.perception.annotate import draw_overlay, overlay_items, render_annotation
from brain.perception.image_io import write_image
from brain.perception.pipeline import VisionPipeline
from brain.ports.sensors import FrameSource

# Motivele de respingere ale viziunii, în clar (pagina nu arată coduri).
REASON_TEXT = {
    "too_few_valid_frames": "prea puține cadre bune (imagine prea întunecată, prea luminoasă sau fără contrast)",
    "overexposed": "imagine supraexpusă (prea multă lumină)",
    "underexposed": "imagine subexpusă (prea întunecată)",
    "low_contrast": "contrast prea mic",
    "grid_not_found": "caroiajul nu e găsit în imagine",
    "grid_lines_not_found": "liniile caroiajului nu sunt găsite",
    "quad_not_convex": "conturul găsit nu e un patrulater valid",
    "grid_too_small": "caroiajul e prea mic în imagine (apropiați camera)",
    "grid_touches_border": "caroiajul atinge marginea imaginii (nu e complet în cadru)",
    "perspective_too_strong": "perspectivă prea puternică (fotografiați mai de sus)",
    "aspect_out_of_range": "proporțiile caroiajului nu se potrivesc",
    "angle_out_of_range": "unghiurile caroiajului sunt prea deformate",
    "homography_ill_conditioned": "transformarea imaginii e instabilă",
    "grid_structure_weak": "liniile interioare ale caroiajului nu se confirmă",
    "grid_line_unsupported": "o linie a caroiajului nu se vede în imagine",
    "grid_structure_unverifiable": "liniile caroiajului sunt acoperite: geometria nu poate fi verificată",
    "markers_missing": "lipsesc markeri ArUco",
    "markers_duplicate": "markeri ArUco duplicați",
    "unknown_failure": "eșec necunoscut",
    "edges_without_color": "contururi fără culoare (obiect necunoscut)",
    "dark_object_without_color": "obiect întunecat, fără culoare",
    "faint_color_object": "obiect cu culoare prea slabă",
    "ambiguous_colored_area": "zonă colorată ambiguă",
    "roi_occluded": "celulă acoperită",
    "off_center": "cub departe de centrul celulei",
    "merged_blob_separated": "cuburi lipite, separate",
    "split_blob": "pată ruptă în bucăți",
    "oversized_blob": "pată prea mare (mai multe cuburi sau o stivă?)",
    "yaw_unreliable": "rotația nu se poate măsura sigur",
    "blob_clipped_by_image": "cub tăiat de marginea imaginii",
    "insufficient_color_pixels": "prea puțini pixeli colorați",
    "ambiguous_hue": "nuanță între două culori",
    "low_saturation": "culoare prea ștearsă",
}


def describe_reason(code: str) -> str:
    """Codul unui motiv (eventual cu detalii după „:”) → text în română."""
    base, _, detail = str(code).partition(":")
    text = REASON_TEXT.get(base, base.replace("_", " "))
    return "{} ({})".format(text, detail) if detail else text


def encode_jpeg(image: np.ndarray, quality: int) -> bytes:
    ok, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise PerceptionInputError("cadrul nu poate fi codat JPEG")
    return buffer.tobytes()


def _cell_summary(cell: CellObservation, system: Optional[SystemConfig]) -> Dict[str, Any]:
    entry: Dict[str, Any] = {
        "state": cell.occupancy.value,
        "occupancy_confidence": round(cell.occupancy_confidence, 2),
        "reasons": list(cell.reasons),
        "reasons_text": [describe_reason(r) for r in cell.reasons],
    }
    if cell.occupancy is not Occupancy.OCCUPIED:
        entry["text"] = "{} ({:.2f})".format(cell.occupancy.value, cell.occupancy_confidence)
        return entry
    yaw = "?" if cell.yaw_deg is None else "{:+.1f}".format(cell.yaw_deg)
    min_confidence = system.planning.min_confidence if system is not None else 0.0
    entry.update({
        "color": cell.color.value,
        "confidence": round(cell.color_confidence, 2),
        "face": cell.top_face.value,
        "yaw": cell.yaw_deg,
        "uncertain": cell.color not in KNOWN_COLORS or cell.color_confidence < min_confidence,
        "text": "{} {} yaw {} (culoare {:.2f})".format(cell.color.value, cell.top_face.value, yaw, cell.color_confidence),
    })
    metrics = cell.metrics
    if "side_w_mm" in metrics:
        entry["side_mm"] = [metrics["side_w_mm"], metrics["side_h_mm"]]
    gaps = {side: metrics[key] for side, key in GAP_METRIC_KEYS if key in metrics}
    entry["gaps_mm"] = gaps
    entry["min_gap_mm"] = min(gaps.values()) if gaps else None
    if system is not None and system.gripper is not None:
        target = Footprint((0.0, 0.0), float(cell.yaw_deg or 0.0), system.cube.size_mm)
        entry["grasp"] = plan_grasp(target, clearances_from_metrics(metrics), system.gripper, 1.0).status
    return entry


def summarize(observation: GridObservation, system: Optional[SystemConfig] = None) -> Dict[str, Any]:
    """Rezumatul unei observații pentru `/status.json`: per celulă starea, culoarea, încrederea, fața, laturile
    și spațiile măsurate (mm), starea prinderii; motivele de respingere și în clar."""
    return {
        "updated": utc_now_iso(),
        "location": observation.location,
        "rows": observation.rows,
        "cols": observation.cols,
        "actionable": observation.actionable,
        "reasons": list(observation.reasons),
        "reasons_text": [describe_reason(r) for r in observation.reasons],
        "processing_ms": observation.processing_ms,
        "cells": {"r{}c{}".format(c.coord.row, c.coord.col): _cell_summary(c, system) for c in observation.cells},
    }


class Analysis(NamedTuple):
    jpeg: bytes  # imaginea adnotată
    summary: Dict[str, Any]
    observation_dict: Dict[str, Any]
    observation: GridObservation
    overlay: List[Dict[str, Any]]  # conturul cuburilor în cadrul analizat
    image: Optional[np.ndarray]  # imaginea adnotată necodată (doar la cerere)


_WORKER: Dict[str, Any] = {}


def _init_worker(system: Any, colors: Any) -> None:
    """Rulează în procesul de analiză: pipeline-ul se construiește o singură dată."""
    _WORKER["pipeline"] = VisionPipeline(system, colors)


def analyze_frames(
    pipeline: VisionPipeline, frames: List[np.ndarray], location: str, quality: int, keep_image: bool = False
) -> Analysis:
    """Analiza unei ferestre de cadre: imaginea adnotată, rezumatul, observația și conturul cuburilor."""
    result = pipeline.analyze(frames, location)
    gripper = pipeline.system.gripper
    image = render_annotation(result, gripper)
    return Analysis(encode_jpeg(image, quality), summarize(result.observation, pipeline.system),
                    observation_to_dict(result.observation), result.observation, overlay_items(result, gripper),
                    image if keep_image else None)


def _analyze_in_worker(frames: List[np.ndarray], location: str, quality: int, keep_image: bool) -> Analysis:
    return analyze_frames(_WORKER["pipeline"], frames, location, quality, keep_image)


class PreviewSession:
    def __init__(
        self,
        source: FrameSource,
        hub: PreviewHub,
        preview: PreviewConfig,
        pipeline: Optional[VisionPipeline] = None,
        frames_to_aggregate: int = 5,
        min_valid_frames: int = 3,
        save_dir: Optional[Path] = None,
        use_process: bool = True,
        camera_text: str = "",
        overlay: bool = True,
    ) -> None:
        self.source = source
        self.hub = hub
        self.preview = preview
        self.pipeline = pipeline
        self.min_valid_frames = min_valid_frames
        self.save_dir = save_dir
        self.analyses = 0
        self.last_error: Optional[str] = None
        self.use_process = use_process
        self.analysis_mode = "none" if pipeline is None else "thread"
        self.camera_text = camera_text
        self.overlay_enabled = overlay
        #: setat de execuție: analiza periodică stă, observările sarcinii publică ele imaginile
        self.pause_analysis = threading.Event()
        self._executor: Optional[concurrent.futures.ProcessPoolExecutor] = None
        self._frames: Deque[Tuple[int, np.ndarray]] = collections.deque(maxlen=max(frames_to_aggregate, 1))
        self._seq = 0
        self._lock = threading.Lock()
        self._new_frame = threading.Condition(self._lock)
        self._overlay: List[Dict[str, Any]] = []
        self._stop = threading.Event()
        self._threads = [threading.Thread(target=self._capture_loop, name="preview-capture", daemon=True)]
        if pipeline is not None:
            self._threads.append(threading.Thread(target=self._analysis_loop, name="preview-analysis", daemon=True))

    def start(self) -> "PreviewSession":
        if self.pipeline is not None and self.use_process:
            try:
                self._executor = concurrent.futures.ProcessPoolExecutor(
                    max_workers=1, initializer=_init_worker, initargs=(self.pipeline.system, self.pipeline.colors))
                self.analysis_mode = "process"
            except (OSError, NotImplementedError, ImportError) as exc:  # ex. fără suport de procese: rămâne firul
                self.last_error = "analiza rulează în fir, nu în proces: {}".format(exc)
        self.hub.set_info(camera=self.camera_text, analysis_mode=self.analysis_mode,
                          analyze_interval_s=self.preview.analyze_interval_s)
        for thread in self._threads:
            thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        with self._new_frame:
            self._new_frame.notify_all()
        for thread in self._threads:
            thread.join(timeout=10.0)
        if self._executor is not None:
            self._executor.shutdown(wait=True)
            self._executor = None

    # ------------------------------------------------------------------ cadrele
    def fresh_frames(self, count: int, timeout_s: float = 10.0) -> List[np.ndarray]:
        """`count` cadre capturate DUPĂ apel (nu cele din buffer, de dinainte ca brațul să se fi așezat)."""
        count = max(1, min(count, self._frames.maxlen or count))
        deadline = time.monotonic() + timeout_s
        with self._new_frame:
            start = self._seq
            while self._seq - start < count:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._stop.is_set():
                    raise PerceptionInputError("camera nu a dat {} cadre noi în {:.0f} s".format(count, timeout_s))
                self._new_frame.wait(remaining)
            return [frame for seq, frame in self._frames if seq > start][-count:]

    def analyze_once(self, frames: List[np.ndarray], location: str, keep_image: bool = False) -> Analysis:
        """O analiză la cerere (în procesul de analiză, dacă există), în afara buclei periodice."""
        assert self.pipeline is not None, "sesiune fără pipeline de viziune"
        quality = self.preview.jpeg_quality
        if self._executor is not None:
            return self._executor.submit(_analyze_in_worker, frames, location, quality, keep_image).result(timeout=120.0)
        return analyze_frames(self.pipeline, frames, location, quality, keep_image)

    def set_overlay(self, items: List[Dict[str, Any]]) -> None:
        """Conturul cuburilor (cadrul analizat, îndreptat) → cadrul brut al fluxului live."""
        undistort = getattr(self.pipeline, "_undistort", None) if self.pipeline is not None else None
        width_height = self.hub.latest()[2].get("info", {}).get("size")
        if undistort is not None and items and width_height:
            converted = []
            for item in items:
                points = undistort.distort_points(np.array(item["poly"], np.float64), width_height[0], width_height[1])
                converted.append(dict(item, poly=points.tolist()))
            items = converted
        with self._lock:
            self._overlay = list(items) if self.overlay_enabled else []

    def _capture_loop(self) -> None:
        period = 1.0 / self.preview.max_fps
        due = time.monotonic()
        while not self._stop.is_set():
            try:
                frame = self.source.grab(1)[0]
            except PerceptionInputError as exc:
                self.last_error = str(exc)
                self._stop.wait(0.5)
                continue
            with self._new_frame:
                self._seq += 1
                self._frames.append((self._seq, frame))
                overlay = self._overlay
                self._new_frame.notify_all()
            if self._seq == 1:
                self.hub.set_info(resolution="{}x{}".format(frame.shape[1], frame.shape[0]),
                                  size=[int(frame.shape[1]), int(frame.shape[0])])
            shown = frame
            if overlay:
                shown = frame.copy()
                draw_overlay(shown, overlay)
            self.hub.publish_raw(encode_jpeg(shown, self.preview.jpeg_quality))
            # Termene fixe (t0 + k·perioadă): rotunjirea timer-ului (~15 ms pe Windows) nu se mai adună. Camera
            # reală dă singură ritmul (citirea așteaptă cadrul următor), deci aici de regulă nu se mai așteaptă.
            due += period
            delay = due - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            else:
                due = time.monotonic()  # în urmă: fără rafală de recuperare

    def _analysis_loop(self) -> None:
        assert self.pipeline is not None
        while not self._stop.wait(self.preview.analyze_interval_s):
            if self.pause_analysis.is_set():
                continue
            with self._lock:
                frames = [frame for _, frame in self._frames]
            if len(frames) < self.min_valid_frames:
                continue
            try:
                output = self.analyze_once(frames, self.hub.location)
                if self.pause_analysis.is_set():  # între timp a pornit execuția: imaginile ei au prioritate
                    continue
                summary = dict(output.summary, mode=self.analysis_mode)
                self.hub.publish_analysis(output.jpeg, summary, output.observation)
                self.set_overlay(output.overlay)
                if self.save_dir is not None:
                    self._save(frames[-1], output.jpeg, output.observation_dict)
                self.analyses += 1
            except Exception as exc:  # firul de analiză nu trebuie să oprească fluxul live
                self.last_error = "{}: {}".format(type(exc).__name__, exc)
                self.hub.publish_analysis(None, {"updated": utc_now_iso(), "error": self.last_error})
                self.set_overlay([])

    def _save(self, raw: np.ndarray, annotated_jpeg: bytes, observation: Dict[str, Any]) -> None:
        """Ultimele imagini și observația, suprascrise atomic (un cititor nu vede niciodată un fișier pe jumătate)."""
        folder = Path(self.save_dir)  # type: ignore[arg-type]
        tmp = write_image(folder / "latest_raw.tmp.jpg", raw)
        os.replace(str(tmp), str(folder / "latest_raw.jpg"))
        tmp_annotated = folder / "latest_annotated.tmp.jpg"
        tmp_annotated.write_bytes(annotated_jpeg)
        os.replace(str(tmp_annotated), str(folder / "latest_annotated.jpg"))
        tmp_json = folder / "latest_observation.tmp.json"
        tmp_json.write_text(json.dumps(observation, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(str(tmp_json), str(folder / "latest_observation.json"))


class SessionPerception:
    """Percepția pentru orchestrare (același contract ca `CameraPerception`) din cadrele previzualizării:
    camera nu poate fi deschisă a doua oară, iar observarea folosește doar cadre capturate după cerere."""

    def __init__(self, session: PreviewSession, frames: int, timeout_s: float = 10.0) -> None:
        self.session = session
        self.frames = frames
        self.timeout_s = timeout_s

    def perceive(self, location: str) -> Any:
        from brain.orchestration.observer import PerceptionResult

        frames = self.session.fresh_frames(self.frames, self.timeout_s)
        output = self.session.analyze_once(frames, location, keep_image=True)
        self.session.set_overlay(output.overlay)
        return PerceptionResult(output.observation, frames[-1], output.image)
