"""`python -m brain camera preview`: captură continuă + analiză periodică, publicate în `PreviewHub`.

Captura și analiza sunt separate: pe Pi analiza poate dura aproape o secundă, iar fluxul live (30 fps) nu
trebuie să se blocheze. Analiza rulează implicit într-un PROCES separat: în același proces Python ar concura
cu firul de captură (GIL) și ar scădea cadrele pe secundă; dacă procesul nu poate porni, rămâne un fir.
Analiza folosește ultimele `frames_to_aggregate` cadre, ca `vision analyze`.
"""

from __future__ import annotations

import collections
import concurrent.futures
import json
import os
import threading
import time
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import cv2
import numpy as np

from brain.adapters.preview_server import PreviewHub
from brain.configuration.models import PreviewConfig
from brain.domain.cube import Occupancy
from brain.domain.errors import PerceptionInputError
from brain.domain.observation import GridObservation
from brain.domain.scene_state import utc_now_iso
from brain.memory.codec import observation_to_dict
from brain.perception.annotate import render_annotation
from brain.perception.image_io import write_image
from brain.perception.pipeline import VisionPipeline
from brain.ports.sensors import FrameSource


def encode_jpeg(image: np.ndarray, quality: int) -> bytes:
    ok, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise PerceptionInputError("cadrul nu poate fi codat JPEG")
    return buffer.tobytes()


def summarize(observation: GridObservation) -> Dict[str, Any]:
    """Rezumatul unei observații pentru `/status.json` (o linie per celulă)."""
    cells = {}
    for cell in observation.cells:
        key = "r{}c{}".format(cell.coord.row, cell.coord.col)
        if cell.occupancy is Occupancy.OCCUPIED:
            yaw = "?" if cell.yaw_deg is None else "{:+.1f}".format(cell.yaw_deg)
            cells[key] = "{} {} yaw {} (culoare {:.2f})".format(cell.color.value, cell.top_face.value, yaw, cell.color_confidence)
        else:
            cells[key] = "{} ({:.2f})".format(cell.occupancy.value, cell.occupancy_confidence)
    return {
        "updated": utc_now_iso(),
        "actionable": observation.actionable,
        "reasons": list(observation.reasons),
        "processing_ms": observation.processing_ms,
        "cells": cells,
    }


_WORKER: Dict[str, Any] = {}


def _init_worker(system: Any, colors: Any) -> None:
    """Rulează în procesul de analiză: pipeline-ul se construiește o singură dată."""
    _WORKER["pipeline"] = VisionPipeline(system, colors)


def analyze_frames(
    pipeline: VisionPipeline, frames: List[np.ndarray], location: str, quality: int
) -> Tuple[bytes, Dict[str, Any], Dict[str, Any]]:
    """Analiza unei ferestre de cadre: (JPEG adnotat, rezumat pentru /status.json, observația ca dict)."""
    result = pipeline.analyze(frames, location)
    return (encode_jpeg(render_annotation(result), quality), summarize(result.observation),
            observation_to_dict(result.observation))


def _analyze_in_worker(frames: List[np.ndarray], location: str, quality: int) -> Tuple[bytes, Dict[str, Any], Dict[str, Any]]:
    return analyze_frames(_WORKER["pipeline"], frames, location, quality)


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
        self._executor: Optional[concurrent.futures.ProcessPoolExecutor] = None
        self._frames: Deque[np.ndarray] = collections.deque(maxlen=frames_to_aggregate)
        self._lock = threading.Lock()
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
        for thread in self._threads:
            thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=10.0)
        if self._executor is not None:
            self._executor.shutdown(wait=True)
            self._executor = None

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
            with self._lock:
                self._frames.append(frame)
            self.hub.publish_raw(encode_jpeg(frame, self.preview.jpeg_quality))
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
            with self._lock:
                frames = list(self._frames)
            if len(frames) < self.min_valid_frames:
                continue
            quality = self.preview.jpeg_quality
            try:
                if self._executor is not None:
                    future = self._executor.submit(_analyze_in_worker, frames, self.hub.location, quality)
                    annotated, summary, observation = future.result(timeout=120.0)
                else:
                    annotated, summary, observation = analyze_frames(self.pipeline, frames, self.hub.location, quality)
                summary["mode"] = self.analysis_mode
                self.hub.publish_analysis(annotated, summary)
                if self.save_dir is not None:
                    self._save(frames[-1], annotated, observation)
                self.analyses += 1
            except Exception as exc:  # firul de analiză nu trebuie să oprească fluxul live
                self.last_error = "{}: {}".format(type(exc).__name__, exc)
                self.hub.publish_analysis(None, {"updated": utc_now_iso(), "error": self.last_error})

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
