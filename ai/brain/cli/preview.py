"""`python -m brain camera preview`: captură continuă + analiză periodică, publicate în `PreviewHub`.

Captura și analiza rulează pe fire separate: pe Pi analiza poate dura peste o secundă, iar fluxul live
nu trebuie să se blocheze. Analiza folosește ultimele `frames_to_aggregate` cadre, ca `vision analyze`.
"""

from __future__ import annotations

import collections
import json
import os
import threading
import time
from pathlib import Path
from typing import Any, Deque, Dict, Optional

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
    ) -> None:
        self.source = source
        self.hub = hub
        self.preview = preview
        self.pipeline = pipeline
        self.min_valid_frames = min_valid_frames
        self.save_dir = save_dir
        self.analyses = 0
        self.last_error: Optional[str] = None
        self._frames: Deque[np.ndarray] = collections.deque(maxlen=frames_to_aggregate)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = [threading.Thread(target=self._capture_loop, name="preview-capture", daemon=True)]
        if pipeline is not None:
            self._threads.append(threading.Thread(target=self._analysis_loop, name="preview-analysis", daemon=True))

    def start(self) -> "PreviewSession":
        for thread in self._threads:
            thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=5.0)

    def _capture_loop(self) -> None:
        period = 1.0 / self.preview.max_fps
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                frame = self.source.grab(1)[0]
            except PerceptionInputError as exc:
                self.last_error = str(exc)
                self._stop.wait(0.5)
                continue
            with self._lock:
                self._frames.append(frame)
            self.hub.publish_raw(encode_jpeg(frame, self.preview.jpeg_quality))
            self._stop.wait(max(0.0, period - (time.monotonic() - started)))

    def _analysis_loop(self) -> None:
        assert self.pipeline is not None
        while not self._stop.wait(self.preview.analyze_interval_s):
            with self._lock:
                frames = list(self._frames)
            if len(frames) < self.min_valid_frames:
                continue
            try:
                result = self.pipeline.analyze(frames, self.hub.location)
                annotated = render_annotation(result)
                summary = summarize(result.observation)
                self.hub.publish_analysis(encode_jpeg(annotated, self.preview.jpeg_quality), summary)
                if self.save_dir is not None:
                    self._save(frames[-1], annotated, result.observation)
                self.analyses += 1
            except Exception as exc:  # firul de analiză nu trebuie să oprească fluxul live
                self.last_error = "{}: {}".format(type(exc).__name__, exc)
                self.hub.publish_analysis(None, {"updated": utc_now_iso(), "error": self.last_error})

    def _save(self, raw: np.ndarray, annotated: np.ndarray, observation: GridObservation) -> None:
        """Ultimele imagini și observația, suprascrise atomic (un cititor nu vede niciodată un fișier pe jumătate)."""
        folder = Path(self.save_dir)  # type: ignore[arg-type]
        for name, image in (("latest_raw", raw), ("latest_annotated", annotated)):
            tmp = write_image(folder / "{}.tmp.jpg".format(name), image)
            os.replace(str(tmp), str(folder / "{}.jpg".format(name)))
        tmp_json = folder / "latest_observation.tmp.json"
        tmp_json.write_text(json.dumps(observation_to_dict(observation), ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(str(tmp_json), str(folder / "latest_observation.json"))
