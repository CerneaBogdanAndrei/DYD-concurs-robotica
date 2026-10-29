"""Surse de cadre: camera sintetică (simulare), fișiere statice și camera USB.

Camera USB nu a fost testată pe hardware-ul real; expunerea și balansul de alb nu sunt fixate aici
(Etapa 1). Cadrele vechi din buffer sunt aruncate înainte de captură.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

from brain.domain.errors import PerceptionInputError
from brain.perception.image_io import read_image


class SyntheticCamera:
    """Randează locația văzută din poza curentă a brațului simulat (`pose_of()` → numele pozei)."""

    def __init__(self, render: Callable[[str, int], List[np.ndarray]], pose_to_location: Dict[str, str],
                 pose_of: Callable[[], Optional[str]]) -> None:
        self._render = render
        self._pose_to_location = dict(pose_to_location)
        self._pose_of = pose_of

    def grab(self, count: int) -> List[np.ndarray]:
        pose = self._pose_of()
        location = self._pose_to_location.get(pose or "")
        if location is None:
            raise PerceptionInputError("brațul nu e într-o poză de observare (poza curentă: {})".format(pose))
        return self._render(location, count)


class FileCamera:
    """Cadre din fișiere, câte un set per poză de observare (pentru teste cu poze reale)."""

    def __init__(self, frames_by_pose: Dict[str, Sequence[str]], pose_of: Callable[[], Optional[str]]) -> None:
        self._frames = {pose: [Path(p) for p in paths] for pose, paths in frames_by_pose.items()}
        self._pose_of = pose_of

    def grab(self, count: int) -> List[np.ndarray]:
        pose = self._pose_of()
        paths = self._frames.get(pose or "")
        if not paths:
            raise PerceptionInputError("nu există imagini pentru poza {}".format(pose))
        return [read_image(path) for path in paths[:count]]


class LoopingFileCamera:
    """Cadre din fișiere, reluate în buclă (previzualizare și teste fără cameră)."""

    def __init__(self, paths: Sequence[str]) -> None:
        self._frames = [read_image(path) for path in paths]
        if not self._frames:
            raise PerceptionInputError("nu există imagini pentru camera din fișiere")
        self._next = 0

    def grab(self, count: int) -> List[np.ndarray]:
        frames = []
        for _ in range(count):
            frames.append(self._frames[self._next].copy())
            self._next = (self._next + 1) % len(self._frames)
        return frames


class UsbCamera:  # pragma: no cover - necesită cameră fizică
    def __init__(self, index: int = 0, width: int = 640, height: int = 480, flush: int = 4) -> None:
        import cv2

        self._cv2 = cv2
        self._capture = cv2.VideoCapture(index)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._flush = flush
        if not self._capture.isOpened():
            raise PerceptionInputError("camera {} nu poate fi deschisă (index, cablu, alt program o folosește?)".format(index))

    def grab(self, count: int) -> List[np.ndarray]:
        for _ in range(self._flush):
            self._capture.grab()
        frames = []
        for _ in range(count):
            ok, frame = self._capture.read()
            if ok and frame is not None:
                frames.append(frame)
        if not frames:
            raise PerceptionInputError("camera nu returnează cadre")
        return frames

    def release(self) -> None:
        self._capture.release()
