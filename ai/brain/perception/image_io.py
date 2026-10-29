"""Citire/scriere de imagini care funcționează și cu căi non-ASCII pe Windows."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
import numpy as np

from brain.domain.errors import PerceptionInputError


def read_image(path: Any) -> np.ndarray:
    """Citește o imagine BGR uint8. `cv2.imread` eșuează silențios pe căi Unicode în Windows."""
    file = Path(path)
    if not file.is_file():
        raise PerceptionInputError("imaginea nu există: {}".format(file))
    data = np.fromfile(str(file), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise PerceptionInputError("imaginea nu poate fi decodată: {}".format(file))
    return image


def write_image(path: Any, image: np.ndarray, jpeg_quality: int = 90) -> Path:
    file = Path(path)
    suffix = file.suffix.lower()
    if suffix not in (".jpg", ".jpeg", ".png"):
        raise PerceptionInputError("format de imagine nesuportat: {}".format(file))
    params = [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)] if suffix in (".jpg", ".jpeg") else []
    ok, buffer = cv2.imencode(suffix, image, params)
    if not ok:
        raise PerceptionInputError("imaginea nu poate fi codată: {}".format(file))
    file.parent.mkdir(parents=True, exist_ok=True)
    buffer.tofile(str(file))
    return file
