"""Calitatea cadrelor, agregarea robustă a mai multor cadre și balansul de alb software."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from brain.configuration.models import QualityThresholds
from brain.domain.errors import PerceptionInputError

FLAG_OVEREXPOSED = "overexposed"
FLAG_UNDEREXPOSED = "underexposed"
FLAG_LOW_CONTRAST = "low_contrast"

SATURATED_LEVEL = 250
DARK_LEVEL = 15


@dataclass(frozen=True)
class FrameQuality:
    mean: float
    contrast: float
    saturated_fraction: float
    dark_fraction: float
    sharpness: float
    flags: Tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.flags


def validate_frames(frames: Sequence[np.ndarray]) -> List[np.ndarray]:
    """Cadrele trebuie să fie BGR uint8, nevide, toate de aceeași formă."""
    if not isinstance(frames, (list, tuple)) or not frames:
        raise PerceptionInputError("trebuie cel puțin un cadru")
    shape = None
    result: List[np.ndarray] = []
    for index, frame in enumerate(frames):
        if not isinstance(frame, np.ndarray):
            raise PerceptionInputError("cadrul {} nu e numpy.ndarray".format(index))
        if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
            raise PerceptionInputError(
                "cadrul {} trebuie să fie BGR uint8 (H, W, 3), nu {} {}".format(index, frame.dtype, frame.shape)
            )
        if frame.shape[0] < 32 or frame.shape[1] < 32:
            raise PerceptionInputError("cadrul {} e prea mic: {}".format(index, frame.shape))
        if shape is None:
            shape = frame.shape
        elif frame.shape != shape:
            raise PerceptionInputError("cadrul {} are forma {}, diferită de {}".format(index, frame.shape, shape))
        result.append(frame)
    return result


def assess_frame(image: np.ndarray, thresholds: QualityThresholds) -> FrameQuality:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mean = float(gray.mean())
    contrast = float(gray.std())
    saturated = float((gray >= SATURATED_LEVEL).mean())
    dark = float((gray <= DARK_LEVEL).mean())
    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    flags: List[str] = []
    if mean > thresholds.max_mean_brightness or saturated > thresholds.max_saturated_fraction:
        flags.append(FLAG_OVEREXPOSED)
    if mean < thresholds.min_mean_brightness or dark > thresholds.max_dark_fraction:
        flags.append(FLAG_UNDEREXPOSED)
    if contrast < thresholds.min_contrast:
        flags.append(FLAG_LOW_CONTRAST)
    return FrameQuality(mean, contrast, saturated, dark, sharpness, tuple(flags))


def aggregate_frames(frames: Sequence[np.ndarray]) -> np.ndarray:
    """Mediana per pixel: elimină zgomotul și obiectele tranzitorii prezente într-o minoritate de cadre."""
    if len(frames) == 1:
        return frames[0].copy()
    stack = np.stack(frames, axis=0)
    return np.clip(np.rint(np.median(stack, axis=0)), 0, 255).astype(np.uint8)


def chroma(image: np.ndarray) -> np.ndarray:
    """max(B,G,R) - min(B,G,R): aproape 0 pentru alb/gri/negru, mare pentru culori saturate."""
    return (image.max(axis=2).astype(np.int16) - image.min(axis=2).astype(np.int16)).astype(np.uint8)


def white_reference_mask(image: np.ndarray, chroma_max: int = 45) -> np.ndarray:
    """Pixelii luminoși și necolorați (masa albă), folosiți ca referință pentru balansul de alb."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    bright = gray >= np.percentile(gray, 40)
    return (bright & (chroma(image) <= chroma_max)).astype(np.uint8) * 255


def estimate_white_balance(
    image: np.ndarray, reference_mask: np.ndarray, max_gain: float, min_pixels: int
) -> Optional[Tuple[float, float, float]]:
    """Câștiguri per canal (B, G, R) care fac referința neutră; `None` dacă referința e prea mică."""
    pixels = image[reference_mask > 0].astype(np.float64)
    if pixels.shape[0] < min_pixels:
        return None
    means = pixels.mean(axis=0)
    if np.any(means < 1.0):
        return None
    gains = np.clip(means.mean() / means, 1.0 / max_gain, max_gain)
    return (float(gains[0]), float(gains[1]), float(gains[2]))


def apply_gains(image: np.ndarray, gains: Tuple[float, float, float]) -> np.ndarray:
    scaled = image.astype(np.float32) * np.asarray(gains, dtype=np.float32).reshape(1, 1, 3)
    return np.clip(np.rint(scaled), 0, 255).astype(np.uint8)
