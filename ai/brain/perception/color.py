"""Clasificarea culorii dominante prin vot pe nuanță (HSV, convenția OpenCV)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np

from brain.configuration.models import ColorsConfig
from brain.domain.cube import Color

REASON_FEW_PIXELS = "insufficient_color_pixels"
REASON_AMBIGUOUS_HUE = "ambiguous_hue"
REASON_LOW_SATURATION = "low_saturation"

LOW_SATURATION_MARGIN = 30


@dataclass(frozen=True)
class ColorVote:
    color: Color
    confidence: float
    dominance: float
    pixel_count: int
    fractions: Dict[str, float]
    reasons: Tuple[str, ...]


class HueClassifier:
    def __init__(self, config: ColorsConfig) -> None:
        self.config = config
        self.colors = [cls.color for cls in config.classes]
        self._lut = np.full(180, -1, dtype=np.int16)
        for index, cls in enumerate(config.classes):
            for low, high in cls.hue_ranges:
                self._lut[low:high + 1] = index

    def hue_classes(self, hsv: np.ndarray) -> np.ndarray:
        """Indicele clasei de culoare pentru fiecare pixel după nuanță (-1 = nicio clasă)."""
        return self._lut[hsv[..., 0].astype(np.intp)]

    def colored_mask(self, hsv: np.ndarray) -> np.ndarray:
        """Pixeli suficient de saturați și luminoși pentru a avea o nuanță măsurabilă."""
        mask = (hsv[..., 1] >= self.config.saturation_min) & (hsv[..., 2] >= self.config.value_min)
        return mask.astype(np.uint8) * 255

    def vote(self, hsv_pixels: np.ndarray, min_pixels: int) -> ColorVote:
        """`hsv_pixels`: (N, 3) pixeli deja filtrați ca fiind colorați."""
        pixels = np.asarray(hsv_pixels).reshape(-1, 3)
        count = int(pixels.shape[0])
        if count < min_pixels:
            return ColorVote(Color.UNKNOWN, 0.0, 0.0, count, {}, (REASON_FEW_PIXELS,))
        indices = self._lut[np.clip(pixels[:, 0].astype(np.int64), 0, 179)]
        counts = np.bincount(indices[indices >= 0], minlength=len(self.colors)).astype(np.float64)
        fractions_arr = counts / count
        fractions = {color.value: float(f) for color, f in zip(self.colors, fractions_arr)}
        best = int(np.argmax(fractions_arr))
        dominance = float(fractions_arr[best])
        min_dom = self.config.min_dominance
        if dominance < min_dom:
            return ColorVote(Color.UNKNOWN, 0.0, dominance, count, fractions, (REASON_AMBIGUOUS_HUE,))
        confidence = 1.0 if min_dom >= 1.0 else 0.5 + 0.5 * (dominance - min_dom) / (1.0 - min_dom)
        reasons: Tuple[str, ...] = ()
        if float(np.median(pixels[:, 1])) < self.config.saturation_min + LOW_SATURATION_MARGIN:
            confidence *= 0.8
            reasons = (REASON_LOW_SATURATION,)
        return ColorVote(self.colors[best], float(min(1.0, confidence)), dominance, count, fractions, reasons)
