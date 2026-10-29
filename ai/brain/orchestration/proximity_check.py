"""Verificarea prin senzorul de proximitate după pick/place (montaj coaxial, privește în jos printre fălci).

După pick/place serviciul lasă gheara la `hover_above_target_mm` deasupra punctului de prindere (jumătatea
cubului). Cu `s` = `sensor_above_jaw_mm` (distanța senzor → vârful fălcilor) și cubul de latură `a`:
- cub ținut în gheară → chiar sub senzor → BELOW_RANGE;
- cub sub gheară (după place reușit, sau pick ratat) → hover + s − a/2 ≈ 30 + s mm;
- nimic sub gheară (cubul a căzut / lipsește) → hover + s + a/2 ≈ 60 + s mm.
Cu `s` necunoscut (NECALIBRAT) se poate decide doar „ținut / neținut”. Pragurile nu sunt validate pe hardware.
"""

from __future__ import annotations

import statistics
from typing import List, Optional

from brain.configuration.models import ProximityConfig
from brain.domain.proximity import Validity
from brain.ports.sensors import ProximitySensorInterface

HELD = "held"
NOT_HELD = "not_held"
CUBE_BELOW = "cube_below"
NOTHING_BELOW = "nothing_below"
UNKNOWN = "unknown"


class ProximityVerifier:
    def __init__(self, sensor: ProximitySensorInterface, config: ProximityConfig, cube_mm: float,
                 sensor_above_jaw_mm: Optional[float] = None) -> None:
        self.sensor = sensor
        self.config = config
        self.cube_mm = float(cube_mm)
        self.s = sensor_above_jaw_mm if sensor_above_jaw_mm is not None else config.sensor_above_jaw_mm

    def _readings(self) -> List:
        return [self.sensor.read() for _ in range(self.config.samples)]

    def _classify(self) -> str:
        readings = self._readings()
        below = sum(1 for r in readings if r.validity is Validity.BELOW_RANGE)
        valid = [r.value_mm for r in readings if r.is_valid and r.value_mm is not None]
        if below > len(readings) // 2:
            return HELD
        if len(valid) <= len(readings) // 2:
            return UNKNOWN  # majoritatea citirilor sunt timeout / invalide / deconectat: nu decidem
        distance = statistics.median(valid)
        if self.s is None:
            return NOT_HELD
        hover = self.config.hover_above_target_mm
        expected_cube = hover + self.s - self.cube_mm / 2.0
        expected_empty = hover + self.s + self.cube_mm / 2.0
        if abs(distance - expected_cube) <= self.config.tolerance_mm:
            return CUBE_BELOW
        if abs(distance - expected_empty) <= self.config.tolerance_mm:
            return NOTHING_BELOW
        return NOT_HELD

    def after_pick(self) -> str:
        """HELD = cubul e în gheară; NOT_HELD / CUBE_BELOW / NOTHING_BELOW = nu e; UNKNOWN = senzorul nu decide."""
        return self._classify()

    def after_place(self) -> str:
        """CUBE_BELOW = cubul e pe țintă; NOTHING_BELOW = a căzut; HELD = n-a fost eliberat; UNKNOWN / NOT_HELD = nedecis."""
        return self._classify()
