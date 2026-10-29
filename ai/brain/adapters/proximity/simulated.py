"""Senzor de proximitate simulat, model HC-SR04 (ultrasonic, 20-4000 mm, rezoluție ~3 mm).

Montaj simulat: coaxial pe gheară, privind în jos printre fălci, la `sensor_above_jaw_mm` deasupra
vârfului fălcilor. După pick/place serviciul lasă gheara la `hover_above_target_mm` deasupra țintei.
Un cub ținut în gheară e chiar sub senzor (sub distanța minimă) → citire BELOW_RANGE.
Erorile de măsurare se injectează determinist: timeout, deconectare, date invalide.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from brain.configuration.models import ProximityConfig
from brain.domain.proximity import ProximityReading, SensorSpec, SensorStatus, Validity
from brain.ports.sensors import Clock
from brain.simulation.sim_world import SimWorld

GRASP_HEIGHT_FRACTION = 0.5  # serviciul prinde cubul la jumătatea înălțimii lui


@dataclass(frozen=True)
class ProximityFaults:
    timeout_probability: float = 0.0
    invalid_probability: float = 0.0
    disconnect_after: Optional[int] = None  # după atâtea citiri, senzorul se deconectează


class SimulatedProximity:
    def __init__(
        self,
        world: SimWorld,
        config: ProximityConfig,
        clock: Clock,
        sensor_above_jaw_mm: float = 25.0,
        noise_mm: float = 1.5,
        faults: Optional[ProximityFaults] = None,
        seed: int = 0,
    ) -> None:
        self.world = world
        self.config = config
        self.clock = clock
        self.sensor_above_jaw_mm = float(sensor_above_jaw_mm)
        self.noise_mm = float(noise_mm)
        self.faults = faults or ProximityFaults()
        self.rng = np.random.default_rng(seed)
        self._sequence = 0
        self.observe_distance_mm = 250.0  # brațul într-o poză înaltă (observare): masa e departe

    def spec(self) -> SensorSpec:
        return SensorSpec(
            model="HC-SR04 (simulat)",
            interface="simulare",
            min_range_mm=self.config.min_range_mm,
            max_range_mm=self.config.max_range_mm,
            resolution_mm=self.config.resolution_mm,
            max_rate_hz=self.config.max_rate_hz,
            beam_half_angle_deg=self.config.beam_half_angle_deg,
            confirmed=False,
        )

    def _reading(self, value: Optional[float], validity: Validity, status: SensorStatus = SensorStatus.OK) -> ProximityReading:
        self._sequence += 1
        return ProximityReading(value, validity, status, self.clock.monotonic(), self._sequence)

    def true_distance_mm(self) -> Optional[float]:
        """Distanța reală senzor → suprafață; None dacă un cub ținut blochează fasciculul."""
        hover = self.world.hover
        if hover is None:
            return self.observe_distance_mm
        if self.world.held is not None:
            return None
        cube = self.world.cube_mm
        jaw_z = cube * GRASP_HEIGHT_FRACTION + hover.level * cube + self.config.hover_above_target_mm
        surface = self.world.surface_height_mm(hover.location, hover.cell)
        return jaw_z + self.sensor_above_jaw_mm - surface

    def read(self) -> ProximityReading:
        self.clock.sleep(1.0 / self.config.max_rate_hz)
        if self.faults.disconnect_after is not None and self._sequence >= self.faults.disconnect_after:
            return self._reading(None, Validity.DISCONNECTED, SensorStatus.DISCONNECTED)
        if self.rng.random() < self.faults.timeout_probability:
            return self._reading(None, Validity.TIMEOUT, SensorStatus.DEGRADED)
        if self.rng.random() < self.faults.invalid_probability:
            return self._reading(None, Validity.INVALID_DATA, SensorStatus.DEGRADED)
        distance = self.true_distance_mm()
        if distance is None or distance < self.config.min_range_mm:
            return self._reading(None, Validity.BELOW_RANGE)
        measured = distance + float(self.rng.normal(0.0, self.noise_mm))
        measured = round(measured / self.config.resolution_mm) * self.config.resolution_mm
        if measured > self.config.max_range_mm:
            return self._reading(None, Validity.ABOVE_RANGE)
        return self._reading(float(max(measured, 0.0)), Validity.VALID)
