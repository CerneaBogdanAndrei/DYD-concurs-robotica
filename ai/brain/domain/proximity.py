"""Citirile senzorului de proximitate, normalizate (mm), cu validitate explicită.

Lipsa unei citiri valide NU înseamnă absența obstacolelor: consumatorii trebuie să trateze separat
fiecare stare de validitate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from brain.domain.errors import DomainValidationError


class Validity(str, Enum):
    VALID = "valid"
    BELOW_RANGE = "below_range"  # mai aproape decât distanța minimă (ex. obiect ținut în gheară)
    ABOVE_RANGE = "above_range"  # niciun ecou în domeniu
    TIMEOUT = "timeout"
    DISCONNECTED = "disconnected"
    INVALID_DATA = "invalid_data"
    NOT_READY = "not_ready"


class SensorStatus(str, Enum):
    OK = "ok"
    DEGRADED = "degraded"
    FAULT = "fault"
    DISCONNECTED = "disconnected"


@dataclass(frozen=True)
class SensorSpec:
    """Caracteristicile declarate. `confirmed=False` până la identificarea modelului real."""

    model: str
    interface: str
    min_range_mm: float
    max_range_mm: float
    resolution_mm: float
    max_rate_hz: float
    beam_half_angle_deg: float
    confirmed: bool


@dataclass(frozen=True)
class ProximityReading:
    value_mm: Optional[float]
    validity: Validity
    status: SensorStatus
    t_monotonic: float
    sequence: int
    unit: str = "mm"

    def __post_init__(self) -> None:
        if self.unit != "mm":
            raise DomainValidationError("unitatea citirii trebuie să fie mm")
        if self.validity is Validity.VALID:
            if self.value_mm is None or not math.isfinite(self.value_mm) or self.value_mm < 0:
                raise DomainValidationError("o citire validă trebuie să aibă o valoare finită >= 0")
        elif self.value_mm is not None:
            raise DomainValidationError("o citire nevalidă ({}) nu poate avea valoare".format(self.validity.value))

    @property
    def is_valid(self) -> bool:
        return self.validity is Validity.VALID
