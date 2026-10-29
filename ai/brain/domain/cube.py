"""Proprietățile observabile ale unui cub și convenția de yaw."""

from __future__ import annotations

import math
from enum import Enum
from typing import Tuple

from brain.domain.errors import DomainValidationError


class Color(str, Enum):
    RED = "red"
    YELLOW = "yellow"
    GREEN = "green"
    BLUE = "blue"
    UNKNOWN = "unknown"


KNOWN_COLORS: Tuple[Color, ...] = (Color.RED, Color.YELLOW, Color.GREEN, Color.BLUE)


class TopFace(str, Enum):
    """Tipul feței superioare: complet colorată sau albă cu pictogramă și bordură colorată."""

    FULL = "full"
    ICON = "icon"
    UNKNOWN = "unknown"


class Occupancy(str, Enum):
    OCCUPIED = "occupied"
    EMPTY = "empty"
    UNKNOWN = "unknown"


YAW_MIN_DEG = -45.0
YAW_MAX_DEG = 45.0  # exclusiv


def normalize_yaw_deg(angle_deg: float) -> float:
    """Reduce un unghi la [-45, 45): silueta cubului e invariantă la rotații de 90°.

    Convenție: yaw pozitiv = sens antiorar văzut de sus, relativ la axele caroiajului.
    """
    if not math.isfinite(angle_deg):
        raise DomainValidationError("yaw nefinit: {!r}".format(angle_deg))
    value = math.fmod(angle_deg + 45.0, 90.0)
    if value < 0:
        value += 90.0
    value -= 45.0
    # fmod poate întoarce exact 45.0 din cauza rotunjirii
    if value >= YAW_MAX_DEG:
        value -= 90.0
    return value


def yaw_difference_deg(a_deg: float, b_deg: float) -> float:
    """Diferența minimă dintre două yaw-uri, ținând cont de simetria la 90°; rezultat în [0, 45]."""
    return abs(normalize_yaw_deg(a_deg - b_deg))


def check_confidence(value: float, name: str) -> None:
    if not (isinstance(value, (int, float)) and not isinstance(value, bool)) or not (0.0 <= value <= 1.0):
        raise DomainValidationError("{} trebuie să fie în [0, 1], nu {!r}".format(name, value))


def check_yaw(value: float, name: str = "yaw_deg") -> None:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        raise DomainValidationError("{} invalid: {!r}".format(name, value))
    if not (YAW_MIN_DEG <= value < YAW_MAX_DEG):
        raise DomainValidationError("{} trebuie să fie în [-45, 45), nu {}".format(name, value))
