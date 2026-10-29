"""Validarea argumentelor `ArmInterface`, comună adaptoarelor (respinge înainte de orice mișcare)."""

from __future__ import annotations

import math
from typing import Any, List, Optional, Tuple

from brain.domain.errors import InvalidCommand

SPEEDS = ("slow", "medium", "fast")
MAX_LEVEL = 8
MAX_TIME_MS = 10000


def check_speed(speed: Any) -> str:
    if speed not in SPEEDS:
        raise InvalidCommand("speed invalid: {!r} (permise: {})".format(speed, ", ".join(SPEEDS)))
    return speed


def check_angles(angles: Any, low: float = 0.0, high: float = 180.0) -> List[float]:
    if not isinstance(angles, (list, tuple)) or len(angles) not in (5, 6):
        raise InvalidCommand("angles trebuie să fie o listă de 5 sau 6 valori, nu {!r}".format(angles))
    result = []
    for index, value in enumerate(angles):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise InvalidCommand("unghiul servo {} invalid: {!r}".format(index + 1, value))
        if not low <= value <= high:
            raise InvalidCommand("unghiul servo {} = {} în afara [{}, {}]".format(index + 1, value, low, high))
        result.append(float(value))
    return result


def check_time_ms(time_ms: Any) -> int:
    if isinstance(time_ms, bool) or not isinstance(time_ms, int) or not 0 <= time_ms <= MAX_TIME_MS:
        raise InvalidCommand("time_ms trebuie să fie întreg în [0, {}], nu {!r}".format(MAX_TIME_MS, time_ms))
    return time_ms


def check_cell(cell: Any) -> Optional[Tuple[int, int]]:
    if cell is None:
        return None
    if (
        not isinstance(cell, (list, tuple))
        or len(cell) != 2
        or any(isinstance(v, bool) or not isinstance(v, int) for v in cell)
        or not all(0 <= v <= 2 for v in cell)
    ):
        raise InvalidCommand("cell trebuie să fie (r, c) cu valori 0..2, nu {!r}".format(cell))
    return (int(cell[0]), int(cell[1]))


def check_level(level: Any) -> int:
    if isinstance(level, bool) or not isinstance(level, int) or not 0 <= level <= MAX_LEVEL:
        raise InvalidCommand("level trebuie să fie întreg 0..{}, nu {!r}".format(MAX_LEVEL, level))
    return level


def check_yaw(yaw_deg: Any) -> float:
    if isinstance(yaw_deg, bool) or not isinstance(yaw_deg, (int, float)) or not math.isfinite(yaw_deg):
        raise InvalidCommand("yaw_deg invalid: {!r}".format(yaw_deg))
    return float(yaw_deg)


def check_uv(uv: Any, cell: Optional[Tuple[int, int]]) -> Optional[Tuple[float, float]]:
    if uv is None:
        return None
    if cell is None:
        raise InvalidCommand("uv are sens doar pentru celule de caroiaj")
    if (
        not isinstance(uv, (list, tuple))
        or len(uv) != 2
        or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in uv)
        or not all(0.0 <= v <= 3.0 for v in uv)
    ):
        raise InvalidCommand("uv trebuie să fie [u, v] în 0..3, nu {!r}".format(uv))
    return (float(uv[0]), float(uv[1]))


MAX_OPENING_MM = 200.0


def check_opening(value: Any, name: str) -> Optional[float]:
    """Deschiderea ghearei în mm (interioară) sau None (serviciul alege)."""
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise InvalidCommand("{} invalid: {!r}".format(name, value))
    if not 0.0 < value <= MAX_OPENING_MM:
        raise InvalidCommand("{} = {} în afara (0, {}] mm".format(name, value, MAX_OPENING_MM))
    return float(value)


def check_location(location: Any) -> str:
    if not isinstance(location, str) or not location:
        raise InvalidCommand("location trebuie să fie un șir nevid, nu {!r}".format(location))
    return location


def check_beep(count: Any, duration_ms: Any) -> Tuple[int, int]:
    if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 10:
        raise InvalidCommand("count trebuie să fie întreg 1..10, nu {!r}".format(count))
    if isinstance(duration_ms, bool) or not isinstance(duration_ms, int) or not 1 <= duration_ms <= 5000:
        raise InvalidCommand("duration_ms trebuie să fie întreg 1..5000, nu {!r}".format(duration_ms))
    return count, duration_ms
