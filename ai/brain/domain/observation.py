"""Rezultatul percepției pentru un caroiaj observat dintr-o singură poziție."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from brain.domain.cube import Color, Occupancy, TopFace, check_confidence, check_yaw
from brain.domain.errors import DomainValidationError
from brain.domain.grid import GridCoord, iter_coords


@dataclass(frozen=True)
class CellObservation:
    """Ce s-a observat într-o celulă. Valorile neobservabile rămân `unknown` / `None`."""

    coord: GridCoord
    occupancy: Occupancy
    occupancy_confidence: float
    color: Color = Color.UNKNOWN
    color_confidence: float = 0.0
    yaw_deg: Optional[float] = None
    yaw_confidence: float = 0.0
    top_face: TopFace = TopFace.UNKNOWN
    reasons: Tuple[str, ...] = ()
    metrics: Dict[str, float] = field(default_factory=dict, compare=False)
    # Centrul feței de sus în coordonate de celulă (u = coloană, v = linie, 0..3), NECORECTAT de paralaxă.
    uv_top: Optional[Tuple[float, float]] = None

    def __post_init__(self) -> None:
        if self.uv_top is not None:
            if len(self.uv_top) != 2 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in self.uv_top):
                raise DomainValidationError("uv_top invalid: {!r}".format(self.uv_top))
            if self.occupancy is not Occupancy.OCCUPIED:
                raise DomainValidationError("uv_top există doar pentru celule ocupate")
        check_confidence(self.occupancy_confidence, "occupancy_confidence")
        check_confidence(self.color_confidence, "color_confidence")
        check_confidence(self.yaw_confidence, "yaw_confidence")
        if self.yaw_deg is not None:
            check_yaw(self.yaw_deg)
        elif self.yaw_confidence != 0.0:
            raise DomainValidationError("yaw_confidence trebuie să fie 0 când yaw_deg lipsește")
        if self.occupancy is not Occupancy.OCCUPIED:
            if self.color is not Color.UNKNOWN or self.yaw_deg is not None or self.top_face is not TopFace.UNKNOWN:
                raise DomainValidationError(
                    "celula {} nu e ocupată, dar are culoare/yaw/față: {}".format(self.coord, self)
                )
        if self.color is Color.UNKNOWN and self.color_confidence != 0.0:
            raise DomainValidationError("color_confidence trebuie să fie 0 pentru culoare necunoscută")


@dataclass(frozen=True)
class FrameQualitySummary:
    frames_total: int
    frames_valid: int
    flags: Tuple[str, ...] = ()
    mean_brightness: float = 0.0
    contrast: float = 0.0
    sharpness: float = 0.0


@dataclass(frozen=True)
class GeometrySummary:
    """Geometria grilei în imaginea agregată. `corners_px`: colțurile exterioare TL, TR, BR, BL."""

    method: str
    ok: bool
    corners_px: Tuple[Tuple[float, float], ...] = ()
    structure_score: Optional[float] = None
    metrics: Dict[str, float] = field(default_factory=dict, compare=False)


@dataclass(frozen=True)
class GridObservation:
    """Observația unui caroiaj. Dacă `actionable` e fals, nu conține celule și nu permite manipulare."""

    location: str
    rows: int
    cols: int
    captured_at: str
    actionable: bool
    cells: Tuple[CellObservation, ...]
    reasons: Tuple[str, ...]
    geometry: GeometrySummary
    quality: FrameQualitySummary
    processing_ms: float = 0.0

    def __post_init__(self) -> None:
        if not self.location:
            raise DomainValidationError("GridObservation.location nu poate fi gol")
        if not math.isfinite(self.processing_ms) or self.processing_ms < 0:
            raise DomainValidationError("processing_ms invalid: {!r}".format(self.processing_ms))
        if self.actionable:
            expected = list(iter_coords(self.rows, self.cols))
            actual = [cell.coord for cell in self.cells]
            if actual != expected:
                raise DomainValidationError(
                    "observația {} trebuie să conțină toate celulele în ordine row-major".format(self.location)
                )
        elif self.cells:
            raise DomainValidationError("o observație neacționabilă nu poate conține celule")
        elif not self.reasons:
            raise DomainValidationError("o observație neacționabilă trebuie să aibă motive")

    def cell(self, coord: GridCoord) -> CellObservation:
        if not self.actionable:
            raise DomainValidationError("observația {} nu e acționabilă".format(self.location))
        if not coord.within(self.rows, self.cols):
            raise DomainValidationError("coordonata {} e în afara caroiajului {}x{}".format(coord, self.rows, self.cols))
        return self.cells[coord.row * self.cols + coord.col]
