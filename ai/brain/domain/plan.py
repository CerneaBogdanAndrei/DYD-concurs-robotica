"""Planul: mutări tipizate (pick + place) cu explicații, cost și durată estimată.

Planificatorul produce doar date; execuția aparține orchestrării.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple, Union

from brain.domain.cube import Color, TopFace
from brain.domain.errors import DomainValidationError
from brain.domain.grid import GridCoord


@dataclass(frozen=True, order=True)
class SlotRef:
    """Un loc manipulabil: celula unui caroiaj (`cell` dat) sau punctul unei zone (`cell` = None)."""

    location: str
    cell: Optional[GridCoord] = None

    def __post_init__(self) -> None:
        if not self.location:
            raise DomainValidationError("SlotRef.location nu poate fi gol")

    @property
    def is_zone(self) -> bool:
        return self.cell is None

    def __str__(self) -> str:
        return self.location if self.cell is None else "{}[{},{}]".format(self.location, self.cell.row, self.cell.col)


@dataclass(frozen=True)
class CubeRef:
    """Cubul vizat, așa cum era în instantaneul folosit la planificare."""

    slot: SlotRef
    level: int
    color: Color
    top_face: TopFace = TopFace.UNKNOWN


@dataclass(frozen=True)
class ObserveAction:
    location: str


@dataclass(frozen=True)
class PickAction:
    """`yaw_deg` în convenția sistemului (antiorar pozitiv); `uv` = centrul bazei cubului, corectat de paralaxă."""

    slot: SlotRef
    level: int
    yaw_deg: float = 0.0
    uv: Optional[Tuple[float, float]] = None

    def __post_init__(self) -> None:
        _check_level(self.level)
        if not math.isfinite(self.yaw_deg):
            raise DomainValidationError("yaw_deg nefinit")
        if self.uv is not None and (self.slot.is_zone or not all(math.isfinite(v) for v in self.uv)):
            raise DomainValidationError("uv e permis doar pentru celule, cu valori finite")


@dataclass(frozen=True)
class PlaceAction:
    slot: SlotRef
    level: int
    yaw_deg: float = 0.0

    def __post_init__(self) -> None:
        _check_level(self.level)
        if not math.isfinite(self.yaw_deg):
            raise DomainValidationError("yaw_deg nefinit")


@dataclass(frozen=True)
class VerifyAction:
    slot: SlotRef
    level: int
    expected_color: Color


@dataclass(frozen=True)
class RecoveryAction:
    kind: str
    detail: str


Action = Union[ObserveAction, PickAction, PlaceAction, VerifyAction, RecoveryAction]

PURPOSES = ("goal", "clear", "sort", "staging")


@dataclass(frozen=True)
class Move:
    cube: CubeRef
    pick: PickAction
    place: PlaceAction
    purpose: str
    reason: str
    cost_deg: float
    cost_is_substitute: bool
    estimated_s: float

    def __post_init__(self) -> None:
        if self.purpose not in PURPOSES:
            raise DomainValidationError("scop necunoscut: {}".format(self.purpose))
        if self.pick.slot == self.place.slot:
            raise DomainValidationError("o mutare nu poate avea sursa egală cu destinația: {}".format(self.pick.slot))

    def actions(self) -> List[Action]:
        return [self.pick, self.place, VerifyAction(self.place.slot, self.place.level, self.cube.color)]


@dataclass(frozen=True)
class Plan:
    moves: Tuple[Move, ...]
    complete: bool  # după mutări, toate obiectivele sunt îndeplinite
    unmet: Tuple[str, ...] = ()
    warnings: Tuple[str, ...] = ()
    estimated_s: float = 0.0

    @property
    def already_done(self) -> bool:
        return self.complete and not self.moves

    def explanation(self) -> List[str]:
        lines = []
        for index, move in enumerate(self.moves, 1):
            lines.append(
                "{:>2}. {} {} (niv. {}) -> {} (niv. {}): {}{}".format(
                    index, move.cube.color.value, move.pick.slot, move.pick.level, move.place.slot, move.place.level,
                    move.reason, " [cost substitut]" if move.cost_is_substitute else "",
                )
            )
        lines.extend("NEÎNDEPLINIT: {}".format(text) for text in self.unmet)
        lines.extend("ATENȚIE: {}".format(text) for text in self.warnings)
        return lines


def _check_level(level: int) -> None:
    if isinstance(level, bool) or not isinstance(level, int) or not 0 <= level <= 8:
        raise DomainValidationError("nivel invalid: {!r} (0..8)".format(level))


def correct_parallax(
    uv_top: Tuple[float, float], top_height_mm: float, camera_height_mm: float, nadir_uv: Tuple[float, float]
) -> Tuple[float, float]:
    """Proiectează centrul feței de sus (văzut la înălțimea `top_height_mm`) pe masă.

    Vederea redresată e pe planul mesei; un punct aflat la înălțimea h apare deplasat radial față de
    punctul de sub cameră (nadir) cu factorul H / (H - h). Inversul: uv = nadir + (uv_top - nadir)·(H - h)/H.
    """
    if camera_height_mm <= top_height_mm:
        raise DomainValidationError("camera trebuie să fie deasupra feței cubului")
    k = (camera_height_mm - top_height_mm) / camera_height_mm
    return (nadir_uv[0] + (uv_top[0] - nadir_uv[0]) * k, nadir_uv[1] + (uv_top[1] - nadir_uv[1]) * k)
