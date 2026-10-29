"""Starea lumii (`SceneState`) și diferențele dintre două stări.

Există o singură reprezentare a lumii. După fiecare operație, sistemul reobservă, compară
starea așteptată cu cea observată (`diff_scenes`) și abia apoi actualizează memoria.
"""

from __future__ import annotations

import datetime as _dt
import math
from collections import Counter
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Dict, List, Mapping, Optional, Tuple, Union

from brain.domain.cube import Color, Occupancy, TopFace, check_confidence, check_yaw, yaw_difference_deg
from brain.domain.errors import DomainValidationError
from brain.domain.grid import GridCoord, iter_coords
from brain.domain.observation import GridObservation

SCHEMA_VERSION = 2
MAX_STACK_HEIGHT = 9  # serviciul de mișcare acceptă nivelurile 0..8 (turnul S2 are 5 cuburi)
MAX_ZONE_STACK = 9  # serviciul de mișcare acceptă nivelurile 0..8

MetadataValue = Union[str, int, float, bool, None]


def utc_now_iso() -> str:
    return _dt.datetime.now(_dt.timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


@dataclass(frozen=True)
class CubeState:
    color: Color
    color_confidence: float
    yaw_deg: Optional[float] = None
    yaw_confidence: float = 0.0
    top_face: TopFace = TopFace.UNKNOWN
    cube_id: Optional[str] = None
    # Centrul feței de sus observate, în coordonate de celulă ale locației (necorectat de paralaxă).
    uv_top: Optional[Tuple[float, float]] = None

    def __post_init__(self) -> None:
        if self.uv_top is not None and (
            len(self.uv_top) != 2 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in self.uv_top)
        ):
            raise DomainValidationError("uv_top invalid: {!r}".format(self.uv_top))
        check_confidence(self.color_confidence, "color_confidence")
        check_confidence(self.yaw_confidence, "yaw_confidence")
        if self.yaw_deg is not None:
            check_yaw(self.yaw_deg)
        elif self.yaw_confidence != 0.0:
            raise DomainValidationError("yaw_confidence trebuie să fie 0 când yaw_deg lipsește")
        if self.cube_id is not None and not self.cube_id:
            raise DomainValidationError("cube_id nu poate fi șir gol")


@dataclass(frozen=True)
class CellState:
    """Starea unei celule. `stack` e de jos în sus; camera vede doar cubul din vârf."""

    coord: GridCoord
    occupancy: Occupancy
    occupancy_confidence: float
    stack: Tuple[CubeState, ...] = ()
    reasons: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        check_confidence(self.occupancy_confidence, "occupancy_confidence")
        if self.occupancy is Occupancy.OCCUPIED:
            if not 1 <= len(self.stack) <= MAX_STACK_HEIGHT:
                raise DomainValidationError(
                    "celula ocupată {} trebuie să aibă 1..{} cuburi, nu {}".format(
                        self.coord, MAX_STACK_HEIGHT, len(self.stack)
                    )
                )
        elif self.stack:
            raise DomainValidationError("celula {} ({}) nu poate avea cuburi".format(self.coord, self.occupancy.value))

    @property
    def top(self) -> Optional[CubeState]:
        return self.stack[-1] if self.stack else None


@dataclass(frozen=True)
class LocationState:
    """Starea unui caroiaj (sursă, destinație, staging etc.)."""

    name: str
    rows: int
    cols: int
    cells: Tuple[CellState, ...]
    observed_at: Optional[str] = None
    source: str = "perception"

    def __post_init__(self) -> None:
        if not self.name:
            raise DomainValidationError("LocationState.name nu poate fi gol")
        expected = list(iter_coords(self.rows, self.cols))
        if [cell.coord for cell in self.cells] != expected:
            raise DomainValidationError(
                "locația {} trebuie să conțină toate celulele {}x{} în ordine row-major".format(
                    self.name, self.rows, self.cols
                )
            )

    def cell(self, coord: GridCoord) -> CellState:
        if not coord.within(self.rows, self.cols):
            raise DomainValidationError("coordonata {} e în afara locației {}".format(coord, self.name))
        return self.cells[coord.row * self.cols + coord.col]

    def with_cell(self, cell: CellState) -> "LocationState":
        self.cell(cell.coord)
        index = cell.coord.row * self.cols + cell.coord.col
        cells =self.cells[:index] + (cell,) + self.cells[index + 1:]
        return replace(self, cells=cells)

    @staticmethod
    def empty(name: str, rows: int, cols: int, source: str = "manual") -> "LocationState":
        cells = tuple(CellState(coord, Occupancy.EMPTY, 1.0) for coord in iter_coords(rows, cols))
        return LocationState(name=name, rows=rows, cols=cols, cells=cells, source=source)


@dataclass(frozen=True)
class ZoneState:
    """Stiva de pe punctul unei zone a hărții (Sarcina 3). Ținută din memorie și verificări."""

    name: str
    stack: Tuple[CubeState, ...] = ()
    source: str = "memory"

    def __post_init__(self) -> None:
        if not self.name:
            raise DomainValidationError("ZoneState.name nu poate fi gol")
        if len(self.stack) > MAX_ZONE_STACK:
            raise DomainValidationError("zona {} are prea multe cuburi: {}".format(self.name, len(self.stack)))


@dataclass(frozen=True)
class SceneState:
    scene_id: str
    created_at: str
    updated_at: str
    locations: Mapping[str, LocationState] = field(default_factory=dict)
    metadata: Mapping[str, MetadataValue] = field(default_factory=dict)
    schema_version: int = SCHEMA_VERSION
    zones: Mapping[str, ZoneState] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.scene_id:
            raise DomainValidationError("scene_id nu poate fi gol")
        if self.schema_version != SCHEMA_VERSION:
            raise DomainValidationError(
                "schema_version {} diferă de versiunea curentă {}".format(self.schema_version, SCHEMA_VERSION)
            )
        for key, location in self.locations.items():
            if key != location.name:
                raise DomainValidationError("cheia '{}' nu corespunde locației '{}'".format(key, location.name))
        for key, zone in self.zones.items():
            if key != zone.name:
                raise DomainValidationError("cheia '{}' nu corespunde zonei '{}'".format(key, zone.name))
            if key in self.locations:
                raise DomainValidationError("'{}' nu poate fi și locație, și zonă".format(key))
        for key, value in self.metadata.items():
            if not isinstance(key, str) or not isinstance(value, (str, int, float, bool, type(None))):
                raise DomainValidationError("metadata invalidă: {!r}: {!r}".format(key, value))

    @staticmethod
    def new(scene_id: str, metadata: Optional[Mapping[str, MetadataValue]] = None) -> "SceneState":
        now = utc_now_iso()
        return SceneState(scene_id=scene_id, created_at=now, updated_at=now, metadata=dict(metadata or {}))

    def location(self, name: str) -> LocationState:
        try:
            return self.locations[name]
        except KeyError:
            raise DomainValidationError("locația '{}' nu există în scena {}".format(name, self.scene_id)) from None

    def with_location(self, location: LocationState, updated_at: Optional[str] = None) -> "SceneState":
        locations = dict(self.locations)
        locations[location.name] = location
        return replace(self, locations=locations, updated_at=updated_at or utc_now_iso())

    def with_zone(self, zone: ZoneState, updated_at: Optional[str] = None) -> "SceneState":
        zones = dict(self.zones)
        zones[zone.name] = zone
        return replace(self, zones=zones, updated_at=updated_at or utc_now_iso())

    def zone(self, name: str) -> ZoneState:
        return self.zones.get(name) or ZoneState(name)


def location_from_observation(observation: GridObservation, source: str = "perception") -> LocationState:
    """Convertește o observație acționabilă într-o stare de locație (doar cubul din vârf e vizibil)."""
    if not observation.actionable:
        raise DomainValidationError(
            "observația {} nu e acționabilă: {}".format(observation.location, ", ".join(observation.reasons))
        )
    cells = []
    for obs in observation.cells:
        stack: Tuple[CubeState, ...] = ()
        if obs.occupancy is Occupancy.OCCUPIED:
            stack = (
                CubeState(
                    color=obs.color,
                    color_confidence=obs.color_confidence,
                    yaw_deg=obs.yaw_deg,
                    yaw_confidence=obs.yaw_confidence,
                    top_face=obs.top_face,
                    uv_top=obs.uv_top,
                ),
            )
        cells.append(CellState(obs.coord, obs.occupancy, obs.occupancy_confidence, stack, obs.reasons))
    return LocationState(
        name=observation.location,
        rows=observation.rows,
        cols=observation.cols,
        cells=tuple(cells),
        observed_at=observation.captured_at,
        source=source,
    )


def assign_unique_color_ids(state: SceneState) -> SceneState:
    """Dă `cube_id` = culoarea cubului doar dacă acea culoare apare o singură dată în toată scena.

    Cuburile cu aceeași culoare sunt interschimbabile simbolic, deci nu primesc identificator.
    """
    counts: Counter = Counter()
    stacks = [cell.stack for location in state.locations.values() for cell in location.cells]
    stacks += [zone.stack for zone in state.zones.values()]
    for stack in stacks:
        for cube in stack:
            if cube.color is not Color.UNKNOWN:
                counts[cube.color] += 1
    locations: Dict[str, LocationState] = {}
    for name, location in state.locations.items():
        new_cells = []
        for cell in location.cells:
            stack = tuple(
                replace(cube, cube_id=cube.color.value if counts[cube.color] == 1 else None) for cube in cell.stack
            )
            new_cells.append(replace(cell, stack=stack))
        locations[name] = replace(location, cells=tuple(new_cells))
    zones = {
        name: replace(zone, stack=tuple(
            replace(cube, cube_id=cube.color.value if counts[cube.color] == 1 else None) for cube in zone.stack))
        for name, zone in state.zones.items()
    }
    return replace(state, locations=locations, zones=zones)


class DiffKind(str, Enum):
    MISSING_LOCATION = "missing_location"
    DIMENSIONS = "dimensions"
    OCCUPANCY = "occupancy"
    COLOR = "color"
    TOP_FACE = "top_face"
    YAW = "yaw"
    UNCERTAIN = "uncertain"


@dataclass(frozen=True)
class CellDifference:
    location: str
    coord: Optional[GridCoord]
    kind: DiffKind
    expected: str
    observed: str


def diff_locations(
    expected: LocationState,
    observed: LocationState,
    yaw_tolerance_deg: Optional[float] = None,
    compare_top_face: bool = False,
) -> List[CellDifference]:
    """Compară starea așteptată cu cea observată, celulă cu celulă (doar cubul din vârf).

    `UNCERTAIN` înseamnă că observația nu permite verificarea (necunoscut), nu că ar fi greșită.
    Yaw-ul se compară doar dacă `yaw_tolerance_deg` e dat și ambele valori sunt cunoscute.
    """
    name = expected.name
    if (expected.rows, expected.cols) != (observed.rows, observed.cols):
        return [
            CellDifference(
                name,
                None,
                DiffKind.DIMENSIONS,
                "{}x{}".format(expected.rows, expected.cols),
                "{}x{}".format(observed.rows, observed.cols),
            )
        ]
    diffs: List[CellDifference] = []
    for exp_cell, obs_cell in zip(expected.cells, observed.cells):
        coord = exp_cell.coord
        if obs_cell.occupancy is Occupancy.UNKNOWN:
            if exp_cell.occupancy is not Occupancy.UNKNOWN:
                diffs.append(CellDifference(name, coord, DiffKind.UNCERTAIN, exp_cell.occupancy.value, "unknown"))
            continue
        if exp_cell.occupancy is Occupancy.UNKNOWN:
            continue
        if exp_cell.occupancy is not obs_cell.occupancy:
            diffs.append(
                CellDifference(name, coord, DiffKind.OCCUPANCY, exp_cell.occupancy.value, obs_cell.occupancy.value)
            )
            continue
        exp_top, obs_top = exp_cell.top, obs_cell.top
        if exp_top is None or obs_top is None:
            continue
        if obs_top.color is Color.UNKNOWN:
            if exp_top.color is not Color.UNKNOWN:
                diffs.append(CellDifference(name, coord, DiffKind.UNCERTAIN, exp_top.color.value, "unknown"))
        elif exp_top.color is not Color.UNKNOWN and exp_top.color is not obs_top.color:
            diffs.append(CellDifference(name, coord, DiffKind.COLOR, exp_top.color.value, obs_top.color.value))
        if (
            compare_top_face
            and TopFace.UNKNOWN not in (exp_top.top_face, obs_top.top_face)
            and exp_top.top_face is not obs_top.top_face
        ):
            diffs.append(
                CellDifference(name, coord, DiffKind.TOP_FACE, exp_top.top_face.value, obs_top.top_face.value)
            )
        if yaw_tolerance_deg is not None and exp_top.yaw_deg is not None and obs_top.yaw_deg is not None:
            if yaw_difference_deg(exp_top.yaw_deg, obs_top.yaw_deg) > yaw_tolerance_deg:
                diffs.append(
                    CellDifference(
                        name, coord, DiffKind.YAW, "{:.1f}".format(exp_top.yaw_deg), "{:.1f}".format(obs_top.yaw_deg)
                    )
                )
    return diffs


def diff_scenes(
    expected: SceneState,
    observed: SceneState,
    yaw_tolerance_deg: Optional[float] = None,
    compare_top_face: bool = False,
) -> List[CellDifference]:
    """Diferențele pentru toate locațiile din starea așteptată."""
    diffs: List[CellDifference] = []
    for name, exp_loc in expected.locations.items():
        obs_loc = observed.locations.get(name)
        if obs_loc is None:
            diffs.append(CellDifference(name, None, DiffKind.MISSING_LOCATION, "present", "missing"))
            continue
        diffs.extend(diff_locations(exp_loc, obs_loc, yaw_tolerance_deg, compare_top_face))
    return diffs
