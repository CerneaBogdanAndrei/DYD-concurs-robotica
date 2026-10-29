"""Lumea de planificare: sloturi (celule + zone) cu stive, construită din `SceneState` + configurație.

Doar cubul din vârful unei stive e manipulabil. Un cub e „de încredere” dacă are culoare cunoscută și
încrederea culorii ≥ `planning.min_confidence`; cuburile fără încredere nu sunt mutate deloc (nici ca
obstacole), deci blochează ce e sub ele. Celulele cu ocupare necunoscută nu sunt nici sursă, nici destinație.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Set, Tuple

from brain.domain.cube import KNOWN_COLORS, Color, Occupancy, TopFace
from brain.domain.grid import GridCoord, iter_coords
from brain.domain.plan import SlotRef
from brain.domain.scene_state import CubeState, SceneState


@dataclass(frozen=True)
class PlanCube:
    color: Color
    confidence: float
    top_face: TopFace = TopFace.UNKNOWN
    yaw_deg: Optional[float] = None
    uv_top: Optional[Tuple[float, float]] = None  # doar pentru cubul observat în vârf; None după o mutare planificată
    trusted: bool = True

    @staticmethod
    def from_state(cube: CubeState, min_confidence: float) -> "PlanCube":
        trusted = cube.color in KNOWN_COLORS and cube.color_confidence >= min_confidence
        return PlanCube(cube.color, cube.color_confidence, cube.top_face, cube.yaw_deg, cube.uv_top, trusted)

    def moved(self) -> "PlanCube":
        """Cubul după o mutare: poziția și yaw-ul exacte nu mai sunt cunoscute până la reobservare."""
        return PlanCube(self.color, self.confidence, self.top_face, None, None, self.trusted)


class PlanningWorld:
    def __init__(self, grid_sizes: Dict[str, Tuple[int, int]], zones: Iterable[str] = ()) -> None:
        self.grid_sizes = dict(grid_sizes)
        self.zones: Tuple[str, ...] = tuple(zones)
        self.stacks: Dict[SlotRef, List[PlanCube]] = {}
        self.blocked: Set[SlotRef] = set()  # ocupare necunoscută sau înălțime incertă
        for name, (rows, cols) in self.grid_sizes.items():
            for coord in iter_coords(rows, cols):
                self.stacks[SlotRef(name, coord)] = []
        for zone in self.zones:
            self.stacks[SlotRef(zone)] = []

    # ------------------------------------------------------------------ construire
    @staticmethod
    def from_scene(
        state: SceneState,
        grid_sizes: Dict[str, Tuple[int, int]],
        zones: Iterable[str],
        min_confidence: float,
    ) -> "PlanningWorld":
        world = PlanningWorld(grid_sizes, zones)
        for name in grid_sizes:
            location = state.locations.get(name)
            if location is None:  # locație neobservată: nimic nu se poate decide despre ea
                for coord in iter_coords(*grid_sizes[name]):
                    world.blocked.add(SlotRef(name, coord))
                continue
            for cell in location.cells:
                slot = SlotRef(name, cell.coord)
                if slot not in world.stacks:
                    continue
                if cell.occupancy is Occupancy.UNKNOWN or "height_uncertain" in cell.reasons:
                    world.blocked.add(slot)
                world.stacks[slot] = [PlanCube.from_state(c, min_confidence) for c in cell.stack]
        for zone in world.zones:
            world.stacks[SlotRef(zone)] = [PlanCube.from_state(c, min_confidence) for c in state.zone(zone).stack]
        return world

    def copy(self) -> "PlanningWorld":
        clone = PlanningWorld.__new__(PlanningWorld)
        clone.grid_sizes = dict(self.grid_sizes)
        clone.zones = self.zones
        clone.stacks = {slot: list(stack) for slot, stack in self.stacks.items()}
        clone.blocked = set(self.blocked)
        return clone

    # ------------------------------------------------------------------ interogări
    def slots(self) -> List[SlotRef]:
        return sorted(self.stacks, key=_slot_key)

    def grid_slots(self, locations: Optional[Iterable[str]] = None) -> List[SlotRef]:
        wanted = set(self.grid_sizes if locations is None else locations)
        return [s for s in self.slots() if not s.is_zone and s.location in wanted]

    def stack(self, slot: SlotRef) -> List[PlanCube]:
        return self.stacks[slot]

    def height(self, slot: SlotRef) -> int:
        return len(self.stacks[slot])

    def top(self, slot: SlotRef) -> Optional[PlanCube]:
        stack = self.stacks[slot]
        return stack[-1] if stack else None

    def is_free_cell(self, slot: SlotRef) -> bool:
        return not slot.is_zone and slot not in self.blocked and not self.stacks[slot]

    def neighbors(self, slot: SlotRef) -> List[SlotRef]:
        if slot.is_zone or slot.cell is None:
            return []
        rows, cols = self.grid_sizes[slot.location]
        result = []
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                r, c = slot.cell.row + dr, slot.cell.col + dc
                if (dr or dc) and 0 <= r < rows and 0 <= c < cols:
                    result.append(SlotRef(slot.location, GridCoord(r, c)))
        return result

    def count_trusted(self, color: Color, exclude: Iterable[SlotRef] = ()) -> int:
        skip = set(exclude)
        return sum(1 for slot, stack in self.stacks.items() if slot not in skip for c in stack if c.trusted and c.color is color)

    # ------------------------------------------------------------------ modificare
    def move_top(self, source: SlotRef, target: SlotRef) -> PlanCube:
        cube = self.stacks[source].pop()
        self.stacks[target].append(cube.moved())
        return cube


def _slot_key(slot: SlotRef) -> Tuple[int, str, int, int]:
    if slot.cell is None:
        return (1, slot.location, 0, 0)
    return (0, slot.location, slot.cell.row, slot.cell.col)


def grid_sizes_from_config(locations: Dict[str, object]) -> Dict[str, Tuple[int, int]]:
    """{nume: (rows, cols)} din `SystemConfig.locations`."""
    return {name: (int(getattr(loc, "rows")), int(getattr(loc, "cols"))) for name, loc in locations.items()}


def deepcopy_world(world: PlanningWorld) -> PlanningWorld:  # pragma: no cover - utilitar de depanare
    return copy.deepcopy(world)
