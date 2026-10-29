"""Lumea fizică simulată: șabloane 3x3, zonele hărții, stive de cuburi, gheara.

Regulile sunt simple, dar fac rezultatul să depindă de calitatea comenzilor (ca pe robot):
- prinderea reușește doar dacă poziția comandată (uv sau centrul celulei) e la cel mult
  `grasp_tolerance_mm` de centrul real al cubului și unghiul ghearei e aliniat cu fețele (mod 90°);
- nivelul comandat trebuie să fie vârful stivei: mai sus = gheara se închide în gol, mai jos = coliziune;
- un cub așezat descentrat pe o stivă cade; un cub căzut ajunge într-o celulă liberă sau în afara mesei.
Simularea NU modelează cinematica, coliziunile reale sau dinamica: validează doar logica.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Set, Tuple

import numpy as np

from brain.domain.cube import normalize_yaw_deg, yaw_difference_deg

GRASPED = "grasped"
MISSED = "missed"
COLLISION = "collision"
UNREACHABLE = "unreachable"
PLACED = "placed"
DROPPED = "dropped"
NOTHING_HELD = "nothing_held"


@dataclass
class SimCube:
    uid: int
    color: str  # red / yellow / green / blue sau o culoare ambiguă (orange, desaturated)
    face: str  # full / icon
    yaw_deg: float  # față de axele caroiajului, antiorar pozitiv
    offset_mm: Tuple[float, float] = (0.0, 0.0)  # centrul bazei față de centrul celulei (x = coloane, y = linii)
    icon_shape: int = 0


@dataclass
class SimGrid:
    name: str
    rows: int
    cols: int
    cell_mm: float
    stacks: Dict[Tuple[int, int], List[SimCube]] = field(default_factory=dict)
    unreachable: Set[Tuple[int, int]] = field(default_factory=set)

    def stack(self, cell: Tuple[int, int]) -> List[SimCube]:
        return self.stacks.setdefault(cell, [])

    def free_cells(self) -> List[Tuple[int, int]]:
        return [(r, c) for r in range(self.rows) for c in range(self.cols) if not self.stacks.get((r, c))]


@dataclass
class SimZone:
    name: str
    stack: List[SimCube] = field(default_factory=list)
    unreachable: bool = False


@dataclass(frozen=True)
class SimPhysics:
    grasp_tolerance_mm: float = 7.0
    grasp_yaw_tolerance_deg: float = 20.0
    place_noise_mm: float = 1.2
    place_yaw_noise_deg: float = 1.5
    stack_support_mm: float = 10.0
    lost_probability: float = 0.2  # un cub căzut iese de pe masă
    pick_fail_probability: float = 0.0
    drop_probability: float = 0.0


@dataclass
class Hover:
    location: str
    cell: Optional[Tuple[int, int]]
    level: int


class SimWorld:
    def __init__(
        self,
        grids: Iterable[SimGrid],
        zones: Iterable[SimZone],
        cube_mm: float,
        physics: Optional[SimPhysics] = None,
        seed: int = 0,
    ) -> None:
        self.grids: Dict[str, SimGrid] = {g.name: g for g in grids}
        self.zones: Dict[str, SimZone] = {z.name: z for z in zones}
        self.cube_mm = float(cube_mm)
        self.physics = physics or SimPhysics()
        self.rng = np.random.default_rng(seed)
        self.held: Optional[SimCube] = None
        self._held_error_mm = (0.0, 0.0)
        self._held_yaw_rel = 0.0
        self.hover: Optional[Hover] = None
        self.lost: List[SimCube] = []
        self.events: List[str] = []
        self._next_uid = 1 + max([c.uid for c in self.all_cubes()] + [0])

    # ------------------------------------------------------------------ interogări
    def all_cubes(self) -> List[SimCube]:
        cubes = [c for g in self.grids.values() for s in g.stacks.values() for c in s]
        cubes += [c for z in self.zones.values() for c in z.stack]
        if getattr(self, "held", None) is not None:
            cubes.append(self.held)  # type: ignore[arg-type]
        return cubes

    def knows(self, location: str) -> bool:
        return location in self.grids or location in self.zones

    def stack_at(self, location: str, cell: Optional[Tuple[int, int]]) -> List[SimCube]:
        if location in self.zones:
            return self.zones[location].stack
        return self.grids[location].stack(cell)  # type: ignore[arg-type]

    def is_unreachable(self, location: str, cell: Optional[Tuple[int, int]]) -> bool:
        if location in self.zones:
            return self.zones[location].unreachable
        return cell in self.grids[location].unreachable

    def new_cube(self, color: str, face: str, yaw_deg: float, offset_mm=(0.0, 0.0), icon_shape: int = 0) -> SimCube:
        cube = SimCube(self._next_uid, color, face, normalize_yaw_deg(yaw_deg), offset_mm, icon_shape)
        self._next_uid += 1
        return cube

    def surface_height_mm(self, location: str, cell: Optional[Tuple[int, int]]) -> float:
        return len(self.stack_at(location, cell)) * self.cube_mm

    # ------------------------------------------------------------------ fizica simplificată
    def _cell_size(self, location: str) -> float:
        return self.grids[location].cell_mm if location in self.grids else self.cube_mm * 1.5

    def pick(
        self,
        location: str,
        cell: Optional[Tuple[int, int]],
        level: int,
        yaw_deg: float,
        uv: Optional[Tuple[float, float]],
        force_miss: bool = False,
    ) -> str:
        if self.is_unreachable(location, cell):
            return UNREACHABLE
        if self.held is not None:  # serviciul deschide gheara înainte de pick: cubul ținut cade
            self.events.append("cubul {} a căzut la deschiderea ghearei".format(self.held.uid))
            self._drop(self.held, location)
            self.held = None
        self.hover = Hover(location, cell, level)
        stack = self.stack_at(location, cell)
        top = len(stack) - 1
        if level > top:
            self.events.append("pick {}{} niv. {}: gheara s-a închis în gol".format(location, cell or "", level))
            return MISSED
        if level < top:
            self.events.append("pick {}{} niv. {}: coliziune cu stiva (vârful e la {})".format(location, cell or "", level, top))
            knocked = stack.pop()
            self._drop(knocked, location)
            return COLLISION
        cube = stack[-1]
        size = self._cell_size(location)
        if cell is not None:
            commanded = uv if uv is not None else (cell[1] + 0.5, cell[0] + 0.5)
            true_uv = (cell[1] + 0.5 + cube.offset_mm[0] / size, cell[0] + 0.5 + cube.offset_mm[1] / size)
            error = ((true_uv[0] - commanded[0]) * size, (true_uv[1] - commanded[1]) * size)
        else:
            error = cube.offset_mm
        distance = math.hypot(*error)
        yaw_error = yaw_difference_deg(yaw_deg, cube.yaw_deg)
        failed = force_miss or self.rng.random() < self.physics.pick_fail_probability
        if failed or distance > self.physics.grasp_tolerance_mm or yaw_error > self.physics.grasp_yaw_tolerance_deg:
            nudge = self.rng.uniform(-2.0, 2.0, size=2)
            cube.offset_mm = (cube.offset_mm[0] + float(nudge[0]), cube.offset_mm[1] + float(nudge[1]))
            self.events.append(
                "pick {}{}: ratat (eroare {:.1f} mm, unghi {:.1f}°{})".format(
                    location, cell or "", distance, yaw_error, ", eșec injectat" if failed else "")
            )
            return MISSED
        stack.pop()
        self.held = cube
        self._held_error_mm = error
        self._held_yaw_rel = normalize_yaw_deg(cube.yaw_deg - yaw_deg)
        return GRASPED

    def drop_held_in_transit(self) -> None:
        if self.held is None or self.hover is None:
            return
        self.events.append("cubul {} a căzut în timpul transportului".format(self.held.uid))
        self._drop(self.held, self.hover.location)
        self.held = None

    def place(self, location: str, cell: Optional[Tuple[int, int]], level: int, yaw_deg: float) -> str:
        if self.is_unreachable(location, cell):
            return UNREACHABLE
        self.hover = Hover(location, cell, level)
        cube = self.held
        if cube is None:
            self.events.append("place {}{}: gheara era goală".format(location, cell or ""))
            return NOTHING_HELD
        self.held = None
        stack = self.stack_at(location, cell)
        height = len(stack)
        if level < height:
            self.events.append("place {}{} niv. {}: coliziune (stiva are {} cuburi)".format(location, cell or "", level, height))
            self._drop(cube, location)
            return COLLISION
        sigma = self.physics.place_noise_mm * (1.0 + 6.0 * (level - height))  # eliberat din aer: împrăștiere mare
        noise = self.rng.normal(0.0, sigma, size=2)
        cube.offset_mm = (self._held_error_mm[0] + float(noise[0]), self._held_error_mm[1] + float(noise[1]))
        cube.yaw_deg = normalize_yaw_deg(yaw_deg + self._held_yaw_rel + float(self.rng.normal(0.0, self.physics.place_yaw_noise_deg)))
        if height > 0:
            below = stack[-1]
            shift = math.hypot(cube.offset_mm[0] - below.offset_mm[0], cube.offset_mm[1] - below.offset_mm[1])
            if shift > self.physics.stack_support_mm:
                self.events.append("place {}{}: cubul a căzut de pe stivă (decalaj {:.1f} mm)".format(location, cell or "", shift))
                self._drop(cube, location)
                return DROPPED
        stack.append(cube)
        return PLACED

    def _drop(self, cube: SimCube, near_location: str) -> None:
        grid = self.grids.get(near_location) or (next(iter(self.grids.values())) if self.grids else None)
        free = grid.free_cells() if grid is not None else []
        if grid is None or not free or self.rng.random() < self.physics.lost_probability:
            self.lost.append(cube)
            self.events.append("cubul {} ({}) a ieșit de pe masă".format(cube.uid, cube.color))
            return
        cell = free[int(self.rng.integers(0, len(free)))]
        cube.offset_mm = tuple(float(v) for v in self.rng.uniform(-4.0, 4.0, size=2))  # type: ignore[assignment]
        cube.yaw_deg = normalize_yaw_deg(float(self.rng.uniform(-45.0, 45.0)))
        grid.stack(cell).append(cube)
        self.events.append("cubul {} ({}) a aterizat în {}{}".format(cube.uid, cube.color, grid.name, cell))
