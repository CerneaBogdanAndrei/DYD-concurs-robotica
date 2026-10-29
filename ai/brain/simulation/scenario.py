"""Scenarii de simulare pentru sarcinile S1/S2/S3: lumea inițială, obiectivul și verificarea finală.

Lumea are cele trei șabloane din configurație (small/medium/large) și, pentru S3, zonele hărții.
Verificarea finală (`check_goal`) se face pe adevărul simulat (`SimWorld`), nu pe ce crede brain-ul.
`oracle_observation` dă observația perfectă a unei locații (fără randare și fără viziune): e folosită
în testele rapide; `--perception render` folosește camera sintetică și pipeline-ul real de viziune.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from brain.configuration.models import SortMapConfig, SystemConfig
from brain.domain.cube import KNOWN_COLORS, Color, Occupancy, TopFace, normalize_yaw_deg
from brain.domain.grid import GridCoord, iter_coords
from brain.domain.observation import CellObservation, FrameQualitySummary, GeometrySummary, GridObservation
from brain.domain.plan import SlotRef
from brain.domain.scene_state import CubeState, SceneState, ZoneState, location_from_observation, utc_now_iso
from brain.planning.goals import CellTarget, Goal, ReproduceGoal, SortGoal, TowerGoal, color_name
from brain.simulation.render import NeighborGrid, make_camera, render_location
from brain.simulation.sim_world import SimCube, SimGrid, SimPhysics, SimWorld, SimZone

COLOR_NAMES = tuple(c.value for c in KNOWN_COLORS)
ORACLE_COLOR_CONFIDENCE = 0.95
ORACLE_YAW_CONFIDENCE = 0.9

# Exemplele din regulament (§4 din prompt): S1 = aranjamentul 3x3, S2 = turnul de 5.
EXAMPLE_S1_ROWS = (("yellow", "green", "blue"), ("red", "yellow", "green"), ("blue", "red", "yellow"))
EXAMPLE_S2_ORDER = ("red", "yellow", "green", "blue", "red")


@dataclass
class Scenario:
    name: str
    seed: int
    goal: Goal
    world: SimWorld
    note: str = ""
    expect_success: Optional[bool] = None  # None = nu se știe dinainte (scenariu aleator)
    tags: Tuple[str, ...] = field(default_factory=tuple)
    # Zonele/punctele tampon nu se văd cu camera: conținutul lor inițial e dat brain-ului ca memorie.
    known_points: bool = True

    def initial_memory(self, system: SystemConfig) -> Optional[SceneState]:
        """Memoria de pornire: doar zonele și punctele tampon (caroiajele se observă)."""
        if not self.known_points or not self.world.zones:
            return None
        state = SceneState.new("memorie-{}".format(self.name))
        for name, zone in self.world.zones.items():
            state = state.with_zone(ZoneState(name, tuple(_cube_state(c) for c in zone.stack), source="given"))
        return state


# ---------------------------------------------------------------------- lumea
def build_world(system: SystemConfig, sort_map: SortMapConfig, seed: int, zones: bool = False,
                physics: Optional[SimPhysics] = None) -> SimWorld:
    """Șabloanele active, punctele tampon și (cu `zones`) zonele hărții S3."""
    grids = [SimGrid(name, system.location(name).rows, system.location(name).cols, system.location(name).cell_mm)
             for name in system.planning.active_locations]
    points = [SimZone(name) for name in system.planning.buffer_points]
    if zones:
        points += [SimZone(z.name) for z in sort_map.zones]
    return SimWorld(grids, points, system.cube.size_mm, physics, seed)


def max_offset_mm(cell_mm: float, cube_mm: float) -> float:
    """Cât se poate abate cubul de la centrul celulei fără să atingă liniile (cu 1 mm rezervă)."""
    return max(0.0, (cell_mm - cube_mm) / 2.0 - 1.0)


def put_cube(world: SimWorld, rng: np.random.Generator, location: str, cell: Tuple[int, int], color: str,
             face: Optional[str] = None) -> SimCube:
    grid = world.grids[location]
    limit = max_offset_mm(grid.cell_mm, world.cube_mm)
    stack = grid.stack(cell)
    offset = (0.0, 0.0) if stack else tuple(float(v) for v in rng.uniform(-limit, limit, size=2))
    if stack:  # pe o stivă: aproape centrat pe cubul de dedesubt
        below = stack[-1].offset_mm
        offset = (below[0] + float(rng.uniform(-1.5, 1.5)), below[1] + float(rng.uniform(-1.5, 1.5)))
    cube = world.new_cube(color, face or ("full" if rng.random() < 0.6 else "icon"), float(rng.uniform(-12.0, 12.0)),
                          offset, int(rng.integers(0, 4)))
    stack.append(cube)
    return cube


def free_cells(world: SimWorld, exclude: Sequence[SlotRef] = ()) -> List[Tuple[str, Tuple[int, int]]]:
    skip = {(s.location, s.cell.as_tuple()) for s in exclude if s.cell is not None}
    return [(name, cell) for name, grid in world.grids.items() for cell in grid.free_cells() if (name, cell) not in skip]


# ---------------------------------------------------------------------- percepția „oracol”
def oracle_observation(world: SimWorld, system: SystemConfig, location: str, captured_at: Optional[str] = None) -> GridObservation:
    """Ce ar vedea o cameră perfectă din poza de observare: doar vârful stivelor, cu paralaxă pe `uv_top`."""
    loc = system.location(location)
    grid = world.grids[location]
    cells = []
    for coord in iter_coords(loc.rows, loc.cols):
        stack = grid.stacks.get(coord.as_tuple()) or []
        if not stack:
            cells.append(CellObservation(coord, Occupancy.EMPTY, 0.98))
            continue
        top = stack[-1]
        known = top.color in COLOR_NAMES
        uv = (coord.col + 0.5 + top.offset_mm[0] / grid.cell_mm, coord.row + 0.5 + top.offset_mm[1] / grid.cell_mm)
        height = len(stack) * world.cube_mm
        k = loc.camera_height_mm / (loc.camera_height_mm - height)
        uv_top = (loc.nadir_uv[0] + (uv[0] - loc.nadir_uv[0]) * k, loc.nadir_uv[1] + (uv[1] - loc.nadir_uv[1]) * k)
        cells.append(CellObservation(
            coord, Occupancy.OCCUPIED, 0.98, metrics={"size_ratio": round(k, 4)},
            color=Color(top.color) if known else Color.UNKNOWN,
            color_confidence=ORACLE_COLOR_CONFIDENCE if known else 0.0,
            yaw_deg=normalize_yaw_deg(top.yaw_deg), yaw_confidence=ORACLE_YAW_CONFIDENCE,
            top_face=TopFace(top.face) if top.face in ("full", "icon") else TopFace.UNKNOWN,
            uv_top=(round(uv_top[0], 4), round(uv_top[1], 4)),
        ))
    return GridObservation(
        location=location, rows=loc.rows, cols=loc.cols, captured_at=captured_at or utc_now_iso(), actionable=True,
        cells=tuple(cells), reasons=(), geometry=GeometrySummary("oracle", True), quality=FrameQualitySummary(1, 1),
    )


class OraclePerception:
    """Percepție perfectă (adevărul simulat): același contract ca `CameraPerception` din orchestrare."""

    def __init__(self, world: SimWorld, system: SystemConfig) -> None:
        self.world = world
        self.system = system

    def perceive(self, location: str):
        from brain.orchestration.observer import PerceptionResult

        return PerceptionResult(oracle_observation(self.world, self.system, location))


def scene_from_world(world: SimWorld, system: SystemConfig, scene_id: str = "sim-truth") -> SceneState:
    """Adevărul complet (cu nivelurile ascunse) ca `SceneState`: folosit în teste și la verificare."""
    state = SceneState.new(scene_id)
    for name in world.grids:
        location = location_from_observation(oracle_observation(world, system, name), source="simulation")
        cells = []
        for cell in location.cells:
            stack = world.grids[name].stacks.get(cell.coord.as_tuple()) or []
            if len(stack) > 1:
                below = tuple(_cube_state(c) for c in stack[:-1])
                cell = type(cell)(cell.coord, cell.occupancy, cell.occupancy_confidence, below + cell.stack, cell.reasons)
            cells.append(cell)
        state = state.with_location(type(location)(location.name, location.rows, location.cols, tuple(cells),
                                                   location.observed_at, location.source))
    for name, zone in world.zones.items():
        state = state.with_zone(ZoneState(name, tuple(_cube_state(c) for c in zone.stack), source="simulation"))
    return state


def _cube_state(cube: SimCube) -> CubeState:
    known = cube.color in COLOR_NAMES
    return CubeState(Color(cube.color) if known else Color.UNKNOWN, ORACLE_COLOR_CONFIDENCE if known else 0.0,
                     top_face=TopFace(cube.face) if cube.face in ("full", "icon") else TopFace.UNKNOWN)


# ---------------------------------------------------------------------- camera sintetică
def make_renderer(world: SimWorld, system: SystemConfig, seed: int) -> Callable[[str, int], List[np.ndarray]]:
    """`render(location, count)`: cadrele camerei sintetice din poza de observare a locației (cu șabloanele
    vecine parțial în cadru). Camera fiecărei poze e fixă pe toată rularea."""
    cameras = {}
    neighbors: Dict[str, List[NeighborGrid]] = {}
    names = list(world.grids)
    rng = np.random.default_rng(seed)
    for index, name in enumerate(names):
        loc = system.location(name)
        geometry = system.grid_geometry(name)
        cameras[name] = make_camera(seed * 31 + index, geometry, loc.rows, loc.cols, loc.camera_height_mm, loc.nadir_uv)
        others = [system.location(n) for n in names if n != name][:2]
        gap = float(rng.uniform(12.0, 25.0))
        items = []
        if others:
            left = others[0]
            items.append(NeighborGrid((-gap - left.cols * left.cell_mm, float(rng.uniform(-20, 20))), left.rows, left.cols,
                                      left.cell_mm, left.line_width_mm))
        if len(others) > 1:
            right = others[1]
            items.append(NeighborGrid((loc.cols * loc.cell_mm + gap, float(rng.uniform(-20, 20))), right.rows, right.cols,
                                      right.cell_mm, right.line_width_mm))
        neighbors[name] = items
    frame_rng = np.random.default_rng(seed + 7)

    def render(location: str, count: int) -> List[np.ndarray]:
        return render_location(world.grids[location], system.grid_geometry(location), cameras[location], count, frame_rng,
                               neighbors=neighbors[location])

    return render


# ---------------------------------------------------------------------- verificarea finală
def check_goal(world: SimWorld, goal: Goal, sort_map: SortMapConfig) -> Tuple[bool, List[str]]:
    """(reușit, probleme) pe adevărul simulat."""
    problems: List[str] = []
    if isinstance(goal, TowerGoal):
        cell = goal.target.cell
        stack = world.grids[goal.target.location].stacks.get(cell.as_tuple() if cell else (0, 0)) or []
        got = [c.color for c in stack]
        want = [c.value for c in goal.order]
        if got != want:
            problems.append("turnul de pe {} e {} (cerut {})".format(goal.target, "-".join(got) or "gol", "-".join(want)))
    elif isinstance(goal, ReproduceGoal):
        grid = world.grids[goal.location]
        for coord, target in sorted(goal.cells.items()):
            stack = grid.stacks.get(coord.as_tuple()) or []
            if target.top is None:
                if stack:
                    problems.append("{}{} trebuia goală, are {}".format(goal.location, coord, "-".join(c.color for c in stack)))
            elif len(stack) != 1 or stack[0].color != target.top.value:
                problems.append("{}{}: {} (cerut {})".format(goal.location, coord, "-".join(c.color for c in stack) or "gol",
                                                             target.top.value))
    elif isinstance(goal, SortGoal):
        for name, grid in world.grids.items():
            for cell, stack in grid.stacks.items():
                for cube in stack:
                    if cube.color in COLOR_NAMES and sort_map.zone_for(Color(cube.color)) is not None:
                        problems.append("{} rămas pe {}{}".format(cube.color, name, cell))
        sort_zones = {z.name: [c.value for c in z.accepts] for z in sort_map.zones}
        for name, zone in world.zones.items():
            for cube in zone.stack:
                if name not in sort_zones:
                    if cube.color in COLOR_NAMES:
                        problems.append("{} rămas pe punctul tampon {}".format(cube.color, name))
                elif cube.color not in sort_zones[name]:
                    problems.append("{} pe zona greșită {}".format(cube.color, name))
        for cube in world.lost:
            if cube.color in COLOR_NAMES:
                problems.append("{} a căzut de pe masă".format(cube.color))
    return not problems, problems


# ---------------------------------------------------------------------- scenarii fixe (exemplele din regulament)
def example_s2(system: SystemConfig, sort_map: SortMapConfig, seed: int = 1, target: str = "medium:r1c1") -> Scenario:
    """Turnul din exemplu (de jos în sus roșu, galben, verde, albastru, roșu), cuburile împrăștiate pe șabloane."""
    from brain.planning.goals import parse_order, parse_slot

    rng = np.random.default_rng(seed)
    world = build_world(system, sort_map, seed)
    slot = parse_slot(target)
    cells = free_cells(world, [slot])
    picks = rng.choice(len(cells), size=len(EXAMPLE_S2_ORDER) + 2, replace=False)
    for color, index in zip(list(EXAMPLE_S2_ORDER) + ["green", "blue"], picks):
        name, cell = cells[int(index)]
        put_cube(world, rng, name, cell, color)
    return Scenario("s2_exemplu", seed, TowerGoal(parse_order(",".join(EXAMPLE_S2_ORDER)), slot), world,
                    "exemplul S2 din regulament", True, ("tower", "example"))


def example_s1(system: SystemConfig, sort_map: SortMapConfig, seed: int = 1, location: Optional[str] = None) -> Scenario:
    """Aranjamentul din exemplu (fața de jos = fața de sus). Cele 9 cuburi sunt amestecate pe același caroiaj:
    rearanjarea cere cel puțin un loc liber în afara lui (`planning.buffer_points`)."""
    rng = np.random.default_rng(seed)
    world = build_world(system, sort_map, seed)
    location = location or system.planning.active_locations[0]
    cells_goal: Dict[GridCoord, CellTarget] = {}
    colors = []
    for r, row in enumerate(EXAMPLE_S1_ROWS):
        for c, name in enumerate(row):
            cells_goal[GridCoord(r, c)] = CellTarget(Color(name), Color(name))
            colors.append(name)
    others = [(n, cell) for n, cell in free_cells(world) if n != location]
    if len(others) >= len(colors):  # mai multe șabloane active: cuburile pornesc de pe celelalte
        picks = rng.choice(len(others), size=len(colors), replace=False)
        for color, index in zip(colors, picks):
            name, cell = others[int(index)]
            put_cube(world, rng, name, cell, color)
    else:  # un singur caroiaj: o permutare a aranjamentului, cu cel puțin o celulă greșită
        order = list(rng.permutation(len(colors)))
        if all(colors[i] == colors[j] for i, j in enumerate(order)):
            order = order[1:] + order[:1]
        for index, coord in enumerate(sorted(cells_goal)):
            put_cube(world, rng, location, coord.as_tuple(), colors[int(order[index])])
    feasible = bool(others) or bool(system.planning.buffer_points)
    return Scenario("s1_exemplu", seed, ReproduceGoal(location, cells_goal), world, "exemplul S1 din regulament",
                    feasible, ("reproduce", "example"))


def tower_scenario(system: SystemConfig, sort_map: SortMapConfig, goal: TowerGoal, seed: int, extras: int = 2) -> Scenario:
    """Cuburile cerute de turn (plus câteva în plus) împrăștiate pe celulele libere, în afara celulei țintă."""
    rng = np.random.default_rng(seed)
    world = build_world(system, sort_map, seed)
    if goal.target.location not in world.grids:
        raise ValueError("ținta {} nu e pe un șablon activ ({})".format(goal.target, ", ".join(world.grids)))
    cells = free_cells(world, [goal.target])
    pool = [c.value for c in goal.order] + [COLOR_NAMES[int(rng.integers(0, 4))] for _ in range(extras)]
    pool = pool[: len(cells)]
    for color, index in zip(pool, rng.choice(len(cells), size=len(pool), replace=False)):
        name, cell = cells[int(index)]
        put_cube(world, rng, name, cell, color)
    feasible = len(goal.order) <= system.planning.max_tower_level + 1 and len(goal.order) <= len(cells)
    return Scenario("tower_{}".format(seed), seed, goal, world, goal.describe(), feasible, ("tower",))


def reproduce_scenario(system: SystemConfig, sort_map: SortMapConfig, goal: ReproduceGoal, seed: int) -> Scenario:
    """Cuburile cerute de aranjament, amestecate pe celulele șabloanelor active."""
    rng = np.random.default_rng(seed)
    world = build_world(system, sort_map, seed)
    if goal.location not in world.grids:
        raise ValueError("locația țintă {} nu e activă ({})".format(goal.location, ", ".join(world.grids)))
    colors = [t.top.value for t in goal.cells.values() if t.top is not None]
    cells = free_cells(world)
    if len(colors) > len(cells):
        raise ValueError("aranjamentul cere {} cuburi, dar sunt doar {} celule".format(len(colors), len(cells)))
    for color, index in zip(colors, rng.choice(len(cells), size=len(colors), replace=False)):
        name, cell = cells[int(index)]
        put_cube(world, rng, name, cell, color)
    return Scenario("reproduce_{}".format(seed), seed, goal, world, goal.describe(), None, ("reproduce",))


# ---------------------------------------------------------------------- scenarii aleatoare
def random_scenario(task: str, system: SystemConfig, sort_map: SortMapConfig, seed: int) -> Scenario:
    rng = np.random.default_rng(seed)
    if task == "tower":
        return _random_tower(system, sort_map, rng, seed)
    if task == "reproduce":
        return _random_reproduce(system, sort_map, rng, seed)
    if task == "sort":
        return _random_sort(system, sort_map, rng, seed)
    raise ValueError("sarcină necunoscută: {}".format(task))


def _random_slot(world: SimWorld, rng: np.random.Generator) -> SlotRef:
    name = list(world.grids)[int(rng.integers(0, len(world.grids)))]
    grid = world.grids[name]
    return SlotRef(name, GridCoord(int(rng.integers(0, grid.rows)), int(rng.integers(0, grid.cols))))


def _random_tower(system: SystemConfig, sort_map: SortMapConfig, rng: np.random.Generator, seed: int) -> Scenario:
    world = build_world(system, sort_map, seed)
    limit = system.planning.max_tower_level + 1
    length = int(rng.integers(1, limit + 1))
    order = [COLOR_NAMES[int(rng.integers(0, 4))] for _ in range(length)]
    target = _random_slot(world, rng)
    assert target.cell is not None
    tags = ["tower"]
    pool = list(order)
    # Uneori celula țintă are deja baza corectă sau un cub greșit (trebuie mutat).
    roll = rng.random()
    if roll < 0.2:
        put_cube(world, rng, target.location, target.cell.as_tuple(), order[0])
        pool = pool[1:]
        tags.append("base_ok")
    elif roll < 0.35:
        wrong = [c for c in COLOR_NAMES if c != order[0]]
        put_cube(world, rng, target.location, target.cell.as_tuple(), wrong[int(rng.integers(0, len(wrong)))])
        tags.append("target_blocked")
    missing = rng.random() < 0.1 and pool
    if missing:
        pool = pool[:-1]
        tags.append("missing_cube")
    pool += [COLOR_NAMES[int(rng.integers(0, 4))] for _ in range(int(rng.integers(0, 4)))]
    cells = free_cells(world, [target])
    picks = rng.choice(len(cells), size=len(pool), replace=False)
    for color, index in zip(pool, picks):
        name, cell = cells[int(index)]
        put_cube(world, rng, name, cell, color)
    if rng.random() < 0.3:  # un cub peste unul dintre cuburile necesare (de dezgropat)
        occupied = [(n, c) for n, g in world.grids.items() for c, s in g.stacks.items() if s and (n, c) != (
            target.location, target.cell.as_tuple())]
        if occupied:
            name, cell = occupied[int(rng.integers(0, len(occupied)))]
            put_cube(world, rng, name, cell, COLOR_NAMES[int(rng.integers(0, 4))])
            tags.append("buried")
    goal = TowerGoal(tuple(Color(c) for c in order), target)
    return Scenario("tower_{}".format(seed), seed, goal, world, goal.describe(), None if missing else True, tuple(tags))


def _random_reproduce(system: SystemConfig, sort_map: SortMapConfig, rng: np.random.Generator, seed: int) -> Scenario:
    world = build_world(system, sort_map, seed)
    names = list(world.grids)
    location = names[int(rng.integers(0, len(names)))]
    loc = system.location(location)
    complete = rng.random() < 0.5
    cells_goal: Dict[GridCoord, CellTarget] = {}
    for coord in iter_coords(loc.rows, loc.cols):
        roll = rng.random()
        if complete or roll < 0.55:
            cells_goal[coord] = CellTarget(Color(COLOR_NAMES[int(rng.integers(0, 4))]))
        elif roll < 0.8:
            cells_goal[coord] = CellTarget(None)
    needed = [t.top.value for t in cells_goal.values() if t.top is not None]
    pool = needed + [COLOR_NAMES[int(rng.integers(0, 4))] for _ in range(int(rng.integers(0, 3)))]
    cells = free_cells(world)
    pool = pool[: len(cells)]
    picks = rng.choice(len(cells), size=len(pool), replace=False)
    for color, index in zip(pool, picks):
        name, cell = cells[int(index)]
        put_cube(world, rng, name, cell, color)
    goal = ReproduceGoal(location, cells_goal)
    tags = ("reproduce", "complete" if complete else "partial")
    # Fără puncte tampon și fără alt șablon, o permutare completă nu se poate rearanja.
    feasible = not (complete and len(world.grids) == 1 and not system.planning.buffer_points)
    return Scenario("reproduce_{}".format(seed), seed, goal, world, goal.describe(), True if feasible else None, tags)


def _random_sort(system: SystemConfig, sort_map: SortMapConfig, rng: np.random.Generator, seed: int) -> Scenario:
    world = build_world(system, sort_map, seed, zones=True)
    cells = free_cells(world)
    count = int(rng.integers(3, min(10, len(cells) + 1)))
    picks = rng.choice(len(cells), size=count, replace=False)
    tags = ["sort"]
    for index in picks:
        name, cell = cells[int(index)]
        put_cube(world, rng, name, cell, COLOR_NAMES[int(rng.integers(0, 4))])
        if rng.random() < 0.2:
            put_cube(world, rng, name, cell, COLOR_NAMES[int(rng.integers(0, 4))])
            tags.append("stacked")
    if rng.random() < 0.3:  # cub deja sortat
        zone = sort_map.zones[int(rng.integers(0, len(sort_map.zones)))]
        world.zones[zone.name].stack.append(world.new_cube(zone.accepts[0].value, "full", 0.0))
        tags.append("already_sorted")
    goal = SortGoal()
    capacity = sort_map.max_level + 1
    colors = [c.color for g in world.grids.values() for st in g.stacks.values() for c in st]
    colors += [c.color for z in world.zones.values() for c in z.stack]
    feasible = all(colors.count(name) <= capacity for name in COLOR_NAMES)
    if not feasible:
        tags.append("zone_overflow")
    return Scenario("sort_{}".format(seed), seed, goal, world, "{} cuburi de sortat".format(count), True if feasible else None,
                    tuple(sorted(set(tags))))


def describe_world(world: SimWorld) -> List[str]:
    """Rezumat text al lumii (pentru rapoarte)."""
    lines = []
    for name, grid in world.grids.items():
        cells = ["{}:{}".format("r{}c{}".format(*cell), "-".join(c.color for c in stack))
                 for cell, stack in sorted(grid.stacks.items()) if stack]
        lines.append("{}: {}".format(name, ", ".join(cells) or "gol"))
    for name, zone in world.zones.items():
        lines.append("{}: {}".format(name, "-".join(c.color for c in zone.stack) or "gol"))
    if world.lost:
        lines.append("căzute de pe masă: {}".format(", ".join(c.color for c in world.lost)))
    return lines


__all__ = [
    "Scenario", "build_world", "put_cube", "oracle_observation", "OraclePerception", "scene_from_world", "make_renderer", "check_goal",
    "example_s1", "example_s2", "random_scenario", "describe_world", "color_name",
]
