"""Îmbinarea observației (doar vârful stivelor) cu memoria (nivelurile de dedesubt) și efectul așteptat al mutărilor.

Camera vede doar cubul din vârf. Înălțimea stivei vine din memorie: starea așteptată după ultima mutare
(`expected`) și starea de dinaintea ei (`before`, pentru cazul în care mutarea nu a avut loc). Un vârf observat
care nu se potrivește cu niciuna e pus deasupra stivei memorate, iar celula e marcată `height_uncertain`:
planificatorul nu o mai folosește până nu e lămurită (de ex. un cub căzut peste o stivă).
"""

from __future__ import annotations

from dataclasses import replace
from typing import List, Optional, Sequence, Tuple

from brain.domain.cube import Color, Occupancy
from brain.domain.errors import DomainValidationError
from brain.domain.plan import Move, SlotRef
from brain.domain.scene_state import (
    MAX_STACK_HEIGHT,
    CellState,
    CubeState,
    LocationState,
    SceneState,
    ZoneState,
)

HEIGHT_UNCERTAIN = "height_uncertain"
HEIGHT_FROM_SIZE = "height_from_size"
PARALLAX_GHOST = "parallax_ghost"
GHOST_REASONS = ("split_blob", "ambiguous_colored_area", "off_center")
# Estimarea din mărimea aparentă a feței de sus (size_ratio = H/(H-h)) e sigură doar pentru 1 vs. 2 cuburi
# (date sintetice: 1 cub → ~1,15, 2 cuburi → ~2,13, 3 cuburi → 1,9..2,5). Pe robot depinde de camera_height_mm.
TWO_CUBES_RANGE = (1.65, 2.35)


def get_stack(state: SceneState, slot: SlotRef) -> Tuple[CubeState, ...]:
    if slot.is_zone:
        return state.zone(slot.location).stack
    assert slot.cell is not None
    return state.location(slot.location).cell(slot.cell).stack


def set_stack(state: SceneState, slot: SlotRef, stack: Sequence[CubeState]) -> SceneState:
    stack = tuple(stack)
    if slot.is_zone:
        previous = state.zone(slot.location)
        return state.with_zone(ZoneState(slot.location, stack, source=previous.source if previous.stack else "memory"))
    assert slot.cell is not None
    location = state.location(slot.location)
    cell = CellState(slot.cell, Occupancy.OCCUPIED if stack else Occupancy.EMPTY, 1.0, stack, ())
    return state.with_location(location.with_cell(cell))


def apply_move(state: SceneState, move: Move) -> SceneState:
    """Starea așteptată după o mutare reușită: vârful sursei ajunge pe ținta mutării.

    Poziția exactă și yaw-ul cubului mutat sunt necunoscute până la reobservare."""
    source = get_stack(state, move.pick.slot)
    if not source:
        raise DomainValidationError("memoria nu are niciun cub pe {}".format(move.pick.slot))
    cube = replace(source[-1], yaw_deg=None, yaw_confidence=0.0, uv_top=None)
    state = set_stack(state, move.pick.slot, source[:-1])
    target = get_stack(state, move.place.slot)
    return set_stack(state, move.place.slot, target + (cube,))


def _same(a: CubeState, b: CubeState) -> bool:
    """Același cub? Un cub memorat cu culoare necunoscută (dedus doar din înălțime) se potrivește cu orice culoare."""
    return a.color is b.color or (a.color is Color.UNKNOWN and a.color_confidence == 0.0)


def merge_cell(observed: CellState, expected: Tuple[CubeState, ...], before: Tuple[CubeState, ...]) -> CellState:
    if observed.occupancy is not Occupancy.OCCUPIED:
        return replace(observed, stack=())
    top = observed.stack[-1]
    for prior in (expected, before):
        if prior and _same(prior[-1], top):
            return replace(observed, stack=prior[:-1] + (top,))
    for prior in (expected, before):
        for index in range(len(prior) - 2, -1, -1):
            if _same(prior[index], top):  # cuburile de deasupra au dispărut (luate sau căzute)
                return replace(observed, stack=prior[:index] + (top,))
    base = expected if expected else before
    if not base:
        return replace(observed, stack=(top,))
    stack = (base + (top,))[-MAX_STACK_HEIGHT:]
    return replace(observed, stack=stack, reasons=tuple(observed.reasons) + (HEIGHT_UNCERTAIN,))


def merge_location(observed: LocationState, expected: Optional[LocationState], before: Optional[LocationState] = None) -> LocationState:
    """Observația (vârfurile) + memoria (nivelurile ascunse) → starea locației."""
    cells: List[CellState] = []
    for cell in observed.cells:
        exp = expected.cell(cell.coord).stack if expected is not None else ()
        bef = before.cell(cell.coord).stack if before is not None else ()
        cells.append(merge_cell(cell, exp, bef))
    return replace(observed, cells=tuple(cells))


def visible_colors(location: LocationState) -> List[str]:
    return sorted(cell.top.color.value for cell in location.cells if cell.top is not None)


def estimated_height(size_ratio: Optional[float], camera_height_mm: float, cube_mm: float) -> Optional[float]:
    """Înălțimea stivei (în cuburi) din mărimea aparentă a feței de sus; None dacă lipsește măsura."""
    if size_ratio is None or size_ratio <= 1.0:
        return None if size_ratio is None else 0.0
    return camera_height_mm * (1.0 - 1.0 / size_ratio) / cube_mm


def apply_height_hint(cell: CellState, estimate: Optional[float], memory_known: bool) -> CellState:
    """O celulă fără istoric (brain-ul nu știe ce e dedesubt) care pare de 2 cuburi primește un cub necunoscut
    dedesubt; una care pare și mai înaltă devine `height_uncertain` (nu e atinsă)."""
    if memory_known or estimate is None or cell.occupancy is not Occupancy.OCCUPIED or len(cell.stack) != 1:
        return cell
    if TWO_CUBES_RANGE[0] <= estimate <= TWO_CUBES_RANGE[1]:
        filler = CubeState(Color.UNKNOWN, 0.0)
        return replace(cell, stack=(filler,) + cell.stack, reasons=tuple(cell.reasons) + (HEIGHT_FROM_SIZE,))
    if estimate > TWO_CUBES_RANGE[1]:
        return replace(cell, reasons=tuple(cell.reasons) + (HEIGHT_UNCERTAIN,))
    return cell
