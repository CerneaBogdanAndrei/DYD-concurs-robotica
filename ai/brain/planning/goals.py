"""Obiectivele celor trei sarcini și citirea lor din linia de comandă sau din fișiere.

- S2 `TowerGoal`: turn într-o ordine dată (de jos în sus, cu repetiții) pe o celulă dată de juriu;
- S1 `ReproduceGoal`: aranjamentul unui șablon (per celulă: culoarea feței de sus, opțional fața de jos);
- S3 `SortGoal`: fiecare cub pe zona culorii lui (zonele din `sort_map.yaml`).
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple, Union

import yaml

from brain.domain.cube import KNOWN_COLORS, Color, Occupancy, TopFace
from brain.domain.errors import BrainError
from brain.domain.grid import GridCoord, iter_coords
from brain.domain.plan import SlotRef
from brain.domain.scene_state import LocationState

TASKS = ("tower", "sort", "reproduce")

# Nume în română, acceptate la intrare și folosite în mesaje.
_RO_NAMES = {Color.RED: "roșu", Color.YELLOW: "galben", Color.GREEN: "verde", Color.BLUE: "albastru"}
_RO_PLURAL = {Color.RED: "roșii", Color.YELLOW: "galbene", Color.GREEN: "verzi", Color.BLUE: "albastre"}
_ALIASES = {
    "red": Color.RED, "rosu": Color.RED, "roșu": Color.RED, "r": Color.RED,
    "yellow": Color.YELLOW, "galben": Color.YELLOW, "y": Color.YELLOW,
    "green": Color.GREEN, "verde": Color.GREEN, "g": Color.GREEN,
    "blue": Color.BLUE, "albastru": Color.BLUE, "b": Color.BLUE,
}
_EMPTY_WORDS = ("", "-", ".", "empty", "gol", "goala", "goală", "none", "null")
_CELL_RE = re.compile(r"^\s*(?:r\s*)?(\d)\s*[c,\s]\s*(\d)\s*$", re.IGNORECASE)


class GoalError(BrainError, ValueError):
    """Obiectiv invalid (ordine, celulă, fișier țintă)."""


def color_name(color: Color) -> str:
    return _RO_NAMES.get(color, color.value)


def missing_text(color: Color, count: int) -> str:
    """„lipsește 1 cub roșu” / „lipsesc 2 cuburi roșii”."""
    if count == 1:
        return "lipsește 1 cub {}".format(color_name(color))
    return "lipsesc {} cuburi {}".format(count, _RO_PLURAL.get(color, color.value))


def parse_color(text: Any) -> Color:
    key = str(text).strip().lower()
    if key in _ALIASES:
        return _ALIASES[key]
    raise GoalError("culoare necunoscută: {!r} (permise: red, yellow, green, blue sau roșu, galben, verde, albastru)".format(text))


def parse_order(text: Union[str, Sequence[str]]) -> Tuple[Color, ...]:
    """`red,yellow,green,blue,red` → culorile turnului de jos în sus."""
    items = [t for t in (text.split(",") if isinstance(text, str) else text)]
    colors = tuple(parse_color(t) for t in items if str(t).strip())
    if not colors:
        raise GoalError("ordinea turnului e goală")
    return colors


def parse_cell(text: str) -> GridCoord:
    """`r1c2`, `1,2` sau `1 2` → GridCoord(1, 2)."""
    match = _CELL_RE.match(str(text))
    if not match:
        raise GoalError("celulă invalidă: {!r} (ex. r1c2 sau 1,2)".format(text))
    return GridCoord(int(match.group(1)), int(match.group(2)))


def parse_slot(text: str) -> SlotRef:
    """`medium:r1c1` / `medium:1,1` → SlotRef; `zone_red` → zona."""
    if ":" not in text:
        return SlotRef(text.strip())
    location, cell = text.split(":", 1)
    return SlotRef(location.strip(), parse_cell(cell))


@dataclass(frozen=True)
class TowerGoal:
    order: Tuple[Color, ...]  # de jos în sus
    target: SlotRef

    def describe(self) -> str:
        return "turn {} pe {}".format("-".join(c.value for c in self.order), self.target)


@dataclass(frozen=True)
class CellTarget:
    """Ținta unei celule: `top` = None înseamnă celulă goală. `bottom` (fața de jos) nu se poate verifica."""

    top: Optional[Color]
    bottom: Optional[Color] = None
    face: TopFace = TopFace.UNKNOWN


@dataclass(frozen=True)
class ReproduceGoal:
    location: str
    cells: Mapping[GridCoord, CellTarget] = field(default_factory=dict)  # celulele nelistate nu contează

    def describe(self) -> str:
        filled = sum(1 for t in self.cells.values() if t.top is not None)
        return "reproducere pe {}: {} celule cu cub, {} goale".format(self.location, filled, len(self.cells) - filled)


@dataclass(frozen=True)
class SortGoal:
    def describe(self) -> str:
        return "sortare pe zonele hărții"


Goal = Union[TowerGoal, ReproduceGoal, SortGoal]


# ---------------------------------------------------------------------- ținta S1
def target_from_location(location: LocationState) -> ReproduceGoal:
    """Memorează aranjamentul observat: fiecare celulă ocupată → culoarea și fața de sus; goală → goală."""
    cells: Dict[GridCoord, CellTarget] = {}
    problems: List[str] = []
    for cell in location.cells:
        if cell.occupancy is Occupancy.EMPTY:
            cells[cell.coord] = CellTarget(None)
        elif cell.occupancy is Occupancy.OCCUPIED and cell.top is not None and cell.top.color in KNOWN_COLORS:
            if len(cell.stack) > 1:
                problems.append("{}: stivă de {} cuburi (S1 cere câte un cub)".format(cell.coord, len(cell.stack)))
            cells[cell.coord] = CellTarget(cell.top.color, None, cell.top.top_face)
        else:
            problems.append("{}: {} (culoare sau ocupare necunoscută)".format(cell.coord, cell.occupancy.value))
    if problems:
        raise GoalError("aranjamentul de pe {} nu poate fi memorat: {}".format(location.name, "; ".join(problems)))
    return ReproduceGoal(location.name, cells)


def load_reproduce_goal(path: Any, location: Optional[str] = None, rows: int = 3, cols: int = 3) -> ReproduceGoal:
    """Citește ținta S1 din YAML/JSON. Două forme acceptate:

    ```yaml
    location: medium
    rows: [[yellow, green, blue], [red, yellow, green], [blue, red, "-"]]   # "-" = celulă goală
    ```
    sau, per celulă (cu fața de jos și tipul feței de sus, opționale):
    ```yaml
    location: medium
    cells:
      r0c0: {top: yellow, bottom: yellow, face: full}
      r2c2: empty
    ```
    """
    file = Path(path)
    try:
        text = file.read_text(encoding="utf-8")
        data = json.loads(text) if file.suffix.lower() == ".json" else yaml.safe_load(text)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        raise GoalError("{}: nu poate fi citit: {}".format(file, exc)) from None
    return reproduce_goal_from_dict(data, str(file), location, rows, cols)


def reproduce_goal_from_dict(data: Any, source: str = "<țintă>", location: Optional[str] = None, rows: int = 3,
                             cols: int = 3) -> ReproduceGoal:
    if not isinstance(data, dict):
        raise GoalError("{}: trebuie să fie un dicționar".format(source))
    name = location or data.get("location")
    if not isinstance(name, str) or not name:
        raise GoalError("{}: lipsește 'location' (sau --target-location)".format(source))
    cells: Dict[GridCoord, CellTarget] = {}
    if "rows" in data:
        grid = data["rows"]
        if not isinstance(grid, list) or len(grid) != rows or any(not isinstance(r, list) or len(r) != cols for r in grid):
            raise GoalError("{}: 'rows' trebuie să fie {}x{}".format(source, rows, cols))
        for coord in iter_coords(rows, cols):
            cells[coord] = _cell_target(grid[coord.row][coord.col], source, str(coord))
    elif "cells" in data:
        raw = data["cells"]
        if not isinstance(raw, dict):
            raise GoalError("{}: 'cells' trebuie să fie un dicționar".format(source))
        for key, value in raw.items():
            coord = parse_cell(str(key))
            if not coord.within(rows, cols):
                raise GoalError("{}: celula {} e în afara grilei {}x{}".format(source, key, rows, cols))
            cells[coord] = _cell_target(value, source, str(key))
    else:
        raise GoalError("{}: lipsește 'rows' sau 'cells'".format(source))
    return ReproduceGoal(name, cells)


def _cell_target(value: Any, source: str, where: str) -> CellTarget:
    if value is None or (isinstance(value, str) and value.strip().lower() in _EMPTY_WORDS):
        return CellTarget(None)
    if isinstance(value, str):
        return CellTarget(parse_color(value))
    if isinstance(value, dict):
        top = value.get("top")
        if top is None or (isinstance(top, str) and top.strip().lower() in _EMPTY_WORDS):
            return CellTarget(None)
        bottom = value.get("bottom")
        face = value.get("face", "unknown")
        try:
            face_value = TopFace(str(face))
        except ValueError:
            raise GoalError("{}: {}: față necunoscută {!r} (full, icon)".format(source, where, face)) from None
        return CellTarget(parse_color(top), None if bottom is None else parse_color(bottom), face_value)
    raise GoalError("{}: {}: valoare invalidă {!r}".format(source, where, value))


def reproduce_goal_to_dict(goal: ReproduceGoal) -> Dict[str, Any]:
    cells: Dict[str, Any] = {}
    for coord in sorted(goal.cells):
        target = goal.cells[coord]
        key = "r{}c{}".format(coord.row, coord.col)
        if target.top is None:
            cells[key] = "empty"
        else:
            entry: Dict[str, Any] = {"top": target.top.value}
            if target.bottom is not None:
                entry["bottom"] = target.bottom.value
            if target.face is not TopFace.UNKNOWN:
                entry["face"] = target.face.value
            cells[key] = entry
    return {"location": goal.location, "cells": cells}
