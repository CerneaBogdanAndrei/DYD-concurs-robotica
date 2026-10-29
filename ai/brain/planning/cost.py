"""Costul unei mutări, în grade estimate de mișcare a brațului.

Cu `calib.json`-ul serviciului (`arm.calib_json`): poziția fiecărui slot în sistemul robotului vine din
modelul de caroiaj al serviciului (`src/grid.ts`: centrul celulei = O + c·ecol + r·erow, potrivit prin cele
mai mici pătrate din punctele învățate) sau din punctul zonei. Costul dintre două poziții e
max(|Δ unghi bază|, `radial_deg_per_mm`·|Δ rază|), plus `adjacent_stack_penalty_deg` dacă ținta are lângă
ea o stivă de cel puțin 2 cuburi (gheara poate atinge stiva), plus `adjacent_cube_penalty_deg` pentru fiecare
celulă vecină ocupată (sus/jos/stânga/dreapta) la sursă și la țintă: gheara deschisă e mai lată decât cubul,
deci planul preferă mutările la care degetele au loc să coboare.

Fără `calib.json` (sau cu un slot necalibrat): metrică substitut = distanța Manhattan în celule +
`location_switch_deg` la schimbarea locației, marcată `cost_is_substitute`.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np

from brain.configuration.models import CostConfig
from brain.domain.errors import BrainError
from brain.domain.plan import SlotRef
from brain.planning.world import PlanningWorld

_CELL_LABEL = re.compile(r"^r([0-2])c([0-2])$")
STACK_PENALTY_HEIGHT = 2


class CalibFileError(BrainError):
    """`calib.json` al serviciului nu poate fi citit sau nu are formatul așteptat."""


@dataclass(frozen=True)
class GridModel:
    """Modelul de caroiaj al serviciului: centrul celulei (r, c) = O + c·ecol + r·erow (mm, sistemul robotului)."""

    origin: Tuple[float, float]
    ecol: Tuple[float, float]
    erow: Tuple[float, float]

    def cell_xy(self, row: int, col: int) -> Tuple[float, float]:
        return (
            self.origin[0] + col * self.ecol[0] + row * self.erow[0],
            self.origin[1] + col * self.ecol[1] + row * self.erow[1],
        )

    @staticmethod
    def fit(points: Dict[str, Any]) -> "GridModel":
        rows_a, xs, ys = [], [], []
        for label, point in points.items():
            match = _CELL_LABEL.match(str(label).lower())
            if not match or not isinstance(point, (list, tuple)) or len(point) < 2:
                raise CalibFileError("punct de caroiaj invalid: {}: {!r}".format(label, point))
            r, c = int(match.group(1)), int(match.group(2))
            rows_a.append([1.0, c, r])
            xs.append(float(point[0]))
            ys.append(float(point[1]))
        a = np.array(rows_a, dtype=np.float64)
        if len(rows_a) < 3 or np.linalg.matrix_rank(a) < 3:
            raise CalibFileError("caroiajul cere cel puțin 3 celule necoliniare (ex. r0c0, r0c2, r2c0)")
        sx = np.linalg.lstsq(a, np.array(xs), rcond=None)[0]
        sy = np.linalg.lstsq(a, np.array(ys), rcond=None)[0]
        return GridModel((float(sx[0]), float(sy[0])), (float(sx[1]), float(sy[1])), (float(sx[2]), float(sy[2])))


def load_service_positions(path: Any) -> Tuple[Dict[str, GridModel], Dict[str, Tuple[float, float]]]:
    """Citește `calib.json`: caroiajele (modele potrivite) și zonele (punctele xy)."""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise CalibFileError("{}: nu poate fi citit: {}".format(path, exc)) from None
    sites = data.get("sites") if isinstance(data, dict) else None
    if not isinstance(sites, dict):
        raise CalibFileError("{}: lipsește dicționarul 'sites'".format(path))
    grids: Dict[str, GridModel] = {}
    points: Dict[str, Tuple[float, float]] = {}
    for name, site in sites.items():
        kind = site.get("kind") if isinstance(site, dict) else None
        if kind == "grid":
            grids[str(name)] = GridModel.fit(site.get("points") or {})
        elif kind == "point":
            xyz = site.get("xyz")
            if not isinstance(xyz, (list, tuple)) or len(xyz) < 2:
                raise CalibFileError("{}: zona {} nu are xyz".format(path, name))
            points[str(name)] = (float(xyz[0]), float(xyz[1]))
        else:
            raise CalibFileError("{}: locul {} are tipul necunoscut {!r}".format(path, name, kind))
    return grids, points


class CostModel:
    def __init__(
        self,
        cost: CostConfig,
        grids: Optional[Dict[str, GridModel]] = None,
        points: Optional[Dict[str, Tuple[float, float]]] = None,
    ) -> None:
        self.cost = cost
        self.grids = dict(grids or {})
        self.points = dict(points or {})

    @staticmethod
    def from_calib_file(cost: CostConfig, path: Optional[str]) -> "CostModel":
        if not path:
            return CostModel(cost)
        grids, points = load_service_positions(path)
        return CostModel(cost, grids, points)

    def position(self, slot: SlotRef) -> Optional[Tuple[float, float]]:
        if slot.is_zone:
            return self.points.get(slot.location)
        model = self.grids.get(slot.location)
        if model is None or slot.cell is None:
            return None
        return model.cell_xy(slot.cell.row, slot.cell.col)

    def is_substitute(self, *slots: Optional[SlotRef]) -> bool:
        return any(slot is not None and self.position(slot) is None for slot in slots)

    def travel(self, start: Optional[SlotRef], end: SlotRef) -> Tuple[float, bool]:
        """(cost în grade, substitut?) de la `start` (None = brațul în poza de observare) la `end`."""
        if self.is_substitute(start, end):
            return self._substitute(start, end), True
        q = self.position(end)
        assert q is not None
        if start is None:  # din poza de observare: doar rotația bazei de la axa x (înainte)
            return abs(math.degrees(math.atan2(q[1], q[0]))), False
        p = self.position(start)
        assert p is not None
        d_theta = abs(_angle_diff(math.degrees(math.atan2(q[1], q[0])), math.degrees(math.atan2(p[1], p[0]))))
        d_radius = abs(math.hypot(*q) - math.hypot(*p))
        return max(d_theta, self.cost.radial_deg_per_mm * d_radius), False

    def _substitute(self, start: Optional[SlotRef], end: SlotRef) -> float:
        if start is None:
            return self.cost.location_switch_deg
        if start.location != end.location:
            return self.cost.location_switch_deg
        if start.cell is None or end.cell is None:
            return 0.0
        return float(abs(start.cell.row - end.cell.row) + abs(start.cell.col - end.cell.col))

    def stack_penalty(self, world: PlanningWorld, slot: SlotRef) -> float:
        tall = any(world.height(n) >= STACK_PENALTY_HEIGHT for n in world.neighbors(slot))
        return self.cost.adjacent_stack_penalty_deg if tall else 0.0

    def clearance_penalty(self, world: PlanningWorld, slot: SlotRef) -> float:
        if slot.cell is None or not self.cost.adjacent_cube_penalty_deg:
            return 0.0
        crowded = sum(1 for n in world.neighbors(slot)
                      if n.cell is not None and (n.cell.row == slot.cell.row or n.cell.col == slot.cell.col)
                      and world.height(n) > 0)
        return crowded * self.cost.adjacent_cube_penalty_deg

    def move_cost(
        self, world: PlanningWorld, previous: Optional[SlotRef], source: SlotRef, target: SlotRef
    ) -> Tuple[float, bool]:
        """Costul unei mutări: deplasarea până la sursă + transportul până la țintă + penalizări."""
        to_source, sub_a = self.travel(previous, source)
        carry, sub_b = self.travel(source, target)
        penalty = self.stack_penalty(world, source) + self.stack_penalty(world, target)
        penalty += self.clearance_penalty(world, source) + self.clearance_penalty(world, target)
        return round(to_source + carry + penalty, 3), sub_a or sub_b


def _angle_diff(a: float, b: float) -> float:
    return (a - b + 180.0) % 360.0 - 180.0

