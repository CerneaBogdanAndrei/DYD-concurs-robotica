"""Conversia `SceneState` ↔ dicționar JSON, cu validare strictă a schemei.

Orice abatere de la schemă produce `SceneStateCorrupt` cu calea exactă a câmpului problematic;
o altă versiune de schemă produce `SceneStateIncompatible`.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Mapping, Optional, Tuple

from brain.domain.cube import Color, Occupancy, TopFace
from brain.domain.errors import DomainValidationError, SceneStateCorrupt, SceneStateIncompatible
from brain.domain.grid import GridCoord
from brain.domain.observation import CellObservation, GridObservation
from brain.domain.scene_state import SCHEMA_VERSION, CellState, CubeState, LocationState, SceneState, ZoneState

_MISSING = object()


def _round(value: Optional[float], digits: int = 4) -> Optional[float]:
    return None if value is None else round(float(value), digits)


def _cube_to_dict(cube: CubeState) -> Dict[str, Any]:
    return {
        "color": cube.color.value,
        "color_confidence": _round(cube.color_confidence),
        "yaw_deg": _round(cube.yaw_deg, 2),
        "yaw_confidence": _round(cube.yaw_confidence),
        "top_face": cube.top_face.value,
        "cube_id": cube.cube_id,
        "uv_top": None if cube.uv_top is None else [_round(cube.uv_top[0]), _round(cube.uv_top[1])],
    }


def scene_to_dict(state: SceneState) -> Dict[str, Any]:
    locations: Dict[str, Any] = {}
    for name, location in state.locations.items():
        locations[name] = {
            "rows": location.rows,
            "cols": location.cols,
            "observed_at": location.observed_at,
            "source": location.source,
            "cells": [
                {
                    "row": cell.coord.row,
                    "col": cell.coord.col,
                    "occupancy": cell.occupancy.value,
                    "occupancy_confidence": _round(cell.occupancy_confidence),
                    "stack": [_cube_to_dict(cube) for cube in cell.stack],
                    "reasons": list(cell.reasons),
                }
                for cell in location.cells
            ],
        }
    return {
        "schema_version": state.schema_version,
        "scene_id": state.scene_id,
        "created_at": state.created_at,
        "updated_at": state.updated_at,
        "locations": locations,
        "zones": {
            name: {"source": zone.source, "stack": [_cube_to_dict(cube) for cube in zone.stack]}
            for name, zone in state.zones.items()
        },
        "metadata": dict(state.metadata),
    }


class _Reader:
    """Acces la câmpuri cu calea completă în mesajele de eroare."""

    def __init__(self, source: str) -> None:
        self.source = source

    def fail(self, path: str, message: str) -> SceneStateCorrupt:
        return SceneStateCorrupt("{}: {}: {}".format(self.source, path, message))

    def field(self, data: Mapping[str, Any], key: str, path: str, types: Tuple[type, ...], optional: bool = False) -> Any:
        full = "{}.{}".format(path, key) if path else key
        if key not in data:
            if optional:
                return None
            raise self.fail(full, "câmp obligatoriu lipsă")
        value = data[key]
        if value is None and optional:
            return None
        if isinstance(value, bool) and bool not in types:
            raise self.fail(full, "tip invalid {!r}".format(value))
        if not isinstance(value, types):
            raise self.fail(full, "tip invalid {!r} (așteptat {})".format(value, "/".join(t.__name__ for t in types)))
        if isinstance(value, float) and not math.isfinite(value):
            raise self.fail(full, "valoare nefinită")
        return value

    def enum(self, data: Mapping[str, Any], key: str, path: str, enum_type: Any) -> Any:
        value = self.field(data, key, path, (str,))
        try:
            return enum_type(value)
        except ValueError:
            allowed = ", ".join(member.value for member in enum_type)
            raise self.fail("{}.{}".format(path, key), "valoare {!r} invalidă (permise: {})".format(value, allowed)) from None


def _read_uv(reader: "_Reader", data: Mapping[str, Any], path: str) -> Optional[Tuple[float, float]]:
    value = reader.field(data, "uv_top", path, (list,), optional=True)
    if value is None:
        return None
    if len(value) != 2 or not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value):
        raise reader.fail(path + ".uv_top", "trebuie [u, v] numeric")
    return (float(value[0]), float(value[1]))


def _read_stack(reader: "_Reader", stack_raw: List[Any], path: str) -> List[CubeState]:
    stack: List[CubeState] = []
    for level, cube_raw in enumerate(stack_raw):
        cube_path = "{}.stack[{}]".format(path, level)
        if not isinstance(cube_raw, Mapping):
            raise reader.fail(cube_path, "trebuie să fie un obiect")
        try:
            stack.append(
                CubeState(
                    color=reader.enum(cube_raw, "color", cube_path, Color),
                    color_confidence=float(reader.field(cube_raw, "color_confidence", cube_path, (int, float))),
                    yaw_deg=reader.field(cube_raw, "yaw_deg", cube_path, (int, float), optional=True),
                    yaw_confidence=float(reader.field(cube_raw, "yaw_confidence", cube_path, (int, float))),
                    top_face=reader.enum(cube_raw, "top_face", cube_path, TopFace),
                    cube_id=reader.field(cube_raw, "cube_id", cube_path, (str,), optional=True),
                    uv_top=_read_uv(reader, cube_raw, cube_path),
                )
            )
        except DomainValidationError as exc:
            raise reader.fail(cube_path, str(exc)) from None
    return stack


def scene_from_dict(data: Any, source: str = "<memorie>") -> SceneState:
    reader = _Reader(source)
    if not isinstance(data, Mapping):
        raise reader.fail("<rădăcină>", "trebuie să fie un obiect JSON")
    version = reader.field(data, "schema_version", "", (int,))
    if version != SCHEMA_VERSION:
        raise SceneStateIncompatible(
            "{}: schema_version {} nu e compatibilă cu versiunea curentă {}".format(source, version, SCHEMA_VERSION)
        )
    locations_raw = reader.field(data, "locations", "", (dict,))
    locations: Dict[str, LocationState] = {}
    for name, loc_raw in locations_raw.items():
        path = "locations.{}".format(name)
        if not isinstance(loc_raw, Mapping):
            raise reader.fail(path, "trebuie să fie un obiect")
        rows = reader.field(loc_raw, "rows", path, (int,))
        cols = reader.field(loc_raw, "cols", path, (int,))
        cells_raw = reader.field(loc_raw, "cells", path, (list,))
        cells: List[CellState] = []
        for index, cell_raw in enumerate(cells_raw):
            cell_path = "{}.cells[{}]".format(path, index)
            if not isinstance(cell_raw, Mapping):
                raise reader.fail(cell_path, "trebuie să fie un obiect")
            stack_raw = reader.field(cell_raw, "stack", cell_path, (list,))
            stack = _read_stack(reader, stack_raw, cell_path)
            reasons = reader.field(cell_raw, "reasons", cell_path, (list,))
            if not all(isinstance(r, str) for r in reasons):
                raise reader.fail(cell_path + ".reasons", "trebuie să conțină doar șiruri")
            try:
                cells.append(
                    CellState(
                        coord=GridCoord(
                            reader.field(cell_raw, "row", cell_path, (int,)), reader.field(cell_raw, "col", cell_path, (int,))
                        ),
                        occupancy=reader.enum(cell_raw, "occupancy", cell_path, Occupancy),
                        occupancy_confidence=float(reader.field(cell_raw, "occupancy_confidence", cell_path, (int, float))),
                        stack=tuple(stack),
                        reasons=tuple(reasons),
                    )
                )
            except DomainValidationError as exc:
                raise reader.fail(cell_path, str(exc)) from None
        try:
            locations[name] = LocationState(
                name=name,
                rows=rows,
                cols=cols,
                cells=tuple(cells),
                observed_at=reader.field(loc_raw, "observed_at", path, (str,), optional=True),
                source=reader.field(loc_raw, "source", path, (str,)),
            )
        except DomainValidationError as exc:
            raise reader.fail(path, str(exc)) from None
    zones: Dict[str, ZoneState] = {}
    for name, zone_raw in reader.field(data, "zones", "", (dict,)).items():
        path = "zones.{}".format(name)
        if not isinstance(zone_raw, Mapping):
            raise reader.fail(path, "trebuie să fie un obiect")
        try:
            zones[name] = ZoneState(
                name=name,
                stack=tuple(_read_stack(reader, reader.field(zone_raw, "stack", path, (list,)), path)),
                source=reader.field(zone_raw, "source", path, (str,)),
            )
        except DomainValidationError as exc:
            raise reader.fail(path, str(exc)) from None
    metadata = reader.field(data, "metadata", "", (dict,))
    try:
        return SceneState(
            scene_id=reader.field(data, "scene_id", "", (str,)),
            created_at=reader.field(data, "created_at", "", (str,)),
            updated_at=reader.field(data, "updated_at", "", (str,)),
            locations=locations,
            metadata=dict(metadata),
            schema_version=version,
            zones=zones,
        )
    except DomainValidationError as exc:
        raise reader.fail("<rădăcină>", str(exc)) from None


def _cell_observation_to_dict(cell: CellObservation) -> Dict[str, Any]:
    return {
        "row": cell.coord.row,
        "col": cell.coord.col,
        "occupancy": cell.occupancy.value,
        "occupancy_confidence": _round(cell.occupancy_confidence),
        "color": cell.color.value,
        "color_confidence": _round(cell.color_confidence),
        "yaw_deg": _round(cell.yaw_deg, 2),
        "yaw_confidence": _round(cell.yaw_confidence),
        "top_face": cell.top_face.value,
        "uv_top": None if cell.uv_top is None else [_round(cell.uv_top[0]), _round(cell.uv_top[1])],
        "reasons": list(cell.reasons),
        "metrics": {k: _round(v) for k, v in cell.metrics.items() if v is None or math.isfinite(v)},
    }


def observation_to_dict(observation: GridObservation) -> Dict[str, Any]:
    """Observația completă (cu metrici de diagnostic), pentru rapoarte și depanare."""
    geometry = observation.geometry
    quality = observation.quality
    return {
        "location": observation.location,
        "rows": observation.rows,
        "cols": observation.cols,
        "captured_at": observation.captured_at,
        "actionable": observation.actionable,
        "reasons": list(observation.reasons),
        "processing_ms": observation.processing_ms,
        "geometry": {
            "method": geometry.method,
            "ok": geometry.ok,
            "corners_px": [list(c) for c in geometry.corners_px],
            "structure_score": geometry.structure_score,
            "metrics": {k: _round(v) for k, v in geometry.metrics.items() if math.isfinite(v)},
        },
        "quality": {
            "frames_total": quality.frames_total,
            "frames_valid": quality.frames_valid,
            "flags": list(quality.flags),
            "mean_brightness": quality.mean_brightness,
            "contrast": quality.contrast,
            "sharpness": quality.sharpness,
        },
        "cells": [_cell_observation_to_dict(cell) for cell in observation.cells],
    }
