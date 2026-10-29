import json
import os

import pytest

from brain.domain.cube import Color, Occupancy, TopFace
from brain.domain.errors import SceneStateCorrupt, SceneStateError, SceneStateExists, SceneStateIncompatible, SceneStateNotFound
from brain.domain.grid import GridCoord
from brain.domain.scene_state import CellState, CubeState, LocationState, SceneState, ZoneState
from brain.memory import SceneStore, scene_from_dict, scene_to_dict


def _state() -> SceneState:
    location = LocationState.empty("source", 3, 3, source="perception")
    location = location.with_cell(
        CellState(GridCoord(1, 2), Occupancy.OCCUPIED, 0.97,
                  (CubeState(Color.RED, 0.91, 12.5, 0.8, TopFace.ICON, "red", uv_top=(2.4, 1.6)),), ("off_center",))
    )
    location = location.with_cell(CellState(GridCoord(0, 0), Occupancy.UNKNOWN, 0.0, reasons=("edges_without_color",)))
    zone = ZoneState("zone_blue", (CubeState(Color.BLUE, 0.9), CubeState(Color.BLUE, 0.8, top_face=TopFace.FULL)))
    return SceneState.new("s1-test", {"task": "s1", "mode": "A"}).with_location(location).with_zone(zone)


def test_round_trip_dict():
    state = _state()
    assert scene_from_dict(scene_to_dict(state)) == state


def test_save_load_after_restart(tmp_path):
    path = tmp_path / "mem" / "state.json"
    state = _state()
    SceneStore(path).save(state)
    reloaded = SceneStore(path).load()  # instanță nouă = proces repornit
    assert reloaded == state
    assert not [p for p in path.parent.iterdir() if p.name.endswith(".tmp")]
    assert json.loads(path.read_text(encoding="utf-8"))["schema_version"] == 2


def test_refuses_accidental_overwrite(tmp_path):
    path = tmp_path / "state.json"
    store = SceneStore(path)
    store.save(_state())
    with pytest.raises(SceneStateExists):
        store.save(SceneState.new("altceva"))
    store.save(SceneState.new("altceva"), overwrite=True)
    assert store.load().scene_id == "altceva"


def test_missing_file(tmp_path):
    with pytest.raises(SceneStateNotFound):
        SceneStore(tmp_path / "nope.json").load()


def test_corrupt_file_is_detected_and_protected(tmp_path):
    path = tmp_path / "state.json"
    path.write_text('{"schema_version": 1, "scene_id": ', encoding="utf-8")
    store = SceneStore(path)
    state = _state()
    with pytest.raises(SceneStateCorrupt):
        store.load()
    with pytest.raises(SceneStateCorrupt):
        store.save(state, overwrite=True)
    assert path.read_text(encoding="utf-8").startswith('{"schema_version": 1, "scene_id": ')
    store.save(state, overwrite=True, replace_invalid=True)
    assert store.load() == state


def test_incompatible_schema_is_not_overwritten(tmp_path):
    path = tmp_path / "state.json"
    data = scene_to_dict(_state())
    data["schema_version"] = 99
    path.write_text(json.dumps(data), encoding="utf-8")
    store = SceneStore(path)
    with pytest.raises(SceneStateIncompatible):
        store.load()
    with pytest.raises(SceneStateIncompatible):
        store.save(_state(), overwrite=True)


@pytest.mark.parametrize(
    "mutate, fragment",
    [
        (lambda d: d["locations"]["source"]["cells"][5]["stack"][0].update(color="purple"), "cells[5].stack[0].color"),
        (lambda d: d["locations"]["source"]["cells"][5].update(occupancy_confidence=1.5), "cells[5]"),
        (lambda d: d["locations"]["source"]["cells"].pop(), "locations.source"),
        (lambda d: d["locations"]["source"]["cells"][5]["stack"][0].update(yaw_deg=60.0), "cells[5].stack[0]"),
        (lambda d: d.pop("scene_id"), "scene_id"),
        (lambda d: d["zones"]["zone_blue"]["stack"][0].update(uv_top=[1]), "zones.zone_blue.stack[0].uv_top"),
    ],
)
def test_schema_violations_name_the_field(mutate, fragment):
    data = scene_to_dict(_state())
    mutate(data)
    with pytest.raises(SceneStateCorrupt) as exc_info:
        scene_from_dict(data, source="test.json")
    assert fragment in str(exc_info.value)


def test_failed_replace_keeps_previous_file(tmp_path, monkeypatch):
    path = tmp_path / "state.json"
    store = SceneStore(path)
    store.save(_state())
    before = path.read_bytes()

    def broken_replace(src, dst):
        raise OSError("disc plin (simulat)")

    monkeypatch.setattr(os, "replace", broken_replace)
    with pytest.raises(OSError):
        store.save(SceneState.new("nou"), overwrite=True)
    assert path.read_bytes() == before
    assert not [p for p in tmp_path.iterdir() if p.name.endswith(".tmp")]


def test_non_finite_values_are_rejected(tmp_path):
    state = SceneState.new("s", {"temperature": float("nan")})
    with pytest.raises(SceneStateError):
        SceneStore(tmp_path / "s.json").save(state)
