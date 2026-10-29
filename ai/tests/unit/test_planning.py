"""Planificatorul Greedy (S1/S2/S3), modelul de cost și obiectivele."""

import dataclasses
from pathlib import Path

import numpy as np
import pytest

from brain.domain.cube import Color, TopFace
from brain.domain.grid import GridCoord
from brain.domain.plan import SlotRef
from brain.planning.cost import CalibFileError, CostModel, GridModel, load_service_positions
from brain.planning.goals import (
    CellTarget, GoalError, ReproduceGoal, SortGoal, TowerGoal, load_reproduce_goal, missing_text, parse_order, parse_slot,
    reproduce_goal_from_dict, reproduce_goal_to_dict, target_from_location,
)
from brain.planning.planner import Planner
from brain.simulation import scenario as sc

FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"
TARGET = SlotRef("medium", GridCoord(1, 1))


def _config(app_config, **planning):
    system = app_config.system
    return dataclasses.replace(app_config, system=dataclasses.replace(
        system, planning=dataclasses.replace(system.planning, **planning)))


def _planner(config, calib=None):
    return Planner(config.system, config.sort_map, CostModel.from_calib_file(config.system.planning.cost, calib))


def _world(config, cubes, zones=False, seed=0):
    """cubes: {(r, c): "red-blue"} de jos în sus, pe medium."""
    world = sc.build_world(config.system, config.sort_map, seed, zones=zones)
    rng = np.random.default_rng(seed)
    for cell, colors in cubes.items():
        for color in colors.split("-"):
            sc.put_cube(world, rng, "medium", cell, color, "full")
    return world


def _replay(plan, state_world_stacks):
    """Aplică mutările pe un dicționar slot → listă de culori; verifică că fiecare ia vârful real."""
    stacks = {k: list(v) for k, v in state_world_stacks.items()}
    for move in plan.moves:
        source = stacks.setdefault(str(move.pick.slot), [])
        assert source and len(source) - 1 == move.pick.level, "mutarea nu ia vârful: {}".format(move)
        assert source[-1] == move.cube.color.value
        target = stacks.setdefault(str(move.place.slot), [])
        assert len(target) == move.place.level
        target.append(source.pop())
    return stacks


def _stacks(world):
    result = {}
    for name, grid in world.grids.items():
        for (r, c), stack in grid.stacks.items():
            result[str(SlotRef(name, GridCoord(r, c)))] = [cube.color for cube in stack]
    for name, zone in world.zones.items():
        result[name] = [cube.color for cube in zone.stack]
    return result


# ---------------------------------------------------------------------- S2: turn
def test_tower_example_builds_bottom_up(app_config):
    world = _world(app_config, {(0, 0): "red", (0, 1): "yellow", (0, 2): "green", (2, 0): "blue", (2, 2): "red"})
    goal = TowerGoal(parse_order("red,yellow,green,blue,red"), TARGET)
    plan = _planner(app_config).plan(goal, sc.scene_from_world(world, app_config.system))
    assert plan.complete and len(plan.moves) == 5
    assert [m.place.level for m in plan.moves] == [0, 1, 2, 3, 4]
    assert _replay(plan, _stacks(world))[str(TARGET)] == ["red", "yellow", "green", "blue", "red"]
    assert all(m.cost_is_substitute for m in plan.moves)  # fără calib.json
    assert plan.estimated_s == pytest.approx(sum(m.estimated_s for m in plan.moves))


def test_tower_keeps_correct_base_and_clears_wrong_top(app_config):
    world = _world(app_config, {(1, 1): "red-blue", (0, 0): "yellow"})
    plan = _planner(app_config).plan(TowerGoal(parse_order("red,yellow"), TARGET), sc.scene_from_world(world, app_config.system))
    assert plan.complete
    assert plan.moves[0].purpose == "clear" and plan.moves[0].pick.slot == TARGET and plan.moves[0].pick.level == 1
    assert plan.moves[1].cube.color is Color.YELLOW and plan.moves[1].place.level == 1
    assert _replay(plan, _stacks(world))[str(TARGET)] == ["red", "yellow"]


def test_tower_already_done(app_config):
    world = _world(app_config, {(1, 1): "green-blue"})
    plan = _planner(app_config).plan(TowerGoal(parse_order("green,blue"), TARGET), sc.scene_from_world(world, app_config.system))
    assert plan.already_done


def test_tower_unburies_needed_cube(app_config):
    world = _world(app_config, {(0, 0): "red-green", (0, 2): "yellow"})
    plan = _planner(app_config).plan(TowerGoal(parse_order("red"), TARGET), sc.scene_from_world(world, app_config.system))
    assert plan.complete and [m.purpose for m in plan.moves] == ["clear", "goal"]
    assert plan.moves[0].cube.color is Color.GREEN and plan.moves[0].place.slot not in (TARGET, SlotRef("medium", GridCoord(0, 0)))


def test_tower_reports_missing_and_level_limit(app_config):
    world = _world(app_config, {(0, 0): "red", (0, 1): "yellow"})
    plan = _planner(app_config).plan(TowerGoal(parse_order("red,red,yellow,blue,green,blue"), TARGET),
                                     sc.scene_from_world(world, app_config.system))
    assert not plan.complete
    text = " | ".join(plan.unmet)
    assert "nivel peste limită" in text and "maximum 5" in text
    assert missing_text(Color.RED, 1) in text and missing_text(Color.BLUE, 1) in text
    assert "prea puține cuburi" in text
    assert len(plan.moves) == 1  # doar baza roșie se poate pune


def test_tower_reveals_hidden_cube(app_config):
    world = _world(app_config, {(0, 0): "red-yellow", (2, 2): "blue"})
    state = sc.scene_from_world(world, app_config.system)
    # Memoria nu știe culoarea de dedesubt (ca după o estimare a înălțimii din imagine).
    from brain.domain.scene_state import CubeState
    from brain.orchestration.memory_merge import get_stack, set_stack

    slot = SlotRef("medium", GridCoord(0, 0))
    state = set_stack(state, slot, (CubeState(Color.UNKNOWN, 0.0),) + get_stack(state, slot)[1:])
    plan = _planner(app_config).plan(TowerGoal(parse_order("red"), TARGET), state)
    assert plan.moves and plan.moves[0].purpose == "clear" and "dezvelește" in plan.moves[0].reason
    assert not plan.unmet and not plan.complete and len(plan.moves) == 1
    assert any("ascunse" in w for w in plan.warnings) and any("reobservare" in w for w in plan.warnings)


def test_untrusted_cubes_are_not_moved(app_config):
    world = _world(app_config, {(0, 0): "red", (1, 1): "yellow"})
    state = sc.scene_from_world(world, app_config.system)
    from brain.domain.scene_state import CubeState
    from brain.orchestration.memory_merge import set_stack

    state = set_stack(state, TARGET, (CubeState(Color.YELLOW, 0.3),))  # încredere sub min_confidence
    plan = _planner(app_config).plan(TowerGoal(parse_order("red"), TARGET), state)
    assert not plan.moves and any("nesigur" in u for u in plan.unmet)


def test_avoided_sources_are_skipped(app_config):
    world = _world(app_config, {(0, 0): "red", (2, 2): "red"})
    planner = _planner(app_config)
    state = sc.scene_from_world(world, app_config.system)
    first = planner.plan(TowerGoal(parse_order("red"), TARGET), state).moves[0].pick.slot
    second = planner.plan(TowerGoal(parse_order("red"), TARGET), state, avoid_sources=frozenset({first})).moves[0].pick.slot
    assert first != second


def test_pick_uses_parallax_corrected_uv_and_observed_yaw(app_config):
    world = _world(app_config, {(0, 0): "red"})
    world.grids["medium"].stack((0, 0))[0].offset_mm = (4.0, -3.0)
    world.grids["medium"].stack((0, 0))[0].yaw_deg = 11.0
    plan = _planner(app_config).plan(TowerGoal(parse_order("red"), TARGET), sc.scene_from_world(world, app_config.system))
    pick = plan.moves[0].pick
    assert pick.uv == pytest.approx((0.5 + 4.0 / 43.0, 0.5 - 3.0 / 43.0), abs=1e-3)  # centrul bazei, nu al feței de sus
    assert pick.yaw_deg == pytest.approx(11.0) and plan.moves[0].place.yaw_deg == 0.0


# ---------------------------------------------------------------------- S1: reproducere
def _goal_rows(rows):
    return reproduce_goal_from_dict({"location": "medium", "rows": rows})


def test_reproduce_permutation_needs_a_buffer(app_config):
    rows = [list(r) for r in sc.EXAMPLE_S1_ROWS]
    world = _world(app_config, {(r, c): rows[(r + 1) % 3][c] for r in range(3) for c in range(3)})
    goal = _goal_rows(rows)
    state = sc.scene_from_world(world, app_config.system)
    stuck = _planner(app_config).plan(goal, state)
    assert not stuck.complete and any("buffer_points" in u for u in stuck.unmet)
    config = _config(app_config, buffer_points=("buffer_1",))
    world_b = sc.build_world(config.system, config.sort_map, 0)
    world_b.grids["medium"].stacks = world.grids["medium"].stacks
    plan = _planner(config).plan(goal, sc.scene_from_world(world_b, config.system))
    assert plan.complete and sum(1 for m in plan.moves if m.place.slot == SlotRef("buffer_1")) >= 1
    final = _replay(plan, _stacks(world_b))
    assert all(final[str(SlotRef("medium", GridCoord(r, c)))] == [rows[r][c]] for r in range(3) for c in range(3))


def test_reproduce_partial_with_empty_cells_and_bottom_warning(app_config):
    goal = reproduce_goal_from_dict({"location": "medium", "cells": {
        "r0c0": {"top": "red", "bottom": "red"}, "r1c1": "empty", "r2c2": "blue"}})
    world = _world(app_config, {(1, 1): "blue", (0, 2): "red", (2, 0): "green"})
    plan = _planner(app_config).plan(goal, sc.scene_from_world(world, app_config.system))
    assert plan.complete
    final = _replay(plan, _stacks(world))
    assert final["medium[0,0]"] == ["red"] and final["medium[1,1]"] == [] and final["medium[2,2]"] == ["blue"]
    assert final["medium[2,0]"] == ["green"]  # celulă nelistată: nu contează, nu e atinsă
    assert any("neverificabil" in w for w in plan.warnings)
    assert len(plan.moves) == 2  # albastrul merge direct pe r2c2 (golește și umple într-o mutare)


def test_reproduce_face_policies(app_config):
    goal = reproduce_goal_from_dict({"location": "medium", "cells": {"r0c0": {"top": "red", "face": "icon"}}})
    world = _world(app_config, {(2, 2): "red"})  # fața „full”
    state = sc.scene_from_world(world, app_config.system)
    assert _planner(app_config).plan(goal, state).complete  # prefer: acceptă și altă față
    strict = _config(app_config, match_top_face="require")
    plan = _planner(strict).plan(goal, state)
    assert not plan.complete and not plan.moves


def test_target_memorized_from_observation(app_config):
    world = _world(app_config, {(0, 0): "red", (1, 2): "blue"})
    location = sc.scene_from_world(world, app_config.system).location("medium")
    goal = target_from_location(location)
    assert goal.cells[GridCoord(0, 0)].top is Color.RED and goal.cells[GridCoord(1, 1)].top is None
    again = reproduce_goal_from_dict(reproduce_goal_to_dict(goal))
    assert {k: v.top for k, v in again.cells.items()} == {k: v.top for k, v in goal.cells.items()}
    world.grids["medium"].stack((0, 0)).append(world.new_cube("orange", "full", 0.0))
    with pytest.raises(GoalError):
        target_from_location(sc.scene_from_world(world, app_config.system).location("medium"))


def test_goal_parsing(tmp_path):
    assert parse_order("roșu, galben,verde ,albastru,r") == (Color.RED, Color.YELLOW, Color.GREEN, Color.BLUE, Color.RED)
    assert parse_slot("medium:r1c2") == SlotRef("medium", GridCoord(1, 2))
    assert parse_slot("medium:0,2") == SlotRef("medium", GridCoord(0, 2)) and parse_slot("zone_red").is_zone
    for bad in ("", "red,mov"):
        with pytest.raises(GoalError):
            parse_order(bad)
    path = tmp_path / "t.yaml"
    path.write_text("location: medium\nrows:\n  - [yellow, green, blue]\n  - [red, '-', green]\n  - [blue, red, null]\n",
                    encoding="utf-8")
    goal = load_reproduce_goal(path)
    assert goal.cells[GridCoord(1, 1)] == CellTarget(None) and goal.cells[GridCoord(2, 1)].top is Color.RED
    (tmp_path / "bad.yaml").write_text("location: medium\nrows: [[red]]\n", encoding="utf-8")
    with pytest.raises(GoalError):
        load_reproduce_goal(tmp_path / "bad.yaml")
    (tmp_path / "t.json").write_text('{"location": "medium", "cells": {"r0c0": {"top": "red", "face": "circle"}}}',
                                     encoding="utf-8")
    with pytest.raises(GoalError):
        load_reproduce_goal(tmp_path / "t.json")


# ---------------------------------------------------------------------- S3: sortare
def test_sort_moves_every_known_color_to_its_zone(app_config):
    world = _world(app_config, {(0, 0): "red-blue", (1, 1): "green", (2, 2): "yellow"}, zones=True)
    plan = _planner(app_config).plan(SortGoal(), sc.scene_from_world(world, app_config.system))
    assert plan.complete and len(plan.moves) == 4
    final = _replay(plan, _stacks(world))
    assert final["zone_red"] == ["red"] and final["zone_blue"] == ["blue"]
    assert plan.moves[0].cube.color is Color.BLUE or plan.moves[0].pick.slot != SlotRef("medium", GridCoord(0, 0))


def test_sort_respects_zone_capacity_and_already_sorted(app_config):
    world = _world(app_config, {(0, 0): "red", (0, 1): "red", (0, 2): "red"}, zones=True)
    for _ in range(2):
        world.zones["zone_red"].stack.append(world.new_cube("red", "full", 0.0))
    plan = _planner(app_config).plan(SortGoal(), sc.scene_from_world(world, app_config.system))
    assert len(plan.moves) == 2 and [m.place.level for m in plan.moves] == [2, 3]  # niv. 0..3 (max_level=3)
    assert any("nivel peste limită" in u and "zone_red" in u for u in plan.unmet)


def test_sort_unknown_color_policies(app_config):
    world = _world(app_config, {(0, 0): "green-orange"}, zones=True)
    state = sc.scene_from_world(world, app_config.system)
    leave = _planner(app_config).plan(SortGoal(), state)
    assert not leave.moves and any("blocat sub un cub nesigur" in u for u in leave.unmet)
    staging_config = dataclasses.replace(app_config, sort_map=dataclasses.replace(app_config.sort_map,
                                                                                 unknown_color_policy="staging"))
    plan = _planner(staging_config).plan(SortGoal(), state)
    assert [m.purpose for m in plan.moves] == ["staging", "sort"] and plan.complete


# ---------------------------------------------------------------------- costul
def test_cost_model_from_service_calib(app_config):
    grids, points = load_service_positions(FIXTURES / "calib_brain.json")
    model = grids["medium"]
    assert model.cell_xy(0, 0) == pytest.approx((228.0, -43.0))
    assert model.cell_xy(2, 2) == pytest.approx((142.0, 43.0))
    assert points["zone_red"] == (160.0, 130.0)
    cost = CostModel.from_calib_file(app_config.system.planning.cost, str(FIXTURES / "calib_brain.json"))
    left, right = SlotRef("medium", GridCoord(1, 0)), SlotRef("medium", GridCoord(1, 2))
    value, substitute = cost.travel(left, right)
    assert not substitute and value == pytest.approx(2 * np.degrees(np.arctan2(43.0, 185.0)), abs=0.01)
    world = _world(app_config, {(1, 1): "red-blue"})
    from brain.planning.world import PlanningWorld

    pw = PlanningWorld.from_scene(sc.scene_from_world(world, app_config.system), {"medium": (3, 3)}, [], 0.6)
    assert cost.stack_penalty(pw, left) == app_config.system.planning.cost.adjacent_stack_penalty_deg
    assert cost.travel(left, SlotRef("necalibrat"))[1] is True


def test_calib_file_errors(tmp_path):
    path = tmp_path / "c.json"
    path.write_text('{"sites": {"g": {"kind": "grid", "points": {"r0c0": [1, 2, 3], "r0c1": [2, 2, 3]}}}}', encoding="utf-8")
    with pytest.raises(CalibFileError):
        load_service_positions(path)
    path.write_text("nu e json", encoding="utf-8")
    with pytest.raises(CalibFileError):
        load_service_positions(path)
    assert GridModel.fit({"r0c0": [0, 0], "r0c2": [0, 80], "r2c0": [-80, 0]}).cell_xy(1, 1) == pytest.approx((-40.0, 40.0))
