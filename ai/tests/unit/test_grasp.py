"""Gheara printre cuburi: spațiile libere, deschiderea de abordare, offset-ul (regulile A, B, C), planificatorul,
transmiterea către serviciu și verificarea degetelor în simulare."""

import dataclasses

import numpy as np
import pytest

from brain.adapters.arm.mock_arm import MockArm
from brain.adapters.arm.team_arm import TeamArm
from brain.adapters.clock import ManualClock
from brain.domain.cube import Color, Occupancy, TopFace
from brain.domain.grasp import (
    Clearances,
    Footprint,
    GripperGeometry,
    clearances,
    convex_overlap,
    finger_polygons,
    footprint_from_uv,
    plan_grasp,
    plan_release,
)
from brain.domain.grid import GridCoord
from brain.domain.plan import SlotRef
from brain.domain.scene_state import CellState, CubeState, LocationState, SceneState
from brain.planning.cost import CostModel
from brain.planning.goals import TowerGoal, parse_order, parse_slot
from brain.planning.planner import Planner, _Builder
from brain.planning.world import PlanCube
from brain.simulation.sim_world import SimCube, SimGrid, SimWorld

GRIPPER = GripperGeometry(max_open_mm=60.0, finger_mm=6.0, clearance_mm=0.5, approach_margin_mm=5.0,
                          min_approach_margin_mm=0.5, release_margin_mm=3.0, min_release_margin_mm=1.0)
CELL, CUBE = 44.0, 29.0


def _ring(shift=None, yaw=0.0):
    """Cubul țintă pe r1c1 și cei 8 vecini centrați; `shift` = {(r, c): (du_mm, dv_mm)} pentru unii vecini."""
    shift = shift or {}
    target = footprint_from_uv((1.5, 1.5), yaw, CELL, CUBE)
    others = []
    for r in range(3):
        for c in range(3):
            if (r, c) != (1, 1):
                du, dv = shift.get((r, c), (0.0, 0.0))
                others.append(footprint_from_uv((c + 0.5 + du / CELL, r + 0.5 + dv / CELL), 0.0, CELL, CUBE))
    return target, others


def test_clearances_between_centered_cubes_are_cell_minus_cube():
    target, others = _ring()
    gaps = clearances(target, others)
    assert gaps.to_dict() == {"a-": 15.0, "a+": 15.0, "b-": 15.0, "b+": 15.0}


def test_clearances_follow_shifted_and_rotated_neighbors():
    target, others = _ring({(1, 0): (7.0, 0.0), (0, 1): (0.0, 16.0)})
    gaps = clearances(target, others)
    assert gaps.a_neg == pytest.approx(8.0) and gaps.b_neg == 0.0  # vecinul de sus îl atinge (suprapunere)
    far = footprint_from_uv((2.5 - 3 / CELL, 0.5 + 3 / CELL), 30.0, CELL, CUBE)  # pe diagonală, nu intră în bandă
    assert clearances(target, [far]).minimum() is None


def test_grasp_centered_when_room_on_every_side():
    target, others = _ring()
    plan = plan_grasp(target, clearances(target, others), GRIPPER, CELL)
    assert plan.open_mm == pytest.approx(CUBE + 2 * GRIPPER.approach_margin_mm)  # regula A: 39 mm, nu 60
    assert plan.centered and plan.feasible and plan.status == "centered"


def test_grasp_offset_moves_toward_larger_gap_and_narrow_finger_fits_exactly():
    target, others = _ring({(1, 0): (7.0, 0.0)})  # vecinul din stânga la 8 mm
    gaps = clearances(target, others)
    plan = plan_grasp(target, gaps, GRIPPER, CELL)
    assert plan.feasible and plan.status == "offset"
    offset = plan.offset_mm[0]
    assert offset > 0.0  # spre dreapta, unde e spațiul mai mare
    left_finger_outer = offset - plan.open_mm / 2 - GRIPPER.finger_mm
    neighbor_face = -CUBE / 2 - gaps.a_neg
    assert left_finger_outer - neighbor_face == pytest.approx(GRIPPER.clearance_mm, abs=1e-6)  # „la milimetru”
    assert offset - plan.open_mm / 2 <= -CUBE / 2  # degetul rămâne în afara cubului
    assert plan.offset_uv[0] == pytest.approx(offset / CELL, abs=1e-4)


def test_grasp_reports_risk_when_finger_cannot_fit_and_keeps_cube_inside():
    target, others = _ring({(1, 0): (9.5, 0.0)})  # 5,5 mm < 7 mm
    plan = plan_grasp(target, clearances(target, others), GRIPPER, CELL)
    assert not plan.feasible and plan.status == "risky" and plan.feasible_axes == ("b",)
    assert "nu încape" in plan.notes[0]
    assert abs(plan.offset_mm[0]) <= (plan.open_mm - CUBE) / 2  # gheara înconjoară cubul în continuare
    both = clearances(target, _ring({(1, 0): (9.5, 0), (0, 1): (0, 9.5)})[1])
    assert plan_grasp(target, both, GRIPPER, CELL).status == "blocked"


def test_release_opening_is_small_and_shrinks_near_neighbors():
    assert plan_release(Clearances(), GRIPPER, CUBE).release_mm == pytest.approx(35.0)
    tight = plan_release(Clearances(a_neg=8.0), GRIPPER, CUBE)  # 8 - 6,5 = 1,5 mm de marjă
    assert tight.release_mm == pytest.approx(32.0) and tight.feasible
    hit = plan_release(Clearances(a_pos=5.0), GRIPPER, CUBE)
    assert not hit.feasible and "atinge" in hit.notes[0]


def test_finger_polygons_and_overlap():
    fingers = finger_polygons((0.0, 0.0), 0.0, "a", 39.0, 6.0, 29.0)
    xs = sorted(round(x, 3) for x, _ in fingers[1])
    assert xs[0] == 19.5 and xs[-1] == 25.5
    neighbor = Footprint((44.0, 0.0), 0.0, 29.0).corners()  # fața stângă la 29,5 mm
    assert not convex_overlap(fingers[1], neighbor)
    assert convex_overlap(finger_polygons((0.0, 0.0), 0.0, "a", 60.0, 6.0, 29.0)[1], neighbor)


def _state_with(cells):
    """medium cu cuburi {(r, c): (culoare, uv_top)}; restul goale."""
    states = []
    for r in range(3):
        for c in range(3):
            if (r, c) in cells:
                color, uv = cells[(r, c)]
                cube = CubeState(Color(color), 0.95, yaw_deg=0.0, yaw_confidence=0.9, top_face=TopFace.FULL, uv_top=uv)
                states.append(CellState(GridCoord(r, c), Occupancy.OCCUPIED, 0.98, (cube,)))
            else:
                states.append(CellState(GridCoord(r, c), Occupancy.EMPTY, 0.98))
    return SceneState.new("t").with_location(LocationState("medium", 3, 3, tuple(states), "2026-10-29T00:00:00Z"))


def test_planner_sends_opening_offset_and_release(app_config):
    system = app_config.system
    loc = system.location("medium")
    # Paralaxă: uv_top ales ca baza să fie exact unde vrem (camera la H, nadir din config).
    def top_for(base, height=1):
        k = loc.camera_height_mm / (loc.camera_height_mm - height * system.cube.size_mm)
        return (loc.nadir_uv[0] + (base[0] - loc.nadir_uv[0]) * k, loc.nadir_uv[1] + (base[1] - loc.nadir_uv[1]) * k)

    state = _state_with({(1, 1): ("red", top_for((1.5, 1.5))), (1, 0): ("blue", top_for((0.5 + 6.0 / loc.cell_mm, 1.5)))})
    planner = Planner(system, app_config.sort_map, CostModel.from_calib_file(system.planning.cost, None))
    plan = planner.plan(TowerGoal(parse_order("red"), parse_slot("medium:r0c2")), state)
    move = plan.moves[0]
    assert move.pick.slot == SlotRef("medium", GridCoord(1, 1))
    assert move.pick.open_mm == pytest.approx(39.0) and move.pick.grasp == "offset"
    assert move.pick.uv[0] > 1.5  # centrul ghearei mutat spre dreapta (departe de vecinul albastru)
    assert move.place.release_mm == pytest.approx(35.0)
    assert "gheară 39/35 mm" in plan.explanation()[0]


def test_planner_stacks_on_the_cube_below_and_opens_wider_at_points(app_config):
    system = dataclasses.replace(app_config.system, planning=dataclasses.replace(
        app_config.system.planning, buffer_points=("buffer_1",)))
    loc = system.location("medium")
    k = loc.camera_height_mm / (loc.camera_height_mm - system.cube.size_mm)
    base = (1.5 + 5.0 / loc.cell_mm, 1.5)  # cubul roșu din r1c1 e cu 5 mm spre dreapta
    top = (loc.nadir_uv[0] + (base[0] - loc.nadir_uv[0]) * k, loc.nadir_uv[1] + (base[1] - loc.nadir_uv[1]) * k)
    state = _state_with({(1, 1): ("red", top), (0, 0): ("yellow", (0.5, 0.5))})
    planner = Planner(system, app_config.sort_map, CostModel.from_calib_file(system.planning.cost, None))
    move = planner.plan(TowerGoal(parse_order("red,yellow"), parse_slot("medium:r1c1")), state).moves[0]
    assert move.place.level == 1 and move.place.uv == pytest.approx(base, abs=1e-3)  # peste roșu, nu în centru
    # La un punct tampon (fără vecini, nevăzut de cameră) gheara coboară mai larg.
    builder = _Builder(planner, planner.world_for(TowerGoal(parse_order("red"), parse_slot("medium:r0c2")), state),
                       frozenset(), frozenset(), None)
    action = builder._pick_action(SlotRef("buffer_1"), PlanCube(Color.RED, 0.9), 0)
    assert action.open_mm == pytest.approx(system.cube.size_mm + 2 * system.gripper.point_approach_margin_mm)


def test_team_arm_sends_openings_only_when_service_gripper_is_calibrated():
    arm = TeamArm("http://127.0.0.1:9")
    sent = []
    arm._call = lambda name, body=None: sent.append((name, dict(body or {}))) or {"result": True}  # type: ignore
    arm.pick("medium", (1, 1), open_mm=39.0)
    arm.place("medium", (0, 0), release_mm=35.0)
    assert "grip_mm" not in sent[0][1] and "release_mm" not in sent[1][1]
    arm.grip_control, arm.grip_range = True, (30.0, 37.0)
    arm.pick("medium", (1, 1), open_mm=39.0)
    arm.place("medium", (0, 0), release_mm=35.0)
    assert sent[2][1]["grip_mm"] == 37.0 and sent[3][1]["release_mm"] == 35.0  # adus în intervalul măsurat


def test_mock_arm_reports_finger_collisions_for_wide_opening_only():
    grid = SimGrid("medium", 3, 3, 43.0)
    grid.stack((1, 1)).append(SimCube(1, "red", "full", 0.0))
    grid.stack((1, 2)).append(SimCube(2, "blue", "full", 0.0, (-6.0, 0.0)))  # 8 mm până la roșu
    world = SimWorld([grid], [], 29.0, seed=1)
    arm = MockArm(world, ["observe_medium"], ManualClock(), gripper=GRIPPER)
    target, gaps = footprint_from_uv((1.5, 1.5), 0.0, 43.0, 29.0), None
    gaps = clearances(target, [footprint_from_uv((2.5 - 6.0 / 43.0, 1.5), 0.0, 43.0, 29.0)])
    plan = plan_grasp(target, gaps, GRIPPER, 43.0)
    assert arm.pick("medium", (1, 1), uv=(1.5 + plan.offset_uv[0], 1.5 + plan.offset_uv[1]), open_mm=plan.open_mm)
    assert arm.collisions == [] and world.held is not None and world.held.uid == 1  # prins, centrat de degete
    assert abs(world._held_error_mm[0]) < 1e-9
    arm.place("medium", (1, 1), release_mm=35.0)
    wide = MockArm(world, ["observe_medium"], ManualClock(), gripper=GRIPPER)
    wide.pick("medium", (1, 1))  # fără deschidere cerută = gheara larg deschisă (60 mm)
    assert wide.collisions and "atinge cubul de pe medium(1, 2)" in wide.collisions[0]


def test_config_without_gripper_keeps_old_behaviour(app_config):
    system = dataclasses.replace(app_config.system, gripper=None)
    state = _state_with({(1, 1): ("red", (1.5, 1.5))})
    plan = Planner(system, app_config.sort_map, CostModel.from_calib_file(system.planning.cost, None)).plan(
        TowerGoal(parse_order("red"), parse_slot("medium:r0c2")), state)
    assert plan.moves[0].pick.open_mm is None and plan.moves[0].place.release_mm is None


def test_vision_measures_cube_sides_and_gaps(app_config):
    from brain.perception.pipeline import VisionPipeline
    from brain.simulation.render import make_camera, render_location

    system = app_config.system
    loc = system.location("medium")
    grid = SimGrid("medium", 3, 3, loc.cell_mm)
    grid.stack((1, 1)).append(SimCube(1, "red", "full", 0.0))
    grid.stack((1, 0)).append(SimCube(2, "yellow", "full", 0.0, (5.0, 0.0)))
    camera = make_camera(5, system.grid_geometry("medium"), 3, 3, loc.camera_height_mm, loc.nadir_uv)
    frames = render_location(grid, system.grid_geometry("medium"), camera, 5, np.random.default_rng(2))
    observation = VisionPipeline(system, app_config.colors).analyze(frames, "medium").observation
    red = observation.cell(GridCoord(1, 1))
    expected_gap = loc.cell_mm - system.cube.size_mm - 5.0
    assert red.metrics["gap_a_neg_mm"] == pytest.approx(expected_gap, abs=1.5)
    assert red.metrics["side_w_mm"] == pytest.approx(system.cube.size_mm, abs=2.0)
    assert "gap_a_pos_mm" not in red.metrics  # în dreapta nu e nimic
