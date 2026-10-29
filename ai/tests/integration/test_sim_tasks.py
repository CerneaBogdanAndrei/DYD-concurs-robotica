"""Sarcinile S1/S2/S3 cap-coadă în simulare: exemplele, scenarii aleatoare, cazuri speciale, robustețe, CLI."""

import dataclasses
import json

import numpy as np
import pytest

from brain.cli.main import EXIT_OK, EXIT_SELFTEST_FAILED, main
from brain.cli.sim_run import SimOptions, parse_faults, run_scenario, summarize_batch
from brain.domain.errors import BrainError
from brain.domain.grid import GridCoord
from brain.domain.plan import SlotRef
from brain.planning.goals import ReproduceGoal, TowerGoal, parse_order, target_from_location
from brain.simulation import scenario as sc

RANDOM_PER_TASK = 20  # 3 sarcini x 20 = 60 de scenarii aleatoare (≥ 50)


def _with(app_config, **planning):
    system = app_config.system
    return dataclasses.replace(app_config, system=dataclasses.replace(
        system, planning=dataclasses.replace(system.planning, **planning)))


@pytest.fixture(scope="module")
def buffered(app_config):
    """Configurația probei (un singur caroiaj) + două puncte tampon (în simulare)."""
    return _with(app_config, buffer_points=("buffer_1", "buffer_2"))


def _strip(report):
    return {k: v for k, v in report.items() if k not in ("wall_time_s",)}


# ---------------------------------------------------------------------- exemplele din regulament
def test_example_s2_tower(app_config):
    report = run_scenario(app_config, sc.example_s2(app_config.system, app_config.sort_map), SimOptions())
    assert report["success"] and report["goal_reached"] and report["final_state"] == "DONE"
    assert report["move_outcomes"] == {"ok": 5} and report["within_time_limit"]


def test_example_s1_reproduce(app_config, buffered):
    stuck = run_scenario(app_config, sc.example_s1(app_config.system, app_config.sort_map), SimOptions())
    assert not stuck["success"] and any("buffer_points" in u for u in stuck["unmet"])  # fără loc liber: imposibil
    report = run_scenario(buffered, sc.example_s1(buffered.system, buffered.sort_map), SimOptions())
    assert report["success"] and report["goal_reached"]
    assert any("neverificabil" in w for w in report["warnings"])  # fața de jos


def test_examples_with_rendered_camera(app_config, buffered):
    """Aceleași exemple, dar cu camera sintetică și viziunea reală (paralaxă, stive înalte, caroiaj plin)."""
    s2 = run_scenario(app_config, sc.example_s2(app_config.system, app_config.sort_map), SimOptions(perception="render"))
    assert s2["success"] and s2["final_state"] == "DONE", s2["moves"]
    s1 = run_scenario(buffered, sc.example_s1(buffered.system, buffered.sort_map), SimOptions(perception="render"))
    assert s1["success"] and s1["final_state"] == "DONE", s1["moves"]


# ---------------------------------------------------------------------- scenarii aleatoare
@pytest.fixture(scope="module")
def random_reports(buffered):
    reports = []
    for task in ("tower", "reproduce", "sort"):
        for seed in range(RANDOM_PER_TASK):
            scenario = sc.random_scenario(task, buffered.system, buffered.sort_map, 500 + seed)
            reports.append(run_scenario(buffered, scenario, SimOptions()))
    return reports


def test_random_scenarios_succeed_when_feasible(random_reports):
    assert len(random_reports) >= 50
    feasible = [r for r in random_reports if r["expected_success"]]
    failed = [(r["scenario"], r["truth_problems"], r["unmet"]) for r in feasible if not r["success"]]
    assert not failed
    assert len(feasible) >= 45
    summary = summarize_batch(random_reports)
    assert set(summary) == {"tower", "reproduce", "sort"}
    assert all(item["over_time_limit"] == 0 for item in summary.values())


def test_infeasible_scenarios_explain_why(random_reports):
    for report in random_reports:
        if report["expected_success"] is None and not report["success"]:
            assert report["unmet"], report["scenario"]
            assert report["final_state"] == "DONE"


def test_determinism(buffered):
    scenario_a = sc.random_scenario("tower", buffered.system, buffered.sort_map, 77)
    scenario_b = sc.random_scenario("tower", buffered.system, buffered.sort_map, 77)
    faults = parse_faults("pick:grasp_fail@2")
    a = run_scenario(buffered, scenario_a, SimOptions(faults=faults))
    b = run_scenario(buffered, scenario_b, SimOptions(faults=faults))
    assert _strip(a) == _strip(b)


# ---------------------------------------------------------------------- cazuri speciale
def test_blocking_cube_on_target_and_missing_color(app_config):
    world = sc.build_world(app_config.system, app_config.sort_map, 3)
    rng = np.random.default_rng(3)
    sc.put_cube(world, rng, "medium", (1, 1), "blue")  # blochează celula turnului
    for cell, color in (((0, 0), "red"), ((2, 2), "yellow")):
        sc.put_cube(world, rng, "medium", cell, color)
    goal = TowerGoal(parse_order("red,yellow,green"), SlotRef("medium", GridCoord(1, 1)))
    report = run_scenario(app_config, sc.Scenario("special", 3, goal, world))
    assert report["moves"][0]["purpose"] == "clear"
    assert not report["success"] and "lipsește 1 cub verde" in report["unmet"]
    assert [c.color for c in world.grids["medium"].stack((1, 1))] == ["red", "yellow"]


def test_sort_with_cubes_already_on_zones(app_config):
    world = sc.build_world(app_config.system, app_config.sort_map, 4, zones=True)
    rng = np.random.default_rng(4)
    world.zones["zone_red"].stack.append(world.new_cube("red", "full", 0.0))
    for cell, color in (((0, 0), "red"), ((1, 1), "green-blue")):
        for name in color.split("-"):
            sc.put_cube(world, rng, "medium", cell, name)
    report = run_scenario(app_config, sc.Scenario("sort_known", 4, sc.SortGoal(), world))
    assert report["success"] and report["move_outcomes"] == {"ok": 3}
    assert [m["move"] for m in report["moves"] if "zone_red" in m["move"]] == ["red medium[0,0] (niv. 0) -> zone_red (niv. 1)"]


def test_reproduce_from_small_onto_large(app_config):
    """Cu mai multe foi pe masă: aranjamentul memorat pe small se construiește pe large."""
    config = _with(app_config, active_locations=("small", "medium", "large"), staging_locations=("small", "medium", "large"))
    world = sc.build_world(config.system, config.sort_map, 9)
    rng = np.random.default_rng(9)
    for (r, c), color in {(0, 0): "red", (1, 1): "yellow", (2, 1): "blue"}.items():
        sc.put_cube(world, rng, "small", (r, c), color)
    memorized = target_from_location(sc.scene_from_world(world, config.system).location("small"))
    goal = ReproduceGoal("large", memorized.cells)
    for color in ("red", "yellow", "blue"):  # cuburile de folosit stau pe medium
        free = [cell for cell in world.grids["medium"].free_cells()]
        sc.put_cube(world, rng, "medium", free[0], color)
    report = run_scenario(config, sc.Scenario("small_to_large", 9, goal, world))
    assert report["success"], report["truth_problems"]
    assert [c.color for c in world.grids["large"].stack((2, 1))] == ["blue"]  # „ales din toate cuburile prezente”


# ---------------------------------------------------------------------- robustețe
@pytest.mark.parametrize("faults, outcome", [
    ("pick:grasp_fail@2", "pick_missed"),
    ("place:drop@3", "mismatch"),
    ("pick:grasp_fail@1,pick:grasp_fail@2,pick:grasp_fail@3", "pick_missed"),
])
def test_recovers_from_injected_faults(app_config, faults, outcome):
    report = run_scenario(app_config, sc.example_s2(app_config.system, app_config.sort_map), SimOptions(faults=parse_faults(faults)))
    assert outcome in report["move_outcomes"]
    assert report["final_state"] == "DONE"
    if faults.count("grasp_fail") < 3:
        assert report["success"]


def test_unreachable_and_communication_errors(app_config):
    unreachable = run_scenario(app_config, sc.example_s2(app_config.system, app_config.sort_map),
                               SimOptions(faults=parse_faults("pick:unreachable@1")))
    assert unreachable["move_outcomes"].get("unreachable_pick") == 1 and unreachable["final_state"] == "DONE"
    timeout = run_scenario(app_config, sc.example_s2(app_config.system, app_config.sort_map),
                           SimOptions(faults=parse_faults("place:timeout@2")))
    assert timeout["final_state"] == "ERROR" and "nu s-a trimis home" in timeout["error"]
    assert not any(t[1] == "home" for t in timeout["transitions"])
    with pytest.raises(BrainError):
        parse_faults("pick:explozie@1")


def test_budget_exceeded_stops_new_moves(app_config):
    system = app_config.system
    tight = dataclasses.replace(app_config, system=dataclasses.replace(
        system, timing=dataclasses.replace(system.timing, task_limit_s=60.0, reserve_s=5.0)))
    report = run_scenario(tight, sc.example_s2(tight.system, tight.sort_map), SimOptions())
    assert report["budget_exhausted"] and report["final_state"] == "DONE" and not report["success"]
    assert report["simulated_time_s"] <= 60.0 and 0 < report["moves_attempted"] < 5


def test_dry_run_only_plans(app_config):
    report = run_scenario(app_config, sc.example_s2(app_config.system, app_config.sort_map), SimOptions(dry_run=True))
    assert report["final_state"] == "DONE" and not report["moves"] and len(report["first_plan"]) == 5
    assert not report["success"]


# ---------------------------------------------------------------------- CLI
def test_cli_sim_run_and_reports(tmp_path, config_dir):
    common = ["--config-dir", str(config_dir)]
    out = tmp_path / "s2"
    gif = tmp_path / "s2.gif"
    assert main(common + ["sim", "run", "--task", "tower", "--example", "--perception", "oracle", "--out", str(out),
                          "--gif", str(gif)]) == EXIT_OK
    report = json.loads((out / "report.json").read_text(encoding="utf-8"))
    assert report["success"] and (out / "scene_state.json").is_file()
    assert gif.exists() or gif.with_suffix("").is_dir()
    assert main(common + ["sim", "run", "--task", "reproduce", "--example", "--perception", "oracle",
                          "--out", str(tmp_path / "s1")]) == EXIT_SELFTEST_FAILED  # fără puncte tampon
    assert main(common + ["sim", "run", "--task", "reproduce", "--example", "--perception", "oracle", "--buffers", "1",
                          "--out", str(tmp_path / "s1b")]) == EXIT_OK
    assert main(common + ["sim", "run", "--task", "tower", "--order", "blue,green", "--target", "medium:r0c0",
                          "--perception", "oracle", "--out", str(tmp_path / "t")]) == EXIT_OK
    assert main(common + ["sim", "run", "--task", "all", "--random", "2", "--perception", "oracle",
                          "--out", str(tmp_path / "rnd")]) == EXIT_OK
    assert json.loads((tmp_path / "rnd" / "summary.json").read_text(encoding="utf-8"))["tower"]["scenarios"] == 2
    assert main(common + ["sim", "run", "--task", "tower", "--order", "blue", "--target", "large:r0c0",
                          "--perception", "oracle", "--out", str(tmp_path / "x")]) == 2  # large nu e activ
    assert main(common + ["report", "timings", "--journal", str(out / "timings.jsonl")]) == EXIT_OK


def test_report_timings_summary(tmp_path):
    from brain.cli.tools import load_journal, summarize_timings

    path = tmp_path / "t.jsonl"
    lines = [{"method": "pick", "duration_s": d, "result": "ok", "simulated": False} for d in (4.0, 5.0, 6.0, 9.0)]
    lines += [{"method": "observe", "duration_s": 2.0, "result": "actionable", "simulated": False},
              {"method": "pick", "duration_s": 0.1, "result": "false", "simulated": False}]
    path.write_text("\n".join(json.dumps(x) for x in lines), encoding="utf-8")
    summary = summarize_timings(load_journal([path]))
    assert summary["methods"]["pick"]["n"] == 4 and summary["methods"]["pick"]["median_s"] == 5.5
    assert summary["proposal"] == {"observe_s": 2.0, "pick_s": 9.0}  # p90 (rang cel mai apropiat): estimare prudentă
