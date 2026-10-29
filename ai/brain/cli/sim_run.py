"""`python -m brain sim run`: o sarcină cap-coadă în simulare (MockArm + SimWorld + percepție + FSM).

Timpul e simulat (`ManualClock` + `SimDurations`): rularea durează cât calculul, iar bugetul de 5 minute se
aplică timpului simulat. Reușita se verifică pe adevărul simulat, nu pe ce crede brain-ul.
"""

from __future__ import annotations

import dataclasses
import json
import re
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from brain.adapters.arm.mock_arm import FaultSpec, MockArm
from brain.adapters.clock import ManualClock
from brain.configuration.loader import AppConfig
from brain.domain.errors import BrainError
from brain.memory.store import SceneStore
from brain.orchestration.fsm import TaskRunner
from brain.orchestration.observer import CameraPerception, LocationObserver
from brain.orchestration.proximity_check import ProximityVerifier
from brain.orchestration.session import ArmSession, DurationJournal, TimeBudget
from brain.planning.cost import CostModel
from brain.planning.planner import Planner
from brain.ports.arm import MODE_SIMULATION
from brain.simulation.scenario import OraclePerception, Scenario, check_goal, describe_world, make_renderer
from brain.simulation.sim_world import SimPhysics

PERCEPTIONS = ("oracle", "render")
SIM_SENSOR_ABOVE_JAW_MM = 25.0  # cunoscut doar în simulare; pe robot e NECALIBRAT
_FAULT_RE = re.compile(r"^\s*(\w+):(\w+)(?:@(\d+)|%([0-9.]+))\s*$")


@dataclass
class SimOptions:
    perception: str = "oracle"
    faults: Tuple[FaultSpec, ...] = ()
    proximity: bool = False
    dry_run: bool = False
    gif: Optional[Path] = None
    out_dir: Optional[Path] = None
    service_url: Optional[str] = None  # serviciul colegilor pornit cu --sim (TeeArm); None = doar MockArm
    physics: Optional[SimPhysics] = None


def parse_faults(text: Optional[str]) -> Tuple[FaultSpec, ...]:
    """`pick:grasp_fail@1,place:drop@2,pick:unreachable%0.2` → erori injectate (la apelul N sau cu probabilitate)."""
    if not text:
        return ()
    faults = []
    for item in text.split(","):
        match = _FAULT_RE.match(item)
        if not match:
            raise BrainError("eroare injectată invalidă: {!r} (ex. pick:grasp_fail@1 sau place:drop%0.1)".format(item))
        method, kind, call, probability = match.groups()
        try:
            faults.append(FaultSpec(method, kind, int(call) if call else None, float(probability) if probability else 0.0))
        except ValueError as exc:
            raise BrainError(str(exc)) from None
    return tuple(faults)


def run_scenario(config: AppConfig, scenario: Scenario, options: Optional[SimOptions] = None) -> Dict[str, Any]:
    options = options or SimOptions()
    if options.perception not in PERCEPTIONS:
        raise BrainError("percepție necunoscută: {}".format(options.perception))
    system = config.system
    started = time.perf_counter()
    clock = ManualClock()
    world = scenario.world
    if options.physics is not None:
        world.physics = options.physics
    poses = {system.location(name).observe_pose: name for name in system.planning.active_locations
             if system.location(name).observe_pose}
    mock = MockArm(world, poses, clock, faults=options.faults, seed=scenario.seed)
    arm: Any = mock
    if options.service_url:
        from brain.adapters.arm.tee_arm import TeeArm
        from brain.adapters.arm.team_arm import TeamArm

        arm = TeeArm(TeamArm(options.service_url, timeout_s=system.arm.timeout_s), mock)
    out = options.out_dir
    if out is not None:
        out.mkdir(parents=True, exist_ok=True)
        timings = out / "timings.jsonl"
        if timings.exists():
            timings.unlink()
    journal = DurationJournal(out / "timings.jsonl" if out is not None else None, simulated=True)
    task_name = _task_name(scenario)
    session = ArmSession(arm, clock, journal, MODE_SIMULATION, task=task_name)
    session.connect()
    if options.perception == "oracle":
        perception: Any = OraclePerception(world, system)
    else:
        from brain.adapters.camera import SyntheticCamera
        from brain.perception.pipeline import VisionPipeline

        camera = SyntheticCamera(make_renderer(world, system, scenario.seed), poses, lambda: mock.current_pose)
        perception = CameraPerception(camera, VisionPipeline(system, config.colors), system.vision.frames_to_aggregate)
    observer = LocationObserver(system, perception, clock, journal, session, task=task_name)
    proximity = None
    if options.proximity:
        from brain.adapters.proximity.simulated import SimulatedProximity

        sensor = SimulatedProximity(world, system.proximity, clock, sensor_above_jaw_mm=SIM_SENSOR_ABOVE_JAW_MM,
                                    seed=scenario.seed)
        proximity = ProximityVerifier(sensor, system.proximity, system.cube.size_mm, SIM_SENSOR_ABOVE_JAW_MM)
    planner = Planner(system, config.sort_map, CostModel.from_calib_file(system.planning.cost, system.arm.calib_json))
    budget = TimeBudget(clock, system.timing.task_limit_s, system.timing.reserve_s)
    store = SceneStore(out / "scene_state.json") if out is not None else None
    recorder = None
    if options.gif is not None:
        from brain.simulation.schematic import Recorder

        recorder = Recorder(world, "{} ({})".format(scenario.name, scenario.goal.describe()))
    runner = TaskRunner(scenario.goal, config, planner, observer, clock, budget, session, proximity, store,
                        initial_state=scenario.initial_memory(system), execute=not options.dry_run, task=task_name,
                        on_step=recorder)
    result = runner.run()
    success, problems = check_goal(world, scenario.goal, config.sort_map)
    report = result.to_dict()
    report.update({
        "scenario": scenario.name, "seed": scenario.seed, "tags": list(scenario.tags), "note": scenario.note,
        "perception": options.perception, "proximity": options.proximity, "dry_run": options.dry_run,
        "arm": "tee(service+mock)" if options.service_url else "mock",
        "faults": ["{}:{}{}".format(f.method, f.kind, "@{}".format(f.call) if f.call else "%{}".format(f.probability))
                   for f in options.faults],
        "success": bool(success) and not options.dry_run, "truth_problems": problems,
        "expected_success": scenario.expect_success, "simulated_time_s": round(clock.monotonic(), 2),
        "within_time_limit": clock.monotonic() <= system.timing.task_limit_s,
        "sim_events": list(world.events), "world_final": describe_world(world),
        "wall_time_s": round(time.perf_counter() - started, 2),
        "note_timing": "timpi SIMULAȚI (SimDurations), nu măsurători",
    })
    if recorder is not None and options.gif is not None:
        from brain.simulation.schematic import save_animation

        recorder.finish("final: {} ({})".format("REUSIT" if success else "NEREUSIT", scenario.goal.describe()))
        report["gif"] = str(save_animation(recorder.frames, options.gif))
    if out is not None:
        (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return report


def _task_name(scenario: Scenario) -> str:
    from brain.planning.goals import ReproduceGoal, SortGoal, TowerGoal

    if isinstance(scenario.goal, TowerGoal):
        return "tower"
    if isinstance(scenario.goal, ReproduceGoal):
        return "reproduce"
    if isinstance(scenario.goal, SortGoal):
        return "sort"
    return "task"


def summarize_batch(reports: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Rata de succes și timpul simulat per sarcină."""
    by_task: Dict[str, List[Dict[str, Any]]] = {}
    for report in reports:
        by_task.setdefault(report["task"], []).append(report)
    summary: Dict[str, Any] = {}
    for task, items in sorted(by_task.items()):
        times = [r["simulated_time_s"] for r in items]
        expected = [r for r in items if r.get("expected_success")]
        summary[task] = {
            "scenarios": len(items),
            "success": sum(1 for r in items if r["success"]),
            "success_rate": round(sum(1 for r in items if r["success"]) / len(items), 3),
            "success_rate_feasible": round(sum(1 for r in expected if r["success"]) / len(expected), 3) if expected else None,
            "simulated_time_median_s": round(statistics.median(times), 1),
            "simulated_time_max_s": round(max(times), 1),
            "over_time_limit": sum(1 for r in items if not r["within_time_limit"]),
            "failures": ["{} (seed {}): {}".format(r["scenario"], r["seed"], "; ".join(r["truth_problems"][:3]) or r.get("error") or "")
                         for r in items if not r["success"]],
        }
    return summary


def with_physics(options: SimOptions, **changes: Any) -> SimOptions:
    physics = dataclasses.replace(options.physics or SimPhysics(), **changes)
    return dataclasses.replace(options, physics=physics)
