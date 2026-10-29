"""`python -m brain run`: o sarcină pe serviciul de mișcare al colegilor (brațul real sau `npm run sim`).

Siguranță:
- implicit doar observă (fără să miște brațul) și planifică; brațul se mișcă NUMAI cu `--execute`;
- cu `--execute`: se verifică modul serviciului (`status.sim` trebuie să fie false, sau true doar cu
  `--service-sim`), locurile cerute trebuie să fie calibrate, iar înainte de prima mișcare se cere confirmare;
- Ctrl+C: comanda curentă se termină, apoi execuția se oprește; NU se trimite `home` automat;
- totul se scrie în `--out`: report.json, timings.jsonl, scene_state.json, log.jsonl.
"""

from __future__ import annotations

import dataclasses
import json
import signal
import sys
import threading
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import yaml

from brain.adapters.clock import SystemClock
from brain.configuration.loader import AppConfig
from brain.domain.errors import BrainError, SafetyRejected
from brain.domain.scene_state import location_from_observation
from brain.memory.store import SceneStore
from brain.orchestration.fsm import TaskRunner
from brain.orchestration.observer import CameraPerception, LocationObserver
from brain.orchestration.session import ArmSession, DurationJournal, TimeBudget
from brain.planning.cost import CostModel
from brain.planning.goals import Goal, reproduce_goal_to_dict, target_from_location
from brain.planning.planner import Planner
from brain.ports.arm import MODE_HARDWARE, MODE_SIMULATION

CONFIRM_WORD = "da"


class StopFlag:
    """Primul Ctrl+C cere oprirea după comanda curentă; al doilea întrerupe imediat (KeyboardInterrupt)."""

    def __init__(self) -> None:
        self.requested = False
        self._previous: Any = None

    def __call__(self) -> bool:
        return self.requested

    def install(self) -> None:
        if threading.current_thread() is not threading.main_thread():
            return
        self._previous = signal.getsignal(signal.SIGINT)

        def handler(signum: int, frame: Any) -> None:
            if self.requested:
                raise KeyboardInterrupt
            self.requested = True
            print("\nOprire cerută: se termină comanda curentă, fără home. Încă un Ctrl+C întrerupe imediat.",
                  file=sys.stderr)

        signal.signal(signal.SIGINT, handler)

    def restore(self) -> None:
        if self._previous is not None:
            signal.signal(signal.SIGINT, self._previous)


def check_sites(config: AppConfig, status: Dict[str, Any], need_zones: bool) -> AppConfig:
    """Locurile folosite trebuie să existe în `status.sites` al serviciului. Punctele tampon lipsă sunt
    scoase (cu avertisment); un șablon activ sau o zonă S3 lipsă oprește rularea."""
    sites = status.get("sites") or {}
    poses = set(status.get("poses") or [])
    missing = [name for name in config.system.planning.active_locations if sites.get(name) != "grid"]
    missing_poses = [config.system.location(n).observe_pose for n in config.system.planning.active_locations
                     if config.system.location(n).observe_pose not in poses]
    if need_zones:
        missing += [z.name for z in config.sort_map.zones if sites.get(z.name) != "point"]
    if missing or missing_poses:
        raise SafetyRejected("locuri/poze necalibrate în serviciu: {} (npm run calib -- celule/zona/poza NUME)".format(
            ", ".join(str(m) for m in missing + missing_poses)))
    buffers = tuple(b for b in config.system.planning.buffer_points if sites.get(b) == "point")
    dropped = [b for b in config.system.planning.buffer_points if b not in buffers]
    if dropped:
        print("ATENȚIE: punctele tampon necalibrate sunt ignorate: {}".format(", ".join(dropped)))
    planning = dataclasses.replace(config.system.planning, buffer_points=buffers)
    return dataclasses.replace(config, system=dataclasses.replace(config.system, planning=planning))


def build_camera(args: Any, observer_pose: Callable[[], Optional[str]], config: AppConfig) -> Any:
    from brain.adapters.camera import FileCamera, UsbCamera

    if args.camera == "usb":
        return UsbCamera(args.camera_index, args.width, args.height)
    if not args.frames_dir:
        raise BrainError("--camera files cere --frames-dir (câte un subdirector per locație, cu PNG-uri)")
    folder = Path(args.frames_dir)
    frames = {}
    for name in config.system.planning.active_locations:
        pose = config.system.location(name).observe_pose
        files = sorted(str(p) for p in (folder / name).glob("*.png")) or sorted(str(p) for p in folder.glob("*.png"))
        if not files:
            raise BrainError("nu există imagini pentru {} în {}".format(name, folder))
        frames[pose or name] = files
    return FileCamera(frames, observer_pose)


def run_task(config: AppConfig, goal: Optional[Goal], args: Any, confirm: Callable[[str], str] = input) -> Dict[str, Any]:
    """Rulează sarcina (sau `memorize` dacă `goal` e None). Întoarce raportul (scris și în `args.out`)."""
    from brain.adapters.arm.team_arm import TeamArm
    from brain.perception.pipeline import VisionPipeline

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    stop = StopFlag()
    clock = SystemClock()
    journal = DurationJournal(out / "timings.jsonl", simulated=bool(args.service_sim))
    task = "memorize" if goal is None else args.task
    session = None
    if args.execute:
        arm = TeamArm(config.system.arm.team_url, timeout_s=config.system.arm.timeout_s)
        session = ArmSession(arm, clock, journal, MODE_SIMULATION if args.service_sim else MODE_HARDWARE, stop, task=task)
        status = session.connect()
        config = check_sites(config, status, need_zones=task == "sort")
    hub = server = None
    if args.preview:
        from brain.adapters.preview_server import PreviewHub, PreviewServer, local_addresses

        settings = config.system.preview
        hub = PreviewHub(config.system.planning.active_locations[0])
        server = PreviewServer(hub, settings.host, settings.port if args.port is None else args.port).start()
        hosts = local_addresses() if settings.host == "0.0.0.0" else [settings.host]
        print("Previzualizare: {}".format("  ".join("http://{}:{}/".format(h, server.port) for h in hosts)))
    observer: Optional[LocationObserver] = None
    camera = build_camera(args, lambda: observer.current_pose if observer is not None else None, config)
    perception = CameraPerception(camera, VisionPipeline(config.system, config.colors), config.system.vision.frames_to_aggregate)
    observer = LocationObserver(config.system, perception, clock, journal, session, hub, move_arm=bool(args.execute), task=task)
    store = SceneStore(out / "scene_state.json")
    initial = SceneStore(args.state).load() if args.state else None
    try:
        if goal is None:
            return _memorize(config, observer, args, out)
        if args.execute:
            answer = confirm("Brațul REAL se va mișca ({}). Mâna pe alimentare. Scrieți '{}' pentru a continua: ".format(
                goal.describe(), CONFIRM_WORD))
            if answer.strip().lower() != CONFIRM_WORD:
                raise SafetyRejected("execuția nu a fost confirmată")
            stop.install()
        planner = Planner(config.system, config.sort_map,
                          CostModel.from_calib_file(config.system.planning.cost, config.system.arm.calib_json))
        budget = TimeBudget(clock, config.system.timing.task_limit_s, config.system.timing.reserve_s)
        runner = TaskRunner(goal, config, planner, observer, clock, budget, session, store, initial_state=initial,
                            execute=bool(args.execute), task=task)
        result = runner.run()
        report = result.to_dict()
        report.update({"execute": bool(args.execute), "service_sim": bool(args.service_sim), "camera": args.camera})
        (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        return report
    finally:
        stop.restore()
        release = getattr(camera, "release", None)
        if release is not None:
            release()
        if server is not None:
            server.stop()


def _memorize(config: AppConfig, observer: LocationObserver, args: Any, out: Path) -> Dict[str, Any]:
    """S1, faza 1: observă șablonul și salvează aranjamentul ca țintă (YAML)."""
    location = args.location or config.system.planning.active_locations[0]
    observation = observer.observe(location)
    goal = target_from_location(location_from_observation(observation))
    target_path = Path(args.target_out or out / "target.yaml")
    target_path.parent.mkdir(parents=True, exist_ok=True)
    target_path.write_text(yaml.safe_dump(reproduce_goal_to_dict(goal), allow_unicode=True, sort_keys=False), encoding="utf-8")
    report = {"task": "memorize", "location": location, "target_file": str(target_path), "goal": goal.describe()}
    (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return report


def print_report(report: Dict[str, Any]) -> None:
    if report.get("task") == "memorize":
        print("Aranjament memorat: {} → {}".format(report["goal"], report["target_file"]))
        return
    print("Sarcina {} ({}): stare finală {}, obiectiv {}".format(
        report["task"], report["goal"], report["final_state"], "ÎNDEPLINIT" if report["goal_reached"] else "neîndeplinit"))
    plan = report["first_plan"] if not report.get("moves") else report["last_plan"]
    for line in plan:
        print("  " + line)
    for move in report.get("moves", []):
        print("  #{:<2} {:<55} {}{}".format(move["index"], move["move"], move["outcome"],
                                           " ({})".format(move["detail"]) if move["detail"] else ""))
    if report.get("error"):
        print("  EROARE: {}".format(report["error"]))
    for text in report.get("warnings", []):
        print("  ATENȚIE: {}".format(text))
    print("  timp: {:.1f} s, mutări: {}, observări: {}, comenzi braț: {}".format(
        report["elapsed_s"], report["moves_attempted"], report["observations"], report["arm_commands"]))
