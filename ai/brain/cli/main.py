"""`python -m brain` — viziune offline, memorie, selftest, simularea sarcinilor și rularea pe braț.

Coduri de ieșire: 0 = succes; 1 = selftest picat / obiectiv neîndeplinit; 2 = eroare de configurare sau de
intrare; 3 = observație neacționabilă (grila sau imaginea nu permit decizii sigure); 4 = execuție oprită
(eroare de braț, oprire cerută sau refuz de siguranță).
"""

from __future__ import annotations

import argparse
import glob
import json
import logging
import sys
from pathlib import Path
from typing import List, Optional, Sequence

from brain.configuration.loader import AppConfig, default_config_dir, load_app_config
from brain.domain.cube import Occupancy
from brain.domain.errors import (
    ArmError,
    BrainError,
    ConfigError,
    PerceptionInputError,
    SafetyRejected,
    SceneStateError,
    SceneStateNotFound,
)
from brain.domain.observation import GridObservation
from brain.domain.scene_state import SceneState, assign_unique_color_ids, location_from_observation, utc_now_iso
from brain.logging_setup import configure_logging, log_event

EXIT_OK = 0
EXIT_SELFTEST_FAILED = 1
EXIT_INPUT_ERROR = 2
EXIT_NOT_ACTIONABLE = 3
EXIT_STOPPED = 4

logger = logging.getLogger("brain.cli")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m brain", description="Subsistemul Brain — DOFBOT CRI 2026")
    parser.add_argument("--config-dir", default=None, help="directorul cu system.yaml și colors.yaml")
    parser.add_argument("--log-file", default=None, help="fișier JSON Lines pentru loguri (nivel INFO)")
    parser.add_argument("--log-level", default="WARNING", help="nivelul logurilor în consolă")
    sub = parser.add_subparsers(dest="command", required=True)

    config = sub.add_parser("config", help="configurare").add_subparsers(dest="action", required=True)
    config.add_parser("validate", help="validează fișierele de configurare")

    sim = sub.add_parser("sim", help="simulare").add_subparsers(dest="action", required=True)
    gen = sim.add_parser("generate", help="generează scene sintetice (PNG + adevăr de referință JSON)")
    gen.add_argument("--seed", type=int, default=42)
    gen.add_argument("--count", type=int, default=1)
    gen.add_argument("--kind", default="positive")
    gen.add_argument("--difficulty", default="nominal", choices=("nominal", "hard"))
    gen.add_argument("--frames", type=int, default=5)
    gen.add_argument("--aruco", action="store_true", help="desenează markerii ArUco din config")
    gen.add_argument("--location", default=None, help="locația din config (dimensiunile grilei)")
    gen.add_argument("--out", default="out/synthetic")

    vision = sub.add_parser("vision", help="viziune").add_subparsers(dest="action", required=True)
    analyze = vision.add_parser("analyze", help="analizează cadre statice ale unei locații")
    analyze.add_argument("--frames", nargs="+", required=True, help="fișiere sau șabloane glob (ex. 'scene/frame_*.png')")
    analyze.add_argument("--location", required=True)
    analyze.add_argument("--method", choices=("lines", "aruco"), default=None, help="suprascrie vision.detection_method")
    analyze.add_argument("--out", default="out/analysis")
    analyze.add_argument("--state-out", default=None, help="salvează observația în SceneState (JSON)")
    analyze.add_argument("--overwrite", action="store_true", help="permite actualizarea unui fișier de stare existent")
    analyze.add_argument("--replace-invalid", action="store_true", help="permite înlocuirea unui fișier de stare invalid")

    camera = sub.add_parser("camera", help="camera (previzualizare, captură)").add_subparsers(dest="action", required=True)
    preview = camera.add_parser("preview", help="flux live + analiză adnotată în browser: http://<ip>:<port>/")
    preview.add_argument("--location", default=None, help="șablonul analizat (implicit medium)")
    preview.add_argument("--index", type=int, default=0, help="indexul camerei USB")
    preview.add_argument("--width", type=int, default=640)
    preview.add_argument("--height", type=int, default=480)
    preview.add_argument("--frames", nargs="+", default=None, help="fișiere în locul camerei, reluate în buclă (test)")
    preview.add_argument("--host", default=None, help="suprascrie preview.host")
    preview.add_argument("--port", type=int, default=None, help="suprascrie preview.port")
    preview.add_argument("--no-analyze", action="store_true", help="doar fluxul live, fără analiză")
    preview.add_argument("--duration", type=float, default=None, help="oprire după N secunde (implicit: Ctrl+C)")
    capture = camera.add_parser("capture", help="salvează cadre de la camera USB (PNG)")
    capture.add_argument("--index", type=int, default=0)
    capture.add_argument("--width", type=int, default=640)
    capture.add_argument("--height", type=int, default=480)
    capture.add_argument("--count", type=int, default=5)
    capture.add_argument("--out", default="out/capture")

    simrun = sim.add_parser("run", help="o sarcină cap-coadă în simulare (MockArm + lume simulată + FSM)")
    simrun.add_argument("--task", required=True, choices=("tower", "sort", "reproduce", "all"))
    simrun.add_argument("--order", default=None, help="turnul, de jos în sus: red,yellow,green,blue,red")
    simrun.add_argument("--target", default=None, help="celula turnului: medium:r1c1 (implicit centrul primului șablon activ)")
    simrun.add_argument("--target-file", default=None, help="ținta S1 (YAML/JSON: rows sau cells)")
    simrun.add_argument("--target-location", default=None, help="unde se construiește aranjamentul S1")
    simrun.add_argument("--example", action="store_true", help="exemplul din regulament (S1 sau S2)")
    simrun.add_argument("--seed", type=int, default=1)
    simrun.add_argument("--random", type=int, default=0, metavar="N", help="N scenarii aleatoare (seed, seed+1, ...)")
    simrun.add_argument("--faults", default=None, help="erori injectate: pick:grasp_fail@1,place:drop%%0.1,...")
    simrun.add_argument("--perception", choices=("oracle", "render"), default="render",
                        help="render = cameră sintetică + viziunea reală; oracle = adevărul simulat (rapid)")
    simrun.add_argument("--proximity", action="store_true", help="senzorul de proximitate simulat (HC-SR04)")
    simrun.add_argument("--buffers", type=int, default=None, metavar="N",
                        help="doar în simulare: N puncte tampon buffer_1..N (pe robot trebuie calibrate)")
    simrun.add_argument("--gif", default=None, help="animația schematică (GIF cu Pillow, altfel PNG-uri)")
    simrun.add_argument("--dry-run", action="store_true", help="doar observă și planifică")
    simrun.add_argument("--service-url", default=None,
                        help="serviciul colegilor pornit cu --sim (TeeArm: contractul real + lumea simulată)")
    simrun.add_argument("--out", default="out/sim_run")

    run = sub.add_parser("run", help="sarcina pe serviciul de mișcare (fără --execute: doar observă și planifică)")
    run.add_argument("--task", required=True, choices=("tower", "sort", "reproduce", "memorize"))
    run.add_argument("--arm", choices=("team",), default="team")
    run.add_argument("--camera", choices=("usb", "files"), default="usb")
    run.add_argument("--frames-dir", default=None, help="--camera files: DIR/<locație>/*.png sau DIR/*.png")
    run.add_argument("--camera-index", type=int, default=0)
    run.add_argument("--width", type=int, default=640)
    run.add_argument("--height", type=int, default=480)
    run.add_argument("--order", default=None)
    run.add_argument("--target", default=None)
    run.add_argument("--target-file", default=None)
    run.add_argument("--target-location", default=None)
    run.add_argument("--target-out", default=None, help="memorize: unde se scrie ținta S1 (implicit OUT/target.yaml)")
    run.add_argument("--location", default=None, help="memorize: șablonul observat")
    run.add_argument("--state", default=None, help="memoria de pornire (SceneState JSON)")
    run.add_argument("--execute", action="store_true", help="MIȘCĂ brațul (cu confirmare)")
    run.add_argument("--service-sim", action="store_true", help="acceptă serviciul pornit cu --sim (teste)")
    run.add_argument("--preview", action="store_true", help="previzualizarea camerei în browser în timpul sarcinii")
    run.add_argument("--port", type=int, default=None)
    run.add_argument("--out", default=None, help="implicit out/run-<data>")

    report = sub.add_parser("report", help="rapoarte").add_subparsers(dest="action", required=True)
    timings = report.add_parser("timings", help="duratele comenzilor → valori propuse pentru secțiunea timing")
    timings.add_argument("--journal", nargs="+", required=True, help="fișiere timings.jsonl (sau șabloane glob)")

    calib = sub.add_parser("calib", help="calibrări din imagini").add_subparsers(dest="action", required=True)
    corners = calib.add_parser("corners", help="colțurile grilei în imagine (expected_corners_px)")
    corners.add_argument("--location", required=True)
    corners.add_argument("--frames", nargs="+", required=True)
    corners.add_argument("--write", action="store_true", help="scrie valoarea în system.yaml (păstrând comentariile)")

    selftest = sub.add_parser("selftest", help="teste reproductibile").add_subparsers(dest="action", required=True)
    stage0 = selftest.add_parser("stage0", help="selftest-ul viziunii pe scene sintetice")
    stage0.add_argument("--seed", type=int, default=42)
    stage0.add_argument("--scenes", type=int, default=200)
    stage0.add_argument("--difficulty", default="nominal", choices=("nominal", "hard"))
    stage0.add_argument("--frames", type=int, default=5)
    stage0.add_argument("--out", default="out/selftest")
    stage0.add_argument("--save-failures", type=int, default=10)
    stage0.add_argument("--location", default=None, help="șablonul folosit (implicit medium, dacă există)")

    memory = sub.add_parser("memory", help="memorie persistentă").add_subparsers(dest="action", required=True)
    show = memory.add_parser("show", help="afișează și validează fișierul de stare")
    show.add_argument("--path", default=None, help="implicit: memory.path din system.yaml")
    return parser


def _load_config(args: argparse.Namespace) -> AppConfig:
    return load_app_config(args.config_dir if args.config_dir is not None else default_config_dir())


def _default_location(config: AppConfig) -> str:
    return "medium" if "medium" in config.system.locations else next(iter(config.system.locations))


def _expand(patterns: Sequence[str]) -> List[Path]:
    files: List[Path] = []
    for pattern in patterns:
        matches = sorted(glob.glob(pattern))
        if not matches and not any(ch in pattern for ch in "*?["):
            matches = [pattern]
        files.extend(Path(m) for m in matches)
    if not files:
        raise PerceptionInputError("niciun cadru găsit pentru: {}".format(" ".join(patterns)))
    return files


def _print_observation(observation: GridObservation) -> None:
    status = "ACȚIONABIL" if observation.actionable else "NEACȚIONABIL"
    print("{}: {} ({:.0f} ms, metodă {})".format(observation.location, status, observation.processing_ms, observation.geometry.method))
    if observation.reasons:
        print("  motive: {}".format(", ".join(observation.reasons)))
    for cell in observation.cells:
        if cell.occupancy is Occupancy.OCCUPIED:
            yaw = "yaw ?" if cell.yaw_deg is None else "yaw {:+.1f}°".format(cell.yaw_deg)
            text = "{:<7} {:<5} {} (ocupare {:.2f}, culoare {:.2f})".format(
                cell.color.value, cell.top_face.value, yaw, cell.occupancy_confidence, cell.color_confidence
            )
        else:
            text = "{} ({:.2f})".format(cell.occupancy.value, cell.occupancy_confidence)
        extra = "  [{}]".format(", ".join(cell.reasons)) if cell.reasons else ""
        print("  {}: {}{}".format(cell.coord, text, extra))


def _cmd_config_validate(config: AppConfig) -> int:
    print("OK: {}/system.yaml, {}/colors.yaml".format(config.config_dir, config.config_dir))
    if not config.system.cube.calibrated:
        print("  ATENȚIE: cube.calibrated=false — latura cubului e nominală, nemăsurată")
    print("  locații: {}; active la probă: {}".format(", ".join(
        "{}={:.0f} mm".format(n, l.cell_mm) for n, l in config.system.locations.items()),
        ", ".join(config.system.planning.active_locations)))
    if config.system.planning.buffer_points:
        print("  puncte tampon (de calibrat în serviciu): {}".format(", ".join(config.system.planning.buffer_points)))
    if not config.sort_map.calibrated:
        print("  ATENȚIE: sort_map.calibrated=false — zonele hărții nu sunt confirmate")
    if not config.colors.calibrated:
        print("  ATENȚIE: colors.calibrated=false — pragurile HSV nu sunt calibrate pe camera reală")
    return EXIT_OK


def _cmd_sim_generate(config: AppConfig, args: argparse.Namespace) -> int:
    from brain.perception.image_io import write_image
    from brain.simulation.scene_gen import generate_scene

    location = config.system.location(args.location or _default_location(config))
    aruco = config.system.vision.aruco if args.aruco or args.kind == "missing_marker" else None
    if (args.aruco or args.kind == "missing_marker") and aruco is None:
        raise PerceptionInputError("secțiunea vision.aruco lipsește din configurație")
    out = Path(args.out)
    for index in range(args.count):
        seed = args.seed + index
        scene = generate_scene(
            seed, config.system.grid_geometry(location.name), kind=args.kind, difficulty=args.difficulty, frames=args.frames,
            rows=location.rows, cols=location.cols, aruco=aruco,
        )
        folder = out / "scene_{}".format(seed)
        for k, frame in enumerate(scene.frames):
            write_image(folder / "frame_{}.png".format(k), frame)
        (folder / "truth.json").write_text(
            json.dumps(scene.truth.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print("{}: {} cadre, {} cuburi, tip {}".format(folder, len(scene.frames), len(scene.truth.cubes), args.kind))
    return EXIT_OK


def _cmd_vision_analyze(config: AppConfig, args: argparse.Namespace) -> int:
    from brain.memory.codec import observation_to_dict
    from brain.memory.store import SceneStore
    from brain.perception.annotate import render_annotation
    from brain.perception.image_io import read_image, write_image
    from brain.perception.pipeline import VisionPipeline

    files = _expand(args.frames)
    frames = [read_image(path) for path in files]
    pipeline = VisionPipeline(config.system, config.colors, method=args.method)
    result = pipeline.analyze(frames, args.location)
    observation = result.observation
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "observation.json").write_text(
        json.dumps(observation_to_dict(observation), ensure_ascii=False, indent=2), encoding="utf-8"
    )
    write_image(out / "annotated.jpg", render_annotation(result))
    log_event(
        logger, "vision_analyze", action="analyze", state="OBSERVE", result="actionable" if observation.actionable else "rejected",
        duration_ms=observation.processing_ms, location=observation.location, reasons=list(observation.reasons),
    )
    _print_observation(observation)
    print("  rezultate: {} , {}".format(out / "observation.json", out / "annotated.jpg"))
    if not observation.actionable:
        return EXIT_NOT_ACTIONABLE
    if args.state_out:
        store = SceneStore(args.state_out)
        if store.exists() and args.overwrite:
            try:
                state = store.load()
            except SceneStateError:
                if not args.replace_invalid:
                    raise
                state = SceneState.new("scene-{}".format(utc_now_iso()))
        else:
            state = SceneState.new("scene-{}".format(utc_now_iso()))
        state = assign_unique_color_ids(state.with_location(location_from_observation(observation)))
        store.save(state, overwrite=args.overwrite, replace_invalid=args.replace_invalid)
        print("  stare salvată: {}".format(store.path))
    return EXIT_OK


def _cmd_camera_preview(config: AppConfig, args: argparse.Namespace) -> int:
    import time

    from brain.adapters.camera import LoopingFileCamera, UsbCamera
    from brain.adapters.preview_server import PreviewHub, PreviewServer, local_addresses
    from brain.cli.preview import PreviewSession
    from brain.perception.pipeline import VisionPipeline

    settings = config.system.preview
    location = config.system.location(args.location or _default_location(config)).name
    if args.frames:
        source = LoopingFileCamera([str(path) for path in _expand(args.frames)])
    else:
        source = UsbCamera(args.index, args.width, args.height, flush=0)
    hub = PreviewHub(location)
    host = args.host or settings.host
    try:
        server = PreviewServer(hub, host, settings.port if args.port is None else args.port).start()
    except OSError as exc:
        raise PerceptionInputError("serverul de previzualizare nu poate porni pe {}:{}: {}".format(
            host, settings.port if args.port is None else args.port, exc)) from None
    vision = config.system.vision
    session = PreviewSession(
        source, hub, settings,
        pipeline=None if args.no_analyze else VisionPipeline(config.system, config.colors),
        frames_to_aggregate=vision.frames_to_aggregate, min_valid_frames=vision.min_valid_frames,
        save_dir=Path(settings.save_dir),
    ).start()
    hosts = local_addresses() if host == "0.0.0.0" else [host]
    print("Previzualizare {} ({}): {}".format(
        location, "fișiere" if args.frames else "camera {}".format(args.index),
        "  ".join("http://{}:{}/".format(h, server.port) for h in hosts)))
    print("  imagini: http://<ip>:{0}/snapshot.jpg  http://<ip>:{0}/annotated.jpg ; pe disc: {1}".format(
        server.port, settings.save_dir))
    print("  oprire: Ctrl+C")
    log_event(logger, "preview_start", action="preview", state="OBSERVE", result="started", location=location, port=server.port)
    started = time.monotonic()
    try:
        while args.duration is None or time.monotonic() - started < args.duration:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        session.stop()
        server.stop()
        release = getattr(source, "release", None)
        if release is not None:
            release()
    print("  oprit: {} cadre, {} analize{}".format(
        hub.latest()[2]["frames"], session.analyses, "; ultima eroare: {}".format(session.last_error) if session.last_error else ""))
    return EXIT_OK


def _cmd_camera_capture(args: argparse.Namespace) -> int:
    from brain.adapters.camera import UsbCamera
    from brain.perception.image_io import write_image

    camera = UsbCamera(args.index, args.width, args.height)
    try:
        frames = camera.grab(args.count)
    finally:
        camera.release()
    out = Path(args.out)
    for k, frame in enumerate(frames):
        write_image(out / "frame_{}.png".format(k), frame)
    print("{} cadre {}x{} salvate în {}".format(len(frames), frames[0].shape[1], frames[0].shape[0], out))
    return EXIT_OK


def _cmd_selftest_stage0(config: AppConfig, args: argparse.Namespace) -> int:
    from brain.cli.selftest import run_stage0_selftest

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    report = run_stage0_selftest(
        config, seed=args.seed, scenes=args.scenes, difficulty=args.difficulty, frames=args.frames,
        out_dir=out, save_failures=args.save_failures, location=args.location or _default_location(config),
    )
    (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print("Selftest Etapa 0 — seed {}, {} scene, dificultate {} ({} s)".format(
        report["seed"], report["scenes"], report["difficulty"], report["duration_s"]))
    for name, value in report["metrics"].items():
        print("  {:<28} {}".format(name, value))
    for name, ok in report["checks"].items():
        print("  [{}] {}".format("OK " if ok else "EȘEC", name))
    for failure in report["failures"][:20]:
        print("  - {}".format(failure))
    print("  raport: {}".format(out / "report.json"))
    print("  {}".format(report["note"]))
    return EXIT_OK if report["passed"] else EXIT_SELFTEST_FAILED


def _goal_from_args(config: AppConfig, args: argparse.Namespace):
    from brain.planning.goals import (
        GoalError, SortGoal, TowerGoal, load_reproduce_goal, parse_order, parse_slot, reproduce_goal_from_dict,
    )

    system = config.system
    first = system.planning.active_locations[0]
    if args.task == "tower":
        if not args.order:
            raise GoalError("--task tower cere --order (ex. red,yellow,green,blue,red)")
        target = parse_slot(args.target) if args.target else parse_slot("{}:r1c1".format(first))
        if target.location not in system.planning.active_locations or target.cell is None:
            raise GoalError("ținta turnului trebuie să fie o celulă a unui șablon activ ({}), nu {}".format(
                ", ".join(system.planning.active_locations), args.target))
        return TowerGoal(parse_order(args.order), target)
    if args.task == "sort":
        return SortGoal()
    if args.task == "reproduce":
        if not args.target_file:
            raise GoalError("--task reproduce cere --target-file (ținta memorată: `brain run --task memorize`)")
        goal = load_reproduce_goal(args.target_file, args.target_location)
        if goal.location not in system.planning.active_locations:
            raise GoalError("locația țintă {} nu e activă ({})".format(goal.location, ", ".join(system.planning.active_locations)))
        return goal
    return None


def _cmd_sim_run(config: AppConfig, args: argparse.Namespace) -> int:
    import dataclasses

    from brain.cli.sim_run import SimOptions, parse_faults, run_scenario, summarize_batch
    from brain.planning.goals import parse_order, parse_slot
    from brain.simulation import scenario as sc

    if args.buffers is not None:
        planning = dataclasses.replace(config.system.planning,
                                       buffer_points=tuple("buffer_{}".format(i + 1) for i in range(args.buffers)))
        config = dataclasses.replace(config, system=dataclasses.replace(config.system, planning=planning))
    system, sort_map = config.system, config.sort_map
    out = Path(args.out)
    options = SimOptions(perception=args.perception, faults=parse_faults(args.faults), proximity=args.proximity,
                         dry_run=args.dry_run, service_url=args.service_url)
    if args.random:
        tasks = ("tower", "reproduce", "sort") if args.task == "all" else (args.task,)
        reports = []
        for task in tasks:
            for index in range(args.random):
                scenario = sc.random_scenario(task, system, sort_map, args.seed + index)
                reports.append(run_scenario(config, scenario, dataclasses.replace(
                    options, out_dir=out / scenario.name)))
        summary = summarize_batch(reports)
        out.mkdir(parents=True, exist_ok=True)
        (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        print("Simulare: {} scenarii aleatoare, seed {}..{}, percepție {} (timpi SIMULAȚI)".format(
            len(reports), args.seed, args.seed + args.random - 1, args.perception))
        for task, item in summary.items():
            print("  {:<9} reușite {}/{} ({:.0%}; dintre cele fezabile {}), timp simulat median {} s, max {} s, peste limită {}".format(
                task, item["success"], item["scenarios"], item["success_rate"],
                "-" if item["success_rate_feasible"] is None else "{:.0%}".format(item["success_rate_feasible"]),
                item["simulated_time_median_s"], item["simulated_time_max_s"], item["over_time_limit"]))
            for failure in item["failures"][:8]:
                print("    - {}".format(failure))
        print("  rezumat: {}".format(out / "summary.json"))
        return EXIT_OK
    if args.task == "all":
        raise PerceptionInputError("--task all merge doar cu --random N")
    if args.example:
        if args.task == "tower":
            scenario = sc.example_s2(system, sort_map, args.seed, target=args.target or "{}:r1c1".format(
                system.planning.active_locations[0]))
        elif args.task == "reproduce":
            scenario = sc.example_s1(system, sort_map, args.seed, args.target_location)
        else:
            scenario = sc.random_scenario("sort", system, sort_map, args.seed)
    elif args.task == "tower":
        goal = _goal_from_args(config, args)
        scenario = sc.tower_scenario(system, sort_map, goal, args.seed)
    elif args.task == "reproduce":
        scenario = sc.reproduce_scenario(system, sort_map, _goal_from_args(config, args), args.seed)
    else:
        scenario = sc.random_scenario("sort", system, sort_map, args.seed)
    options = dataclasses.replace(options, out_dir=out, gif=Path(args.gif) if args.gif else None)
    report = run_scenario(config, scenario, options)
    print("Scenariu {} (seed {}): {}".format(scenario.name, scenario.seed, scenario.note))
    for line in sc.describe_world(scenario.world) if args.dry_run else []:
        print("  lume: {}".format(line))
    from brain.cli.run import print_report

    print_report(report)
    print("  REZULTAT (adevărul simulat): {}{}".format(
        "REUȘIT" if report["success"] else ("doar plan" if args.dry_run else "NEREUȘIT"),
        "" if report["success"] or args.dry_run else ": " + "; ".join(report["truth_problems"][:5])))
    print("  timp simulat {} s (limită {} s); raport: {}{}".format(
        report["simulated_time_s"], system.timing.task_limit_s, out / "report.json",
        "; animație: {}".format(report["gif"]) if report.get("gif") else ""))
    if args.dry_run:
        return EXIT_OK
    return EXIT_OK if report["success"] else EXIT_SELFTEST_FAILED


def _cmd_run(config: AppConfig, args: argparse.Namespace) -> int:
    import datetime

    from brain.cli.run import print_report, run_task

    if args.out is None:
        args.out = "out/run-{}".format(datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
    goal = _goal_from_args(config, args)
    configure_logging(args.log_level, str(Path(args.out) / "log.jsonl"), mode="hardware" if args.execute else "planning")
    report = run_task(config, goal, args)
    print_report(report)
    print("  rezultate: {}".format(args.out))
    if report.get("task") == "memorize":
        return EXIT_OK
    if report["final_state"] == "ERROR":
        return EXIT_STOPPED
    return EXIT_OK if report["goal_reached"] or not args.execute else EXIT_SELFTEST_FAILED


def _cmd_report_timings(args: argparse.Namespace) -> int:
    from brain.cli.tools import load_journal, summarize_timings

    summary = summarize_timings(load_journal(_expand(args.journal)))
    if True in summary["simulated"]:
        print("ATENȚIE: jurnalul conține durate SIMULATE, nu măsurători")
    print("{:<14} {:>4} {:>9} {:>7} {:>7}".format("comandă", "n", "mediană", "p90", "max"))
    for method, item in summary["methods"].items():
        print("{:<14} {:>4} {:>8.2f}s {:>6.2f}s {:>6.2f}s".format(method, item["n"], item["median_s"], item["p90_s"], item["max_s"]))
    if summary["proposal"]:
        print("Propunere pentru config/system.yaml (secțiunea timing, p90):")
        for key, value in summary["proposal"].items():
            print("  {}: {}".format(key, value))
    return EXIT_OK


def _cmd_calib_corners(config: AppConfig, args: argparse.Namespace) -> int:
    from brain.cli.tools import corners_from_frames, write_expected_corners
    from brain.perception.image_io import read_image

    config.system.location(args.location)
    frames = [read_image(path) for path in _expand(args.frames)]
    corners = corners_from_frames(config, args.location, frames)
    print("{}: expected_corners_px: [{}]".format(args.location, ", ".join("[{:.1f}, {:.1f}]".format(x, y) for x, y in corners)))
    if args.write:
        target = Path(config.config_dir) / "system.yaml"
        write_expected_corners(target, args.location, corners)
        print("  scris în {}".format(target))
    return EXIT_OK


def _cmd_memory_show(config: AppConfig, args: argparse.Namespace) -> int:
    from brain.memory.codec import scene_to_dict
    from brain.memory.store import SceneStore

    store = SceneStore(args.path or config.system.memory.path)
    state = store.load()
    print(json.dumps(scene_to_dict(state), ensure_ascii=False, indent=2))
    return EXIT_OK


def main(argv: Optional[Sequence[str]] = None) -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")  # type: ignore[union-attr]  # cp1252 (Windows) nu are ă/ș/ț
        except (AttributeError, ValueError):
            pass
    args = _build_parser().parse_args(argv)
    configure_logging(args.log_level, args.log_file, mode="offline")
    try:
        config = _load_config(args)
        if args.command == "config":
            return _cmd_config_validate(config)
        if args.command == "sim":
            if args.action == "run":
                return _cmd_sim_run(config, args)
            return _cmd_sim_generate(config, args)
        if args.command == "run":
            return _cmd_run(config, args)
        if args.command == "report":
            return _cmd_report_timings(args)
        if args.command == "calib":
            return _cmd_calib_corners(config, args)
        if args.command == "vision":
            return _cmd_vision_analyze(config, args)
        if args.command == "camera":
            if args.action == "capture":
                return _cmd_camera_capture(args)
            return _cmd_camera_preview(config, args)
        if args.command == "selftest":
            return _cmd_selftest_stage0(config, args)
        if args.command == "memory":
            return _cmd_memory_show(config, args)
    except ConfigError as exc:
        print("Eroare de configurare: {}".format(exc), file=sys.stderr)
        return EXIT_INPUT_ERROR
    except SceneStateNotFound as exc:
        print("Memorie: {}".format(exc), file=sys.stderr)
        return EXIT_INPUT_ERROR
    except (PerceptionInputError, SceneStateError, ValueError, KeyError) as exc:
        print("Eroare: {}".format(exc), file=sys.stderr)
        return EXIT_INPUT_ERROR
    except (SafetyRejected, ArmError) as exc:
        print("Oprit: {}".format(exc), file=sys.stderr)
        return EXIT_STOPPED
    except BrainError as exc:
        print("Eroare: {}".format(exc), file=sys.stderr)
        return EXIT_INPUT_ERROR
    return EXIT_INPUT_ERROR
