"""`python -m brain` — comenzile Etapei 0 (viziune offline, memorie, selftest).

Coduri de ieșire: 0 = succes; 1 = selftest picat; 2 = eroare de configurare sau de intrare;
3 = observație neacționabilă (grila sau imaginea nu permit decizii sigure).
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
from brain.domain.errors import BrainError, ConfigError, PerceptionInputError, SceneStateError, SceneStateNotFound
from brain.domain.observation import GridObservation
from brain.domain.scene_state import SceneState, assign_unique_color_ids, location_from_observation, utc_now_iso
from brain.logging_setup import configure_logging, log_event

EXIT_OK = 0
EXIT_SELFTEST_FAILED = 1
EXIT_INPUT_ERROR = 2
EXIT_NOT_ACTIONABLE = 3

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
    print("  locații: {} (cell_mm NECALIBRAT până la măsurare)".format(", ".join(
        "{}={:.0f} mm".format(n, l.cell_mm) for n, l in config.system.locations.items())))
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
            return _cmd_sim_generate(config, args)
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
    except BrainError as exc:
        print("Eroare: {}".format(exc), file=sys.stderr)
        return EXIT_INPUT_ERROR
    return EXIT_INPUT_ERROR
