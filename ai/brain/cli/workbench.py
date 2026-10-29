"""Masa de lucru din pagina de previzualizare: ținta S1 (din poză sau editată), planul și execuția.

- Ținta: o poză încărcată din pagină (telefon) e analizată cu `TopViewPipeline`; rezultatul apare ca un caroiaj
  editabil. Ținta confirmată se salvează în `<save_dir>/target/target.yaml`, în formatul citit de
  `brain run --task reproduce --target-file`, plus poza originală.
- Planul: starea curentă = ultima observație a previzualizării (aceeași îmbinare ca în `brain run`, printr-un
  `TaskRunner` fără execuție); ținta = cea confirmată. Nu mișcă brațul.
- Serviciul de mișcare: `status` citit periodic (doar citire), oprit cât timp rulează o execuție (serviciul
  execută cererile la rând: un `status` ar aștepta după `pick`).
- Execuția: `WebExecution` (doar cu `preview.allow_execute: true` și codul din consolă).
"""

from __future__ import annotations

import datetime
import threading
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import yaml

from brain.adapters.preview_server import ApiError
from brain.configuration.loader import AppConfig
from brain.domain.cube import Color
from brain.domain.errors import BrainError, PerceptionInputError
from brain.domain.grid import GridCoord, iter_coords
from brain.domain.plan import Move, SlotRef
from brain.planning.goals import CellTarget, GoalError, ReproduceGoal, color_name, load_reproduce_goal, reproduce_goal_to_dict

ALLOWED_VALUES = ("red", "yellow", "green", "blue", "empty")
SERVICE_POLL_S = 3.0
SERVICE_TIMEOUT_S = 2.0
TARGET_FILE = "target.yaml"
_GRASP_TEXT = {"offset": ", cu offset spre spațiul liber", "risky": ", RISC: degetele încap pe o singură axă",
               "blocked": ", RISC: degetele ating vecinii"}


def _now() -> str:
    return datetime.datetime.now().isoformat(timespec="seconds")


class ServiceMonitor:
    """`status` de la serviciul de mișcare, citit periodic într-un fir (doar citire, nicio mișcare)."""

    def __init__(self, config: AppConfig, reader: Optional[Callable[[], Dict[str, Any]]] = None,
                 period_s: float = SERVICE_POLL_S) -> None:
        self.config = config
        self.url = config.system.arm.team_url
        self.reader = reader or self._read_service
        self.period_s = period_s
        self.paused = threading.Event()
        self._stop = threading.Event()
        self._snapshot: Dict[str, Any] = {"ok": False, "error": "încă neverificat", "url": self.url}
        self._thread = threading.Thread(target=self._loop, name="service-monitor", daemon=True)

    def _read_service(self) -> Dict[str, Any]:
        from brain.adapters.arm.team_arm import TeamArm

        return TeamArm(self.url, timeout_s=SERVICE_TIMEOUT_S).status()

    def start(self) -> "ServiceMonitor":
        self.refresh()
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()

    def _loop(self) -> None:
        while not self._stop.wait(self.period_s):
            if not self.paused.is_set():
                self.refresh()

    def refresh(self) -> None:
        try:
            status = self.reader()
            snapshot = self._describe(status)
        except Exception as exc:  # serviciul oprit, alt port, răspuns ciudat: pagina doar afișează
            snapshot = {"ok": False, "url": self.url,
                        "error": "serviciul de mișcare nu răspunde la {} ({}). Pornit cu npm run server / npm run sim?".format(
                            self.url, exc)}
        snapshot["checked_at"] = _now()
        self._snapshot = snapshot

    def _describe(self, status: Dict[str, Any]) -> Dict[str, Any]:
        system = self.config.system
        sites = status.get("sites") or {}
        poses = list(status.get("poses") or [])
        kinds = {"grid": "caroiaj", "point": "punct"}
        missing = [name for name in system.planning.active_locations if sites.get(name) != "grid"]
        missing += [system.location(n).observe_pose or "" for n in system.planning.active_locations
                    if system.location(n).observe_pose not in poses]
        span = status.get("grip_range_mm")
        return {
            "ok": True, "url": self.url, "sim": bool(status.get("sim")), "poses": poses,
            "sites_text": ["{} ({})".format(name, kinds.get(str(kind), kind)) for name, kind in sorted(sites.items())],
            "missing": [m for m in missing if m],
            "buffers_missing": [b for b in system.planning.buffer_points if sites.get(b) != "point"],
            "grip_calibrated": status.get("grip_calibrated") is True,
            "grip_range_mm": span if isinstance(span, list) else None,
        }

    def snapshot(self) -> Dict[str, Any]:
        data = dict(self._snapshot)
        data["paused"] = self.paused.is_set()
        return data


class _FixedPerception:
    """Percepția „ultima observație”: planul pornește de la ce a văzut previzualizarea, fără captură nouă."""

    def __init__(self, observation: Any) -> None:
        self.observation = observation

    def perceive(self, location: str) -> Any:
        from brain.orchestration.observer import PerceptionResult

        if location != self.observation.location:
            raise PerceptionInputError("camera vede {}, nu {}".format(self.observation.location, location))
        return PerceptionResult(self.observation)


class Workbench:
    def __init__(
        self,
        config: AppConfig,
        hub: Any,
        preview: Any,
        save_dir: Path,
        arm_factory: Optional[Callable[[], Any]] = None,
        expected_mode: Optional[str] = None,
        clock_factory: Optional[Callable[[], Any]] = None,
        status_reader: Optional[Callable[[], Dict[str, Any]]] = None,
        token: Optional[str] = None,
        poll_service: bool = True,
    ) -> None:
        from brain.adapters.clock import SystemClock
        from brain.cli.web_execution import WebExecution
        from brain.ports.arm import MODE_HARDWARE

        self.config = config
        self.system = config.system
        self.hub = hub
        self.preview = preview
        self.target_dir = Path(save_dir) / "target"
        self.target_location = self.system.planning.active_locations[0]
        self._lock = threading.Lock()
        self._confirmed: Optional[ReproduceGoal] = None
        self._confirmed_info: Dict[str, Any] = {}
        self._detected: Optional[Dict[str, Any]] = None
        self._photo: Optional[bytes] = None
        self._photo_ext = ".jpg"
        self._annotated: Optional[bytes] = None
        self._photo_pipeline: Any = None
        self.monitor = ServiceMonitor(config, status_reader)
        self._poll = poll_service
        if arm_factory is None:
            from brain.adapters.arm.team_arm import TeamArm

            arm = self.system.arm
            arm_factory = lambda: TeamArm(arm.team_url, timeout_s=arm.timeout_s)  # noqa: E731
        self.execution = WebExecution(
            config, hub, preview, Path(save_dir) / "runs", self.system.preview.allow_execute, arm_factory,
            expected_mode or MODE_HARDWARE, clock_factory or SystemClock, token, on_running=self._on_running)
        self._load_saved_target()

    # ------------------------------------------------------------------ ciclul de viață
    @property
    def execute_enabled(self) -> bool:
        return self.execution.enabled

    @property
    def token(self) -> Optional[str]:
        return self.execution.token

    def start(self) -> "Workbench":
        if self._poll:
            self.monitor.start()
        else:
            self.monitor.refresh()
        return self

    def stop(self) -> None:
        self.monitor.stop()

    def _on_running(self, running: bool) -> None:
        if running:
            self.monitor.paused.set()
        else:
            self.monitor.paused.clear()
            self.monitor.refresh()

    # ------------------------------------------------------------------ starea pentru /status.json
    def status(self) -> Dict[str, Any]:
        gripper = self.system.gripper
        return {
            "location": self.hub.location,
            "active_locations": list(self.system.planning.active_locations),
            "target": {"location": self.target_location,
                       "confirmed": dict(self._confirmed_info) if self._confirmed is not None else None,
                       "detected": self._detected is not None},
            "service": self.monitor.snapshot(),
            "execution": self.execution.status(),
            "execute_enabled": self.execution.enabled,
            "execute_locked": self.execution.locked,
            "gripper": None if gripper is None else {
                "max_open_mm": gripper.max_open_mm, "finger_mm": gripper.finger_mm,
                "side_space_mm": round(gripper.side_space_mm, 1)},
        }

    # ------------------------------------------------------------------ ținta
    def _rows_of(self, goal: ReproduceGoal) -> List[List[str]]:
        loc = self.system.location(goal.location)
        rows = [["empty"] * loc.cols for _ in range(loc.rows)]
        for coord, target in goal.cells.items():
            rows[coord.row][coord.col] = "empty" if target.top is None else target.top.value
        return rows

    def get_target(self) -> Dict[str, Any]:
        loc = self.system.location(self.target_location)
        with self._lock:
            confirmed = None
            if self._confirmed is not None:
                confirmed = dict(self._confirmed_info, cells=self._rows_of(self._confirmed))
            return {"location": self.target_location, "rows": loc.rows, "cols": loc.cols, "confirmed": confirmed,
                    "detected": self._detected, "photo": self._annotated is not None}

    def target_image(self, kind: str) -> Optional[bytes]:
        if kind == "annotated":
            return self._annotated
        if kind == "photo":
            return self._photo if self._photo_ext == ".jpg" else self._annotated
        return None

    def upload_target_image(self, data: bytes) -> Dict[str, Any]:
        from brain.cli.preview import describe_reason, encode_jpeg
        from brain.perception.target_photo import TopViewPipeline, decode_image, detect_target, downscale

        if self.execution.running:
            raise ApiError(409, "o execuție e în curs: ținta nu se schimbă acum")
        try:
            image, original = downscale(decode_image(data))
        except PerceptionInputError as exc:
            raise ApiError(400, str(exc)) from None
        if self._photo_pipeline is None:
            self._photo_pipeline = TopViewPipeline(self.system, self.config.colors, self.target_location)
        try:
            detection = detect_target(self._photo_pipeline, image, self.target_location,
                                      self.system.planning.min_confidence, original)
        except PerceptionInputError as exc:
            raise ApiError(400, "poza nu poate fi analizată: {}".format(exc)) from None
        if detection.actionable and detection.uncertain:
            message = "Caroiajul e găsit; verificați celulele marcate ({}) și corectați-le cu click.".format(
                ", ".join(detection.uncertain))
        elif detection.actionable:
            message = "Caroiajul e găsit și toate celulele sunt sigure. Verificați orientarea, apoi confirmați."
        else:
            message = "Caroiajul NU e găsit în poză: completați ținta de mână sau încercați altă poză (de sus, toată foaia)."
        cells = [[{"value": c.value, "confidence": c.confidence, "uncertain": c.uncertain,
                   "note": "; ".join(describe_reason(r) for r in c.reasons)} for c in row] for row in detection.cells]
        result = {
            "location": detection.location, "rows": detection.rows, "cols": detection.cols, "cells": cells,
            "actionable": detection.actionable, "uncertain": detection.uncertain, "message": message,
            "reasons_text": [describe_reason(r) for r in detection.reasons],
            "image_size": list(detection.image_size),
            "scaled_from": list(detection.scaled_from) if detection.scaled_from else None, "uploaded_at": _now(),
        }
        with self._lock:
            self._detected = result
            self._photo = bytes(data)
            self._photo_ext = ".png" if data[:8] == b"\x89PNG\r\n\x1a\n" else (".jpg" if data[:3] == b"\xff\xd8\xff" else ".img")
            self._annotated = encode_jpeg(detection.annotated, self.system.preview.jpeg_quality)
        return result

    def set_target(self, payload: Any) -> Dict[str, Any]:
        """Ținta editată în pagină, validată strict: caroiajul complet, doar culori cunoscute sau „empty”."""
        if self.execution.running:
            raise ApiError(409, "o execuție e în curs: ținta nu se schimbă acum")
        if not isinstance(payload, dict):
            raise ApiError(400, "ținta trebuie să fie un obiect JSON")
        unknown = sorted(set(payload) - {"location", "rows", "from_photo"})
        if unknown:
            raise ApiError(400, "câmpuri necunoscute: {}".format(", ".join(unknown)))
        location = payload.get("location") or self.target_location
        if location not in self.system.planning.active_locations:
            raise ApiError(400, "locația {!r} nu e un șablon activ ({})".format(
                location, ", ".join(self.system.planning.active_locations)))
        loc = self.system.location(location)
        rows = payload.get("rows")
        if not isinstance(rows, list) or len(rows) != loc.rows or any(
                not isinstance(row, list) or len(row) != loc.cols for row in rows):
            raise ApiError(400, "'rows' trebuie să fie un caroiaj {}x{}".format(loc.rows, loc.cols))
        cells: Dict[GridCoord, CellTarget] = {}
        for coord in iter_coords(loc.rows, loc.cols):
            value = rows[coord.row][coord.col]
            if value not in ALLOWED_VALUES:
                raise ApiError(400, "r{}c{}: valoare invalidă {!r} (permise: {})".format(
                    coord.row, coord.col, value, ", ".join(ALLOWED_VALUES)))
            cells[coord] = CellTarget(None if value == "empty" else Color(value))
        from_photo = payload.get("from_photo", False)
        if not isinstance(from_photo, bool):
            raise ApiError(400, "'from_photo' trebuie să fie true sau false")
        goal = ReproduceGoal(location, cells)
        self._save_target(goal, from_photo and self._photo is not None)
        return self.get_target()

    def _save_target(self, goal: ReproduceGoal, with_photo: bool) -> None:
        self.target_dir.mkdir(parents=True, exist_ok=True)
        header = ("# Ținta S1 confirmată în pagina de previzualizare ({}).\n"
                  "# Execuție din linia de comandă: python -m brain run --task reproduce --target-file {}\n").format(
            _now(), (self.target_dir / TARGET_FILE).as_posix())
        text = header + yaml.safe_dump(reproduce_goal_to_dict(goal), allow_unicode=True, sort_keys=False)
        tmp = self.target_dir / (TARGET_FILE + ".tmp")
        tmp.write_text(text, encoding="utf-8")
        tmp.replace(self.target_dir / TARGET_FILE)
        for old in self.target_dir.glob("target_photo.*"):
            old.unlink()
        if with_photo and self._photo is not None:
            (self.target_dir / ("target_photo" + self._photo_ext)).write_bytes(self._photo)
            if self._annotated is not None:
                (self.target_dir / "target_detected.jpg").write_bytes(self._annotated)
        with self._lock:
            self._confirmed = goal
            self._confirmed_info = {"file": (self.target_dir / TARGET_FILE).as_posix(), "saved_at": _now(), "photo": with_photo}

    def _load_saved_target(self) -> None:
        """Ținta confirmată anterior (după o repornire), dacă e validă pentru un șablon activ."""
        path = self.target_dir / TARGET_FILE
        if not path.is_file():
            return
        try:
            goal = load_reproduce_goal(path)
        except GoalError:
            return
        if goal.location in self.system.planning.active_locations:
            self._confirmed = goal
            self.target_location = goal.location
            self._confirmed_info = {"file": path.as_posix(), "saved_at": datetime.datetime.fromtimestamp(
                path.stat().st_mtime).isoformat(timespec="seconds"), "photo": any(self.target_dir.glob("target_photo.*"))}

    # ------------------------------------------------------------------ planul
    def compute_plan(self) -> Dict[str, Any]:
        """Planul de la starea curentă (ultima observație) la ținta confirmată. Nu mișcă brațul."""
        from brain.cli.preview import describe_reason

        observation, age = self.hub.latest_observation()
        goal = self._confirmed
        problems: List[str] = []
        if goal is None:
            problems.append("nu există o țintă confirmată: încărcați o poză sau completați caroiajul, apoi „Confirmă ținta”")
        if observation is None:
            problems.append("camera nu are încă o analiză reușită a caroiajului")
        elif not observation.actionable:
            problems.append("observația curentă nu e acționabilă: {}".format(
                "; ".join(describe_reason(r) for r in observation.reasons)))
        elif observation.location not in self.system.planning.active_locations:
            problems.append("camera analizează {}, care nu e un șablon activ".format(observation.location))
        base = {"computed_at": _now(), "observation_age_s": None if age is None else round(age, 1), "moves": [],
                "unmet": [], "warnings": [], "estimated_s": 0.0, "complete": False, "already_done": False}
        if problems:
            return dict(base, ok=False, problems=problems)
        assert goal is not None
        try:
            plan, warnings = self._plan_from(observation, goal)
        except BrainError as exc:
            return dict(base, ok=False, problems=[str(exc)])
        return dict(base, ok=True, problems=[], moves=[self._move_json(m) for m in plan.moves], unmet=list(plan.unmet),
                    warnings=warnings, estimated_s=plan.estimated_s, complete=plan.complete, already_done=plan.already_done)

    def _plan_from(self, observation: Any, goal: ReproduceGoal) -> Any:
        from brain.adapters.clock import ManualClock
        from brain.orchestration.fsm import ERROR, TaskRunner
        from brain.orchestration.observer import LocationObserver
        from brain.orchestration.session import DurationJournal, TimeBudget
        from brain.planning.cost import CostModel
        from brain.planning.planner import Planner

        system = self.system
        clock = ManualClock()
        observer = LocationObserver(system, _FixedPerception(observation), clock, DurationJournal(None), move_arm=False,
                                    task="plan")
        planner = Planner(system, self.config.sort_map, CostModel.from_calib_file(system.planning.cost, system.arm.calib_json))
        runner = TaskRunner(goal, self.config, planner, observer, clock, TimeBudget(clock, system.timing.task_limit_s,
                                                                                    system.timing.reserve_s),
                            execute=False, task="reproduce")
        runner.pending_observe = [observation.location]  # doar ce vede camera acum; restul șabloanelor sunt neobservate
        result = runner.run()
        if result.final_state == ERROR or result.state is None:
            raise BrainError(result.error or "planificarea a eșuat")
        plan = planner.plan(goal, result.state)
        warnings = list(plan.warnings)
        missing = self.monitor.snapshot().get("buffers_missing") or []
        if missing and self.monitor.snapshot().get("ok"):
            warnings.append("punctele tampon necalibrate în serviciu vor fi ignorate la execuție: {}".format(", ".join(missing)))
        return plan, warnings

    def _slot_text(self, slot: SlotRef, level: int) -> str:
        if slot.cell is None:
            text = slot.location
        elif slot.location == self.target_location:
            text = "r{}c{}".format(slot.cell.row, slot.cell.col)
        else:
            text = "{}:r{}c{}".format(slot.location, slot.cell.row, slot.cell.col)
        return text + (" (niv. {})".format(level) if level else "")

    def _move_json(self, move: Move) -> Dict[str, Any]:
        grip = ""
        if move.pick.open_mm is not None:
            grip = "gheară pre-închisă la {:.0f} mm{}; eliberare la {:.0f} mm".format(
                move.pick.open_mm, _GRASP_TEXT.get(move.pick.grasp, ""), move.place.release_mm or 0.0)
        return {
            "color": move.cube.color.value, "color_ro": color_name(move.cube.color),
            "from": self._slot_text(move.pick.slot, move.pick.level), "to": self._slot_text(move.place.slot, move.place.level),
            "text": "{} din {} → {}".format(color_name(move.cube.color), self._slot_text(move.pick.slot, move.pick.level),
                                           self._slot_text(move.place.slot, move.place.level)),
            "purpose": move.purpose, "reason": move.reason, "estimated_s": move.estimated_s, "grip": grip,
            "grasp": move.pick.grasp, "uv": list(move.pick.uv) if move.pick.uv else None,
        }

    # ------------------------------------------------------------------ execuția
    def check_token(self, token: Optional[str]) -> Dict[str, Any]:
        return self.execution.check_token(token)

    def start_execution(self, token: Optional[str]) -> Dict[str, Any]:
        return self.execution.start(self._confirmed, token)

    def stop_execution(self) -> Dict[str, Any]:
        return self.execution.stop()

    def execution_status(self) -> Dict[str, Any]:
        return self.execution.status()
