"""Execuția țintei S1 din pagină: aceeași orchestrare ca `brain run --task reproduce --execute`.

Siguranță:
- implicit dezactivată (`preview.allow_execute: false`): butonul nu există, iar ruta răspunde 403;
- permisă, cere codul afișat în consola Pi-ului la pornire (serverul ascultă pe 0.0.0.0: oricine din rețea ar
  putea altfel mișca brațul); după `MAX_TOKEN_FAILURES` coduri greșite execuția se blochează până la repornire;
- aceleași verificări ca `brain run --execute`: modul serviciului (`status.sim` false pentru brațul real),
  locurile și pozele calibrate (`check_sites`), bugetul de timp;
- o singură execuție odată; „Oprește” = după comanda curentă, fără `home` automat (ca primul Ctrl+C);
- camera e deja deschisă de previzualizare: observările iau cadre noi din aceeași sesiune (`SessionPerception`).
"""

from __future__ import annotations

import datetime
import hmac
import json
import secrets
import threading
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from brain.adapters.preview_server import ApiError
from brain.configuration.loader import AppConfig
from brain.domain.cube import KNOWN_COLORS, Occupancy
from brain.domain.errors import ArmError, BrainError
from brain.domain.observation import GridObservation
from brain.planning.goals import ReproduceGoal

MAX_TOKEN_FAILURES = 10
TOKEN_ALPHABET = "abcdefghjkmnpqrstuvwxyz23456789"  # fără 0/o, 1/l/i: se citește ușor din consolă
TOKEN_LENGTH = 8


def new_token() -> str:
    return "".join(secrets.choice(TOKEN_ALPHABET) for _ in range(TOKEN_LENGTH))


def compare_with_target(observation: GridObservation, goal: ReproduceGoal) -> Dict[str, Any]:
    """Verificarea vizuală finală: ce se vede pe fiecare celulă a țintei față de ce se cere."""
    if not observation.actionable:
        return {"ok": False, "summary": "observația finală nu e acționabilă ({})".format(", ".join(observation.reasons)),
                "cells": None}
    cells: List[List[Dict[str, Any]]] = []
    good = total = 0
    for r in range(observation.rows):
        row = []
        for c in range(observation.cols):
            cell = observation.cells[r * observation.cols + c]
            seen = "unknown"
            if cell.occupancy is Occupancy.EMPTY:
                seen = "empty"
            elif cell.occupancy is Occupancy.OCCUPIED and cell.color in KNOWN_COLORS:
                seen = cell.color.value
            target = goal.cells.get(cell.coord)
            want = "-" if target is None else ("empty" if target.top is None else target.top.value)
            ok = target is None or want == seen
            if target is not None:
                total += 1
                good += int(ok)
            row.append({"want": want, "seen": seen, "ok": ok})
        cells.append(row)
    return {"ok": good == total, "summary": "{}/{} celule ca în țintă".format(good, total), "cells": cells}


class WebExecution:
    def __init__(
        self,
        config: AppConfig,
        hub: Any,
        preview: Any,
        out_root: Path,
        enabled: bool,
        arm_factory: Callable[[], Any],
        expected_mode: str,
        clock_factory: Callable[[], Any],
        token: Optional[str] = None,
        on_running: Optional[Callable[[bool], None]] = None,
    ) -> None:
        self.config = config
        self.hub = hub
        self.preview = preview  # PreviewSession (sau None: fără cameră, execuția e refuzată)
        self.out_root = Path(out_root)
        self.enabled = bool(enabled)
        self.arm_factory = arm_factory
        self.expected_mode = expected_mode
        self.clock_factory = clock_factory
        self.token = (token or new_token()) if self.enabled else None
        self.on_running = on_running or (lambda running: None)
        self.failures = 0
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._state: Dict[str, Any] = {"state": "idle"}
        self._runner: Any = None
        self._started_clock: Any = None

    # ------------------------------------------------------------------ acces
    @property
    def locked(self) -> bool:
        return self.failures >= MAX_TOKEN_FAILURES

    def _authorize(self, token: Optional[str]) -> None:
        if not self.enabled:
            raise ApiError(403, "execuția din pagină e dezactivată (preview.allow_execute: false în config/system.yaml)")
        if self.locked:
            raise ApiError(403, "prea multe coduri greșite: execuția e blocată până la repornirea previzualizării")
        assert self.token is not None
        if not token or not hmac.compare_digest(token.strip().encode("utf-8"), self.token.encode("utf-8")):
            with self._lock:
                self.failures += 1
                left = MAX_TOKEN_FAILURES - self.failures
            raise ApiError(403, "cod greșit (mai sunt {} încercări)".format(max(left, 0)))

    def check_token(self, token: Optional[str]) -> Dict[str, Any]:
        self._authorize(token)
        return {"ok": True}

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ------------------------------------------------------------------ comenzi
    def start(self, goal: Optional[ReproduceGoal], token: Optional[str]) -> Dict[str, Any]:
        self._authorize(token)
        if goal is None:
            raise ApiError(409, "nu există o țintă confirmată")
        if self.preview is None or getattr(self.preview, "pipeline", None) is None:
            raise ApiError(409, "previzualizarea rulează fără analiză (--no-analyze): execuția nu poate observa")
        with self._lock:
            if self.running:
                raise ApiError(409, "o execuție e deja în curs; așteptați să se termine sau opriți-o")
            stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
            out = self.out_root / "run-{}".format(stamp)
            self._stop.clear()
            self._runner = None
            self._state = {"state": "running", "started_at": datetime.datetime.now().isoformat(timespec="seconds"),
                           "goal": goal.describe(), "out_dir": str(out), "error": None, "report": None,
                           "final_check": None, "goal_reached": False}
            self._thread = threading.Thread(target=self._run, args=(goal, out), name="web-execution", daemon=True)
            self._thread.start()
        return {"started": True, "out_dir": str(out)}

    def stop(self) -> Dict[str, Any]:
        if not self.enabled:
            raise ApiError(403, "execuția din pagină e dezactivată")
        if not self.running:
            raise ApiError(409, "nicio execuție în curs")
        self.request_stop()
        return {"stopping": True}

    def request_stop(self) -> None:
        """Oprire după comanda curentă a brațului, fără `home` (ca primul Ctrl+C din `brain run`)."""
        self._stop.set()

    def wait(self, timeout: Optional[float] = None) -> bool:
        thread = self._thread
        if thread is not None:
            thread.join(timeout)
        return not self.running

    # ------------------------------------------------------------------ starea, pentru pagină
    def status(self) -> Dict[str, Any]:
        with self._lock:
            state = dict(self._state)
        runner = self._runner
        state.update({"enabled": self.enabled, "locked": self.locked, "stop_requested": self._stop.is_set()})
        if runner is not None:
            records = [m.to_dict() for m in list(runner.result.moves)]
            for record in records:
                record["color"] = record["move"].split(" ", 1)[0]
            state["moves"] = records
            state["fsm_state"] = runner.state
            state["remaining"] = [line for line in list(runner.result.last_plan) if line[:3].strip().rstrip(".").isdigit()]
            if self._started_clock is not None and state.get("state") == "running":
                state["elapsed_s"] = round(runner.budget.elapsed(), 1)
        return state

    # ------------------------------------------------------------------ firul de execuție
    def _run(self, goal: ReproduceGoal, out: Path) -> None:
        from brain.cli.preview import SessionPerception
        from brain.cli.run import check_sites
        from brain.memory.store import SceneStore
        from brain.orchestration.fsm import DONE, TaskRunner
        from brain.orchestration.observer import LocationObserver, ObservationFailed
        from brain.orchestration.session import ArmSession, DurationJournal, TimeBudget
        from brain.planning.cost import CostModel
        from brain.planning.planner import Planner
        from brain.ports.arm import MODE_SIMULATION

        out.mkdir(parents=True, exist_ok=True)
        self.on_running(True)
        self.preview.pause_analysis.set()
        final: Dict[str, Any] = {"state": "error"}
        try:
            clock = self.clock_factory()
            self._started_clock = clock
            journal = DurationJournal(out / "timings.jsonl", simulated=self.expected_mode == MODE_SIMULATION)
            session = ArmSession(self.arm_factory(), clock, journal, self.expected_mode, self._stop.is_set, task="reproduce")
            config = check_sites(self.config, session.connect(), need_zones=False)
            system = config.system
            perception = SessionPerception(self.preview, system.vision.frames_to_aggregate)
            observer = LocationObserver(system, perception, clock, journal, session, self.hub, move_arm=True, task="reproduce")
            planner = Planner(system, config.sort_map, CostModel.from_calib_file(system.planning.cost, system.arm.calib_json))
            budget = TimeBudget(clock, system.timing.task_limit_s, system.timing.reserve_s)
            runner = TaskRunner(goal, config, planner, observer, clock, budget, session, SceneStore(out / "scene_state.json"),
                                execute=True, task="reproduce")
            self._runner = runner
            result = runner.run()
            report = result.to_dict()
            if result.final_state == DONE:
                try:  # verificarea vizuală finală (brațul e deja în poza de observare după ultima verificare)
                    check = compare_with_target(observer.observe(goal.location), goal)
                except (ObservationFailed, ArmError, BrainError) as exc:
                    check = {"ok": False, "summary": "verificare imposibilă: {}".format(exc), "cells": None}
            else:
                check = {"ok": False, "summary": "nefăcută: execuția nu s-a terminat ({})".format(result.error or result.final_state),
                         "cells": None}
            report.update({"final_check": check, "execute": True, "source": "pagina de previzualizare",
                           "expected_mode": self.expected_mode})
            (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
            stopped = self._stop.is_set() and result.final_state != DONE
            final = {"state": "stopped" if stopped else ("done" if result.final_state == DONE else "error"),
                     "error": result.error, "report": {k: report[k] for k in ("final_state", "goal_reached", "unmet",
                                                                               "warnings", "elapsed_s", "moves_attempted")},
                     "final_check": check, "goal_reached": bool(result.goal_reached and check.get("ok")),
                     "elapsed_s": round(result.elapsed_s, 1)}
        except (BrainError, ArmError) as exc:
            final = {"state": "error", "error": "{}: {}".format(type(exc).__name__, exc)}
        except Exception as exc:  # firul nu are voie să lase pagina „în curs” la nesfârșit
            final = {"state": "error", "error": "eroare internă: {}: {}".format(type(exc).__name__, exc)}
        finally:
            self.preview.pause_analysis.clear()
            with self._lock:
                self._state.update(final)
            self.on_running(False)
