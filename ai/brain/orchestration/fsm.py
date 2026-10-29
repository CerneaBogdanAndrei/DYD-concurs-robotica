"""FSM-ul unei sarcini: OBSERVE → PLAN → EXECUTE → VERIFY → (PLAN ...) → DONE / ERROR.

Buclă închisă: la fiecare pas se replanifică din memoria verificată și se execută doar prima mutare.
Tranzițiile permise sunt în `TRANSITIONS`; orice altă tranziție e o eroare de program.

Recuperare:
- pick ratat (vizibil la reobservare) → reîncercare până la `max_pick_attempts` pe același
  loc, apoi locul e ocolit;
- pick/place = False sau `MotionAborted` → locul e ocolit; brațul trece prin `go_pose(observe_*)` înainte de
  următoarea comandă (poziția poate fi alta, constatarea 1 a serviciului); dacă place eșuează cu cubul în
  gheară, cubul e pus înapoi de unde a fost luat;
- cub căzut → reobservarea îl găsește, iar planul următor îl tratează ca pe orice alt cub;
- timeout / eroare de comunicație → ERROR, fără `home` automat (starea fizică e necunoscută);
- observație neacționabilă → reîncercări (`max_observe_attempts`), apoi ERROR;
- buget depășit → nicio mutare nouă (DONE cu obiectivele rămase raportate).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field, replace
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

from brain.configuration.loader import AppConfig
from brain.domain.cube import Occupancy
from brain.domain.errors import ArmCommunicationError, ArmError, BrainError, MotionAborted
from brain.domain.plan import Move, Plan, SlotRef
from brain.domain.scene_state import SceneState, location_from_observation
from brain.logging_setup import log_event
from brain.memory.store import SceneStore
from brain.orchestration.memory_merge import (
    GHOST_REASONS,
    PARALLAX_GHOST,
    apply_height_hint,
    apply_move,
    estimated_height,
    get_stack,
    merge_location,
    set_stack,
)
from brain.orchestration.observer import LocationObserver, ObservationFailed
from brain.orchestration.session import ArmSession, StopRequested, TimeBudget
from brain.planning.goals import Goal
from brain.planning.planner import Planner

logger = logging.getLogger("brain.orchestration.fsm")

OBSERVE, PLAN, EXECUTE, VERIFY, DONE, ERROR = "OBSERVE", "PLAN", "EXECUTE", "VERIFY", "DONE", "ERROR"

TRANSITIONS: Dict[Tuple[str, str], str] = {
    (OBSERVE, "observed"): PLAN,
    (OBSERVE, "observe_failed"): ERROR,
    (OBSERVE, "arm_error"): ERROR,
    (OBSERVE, "stopped"): ERROR,
    (PLAN, "goal_reached"): DONE,
    (PLAN, "nothing_doable"): DONE,
    (PLAN, "budget_exhausted"): DONE,
    (PLAN, "dry_run"): DONE,
    (PLAN, "final_check"): OBSERVE,
    (PLAN, "move_selected"): EXECUTE,
    (PLAN, "step_limit"): ERROR,
    (EXECUTE, "executed"): VERIFY,
    (EXECUTE, "pick_failed"): VERIFY,
    (EXECUTE, "unreachable"): VERIFY,
    (EXECUTE, "arm_error"): ERROR,
    (EXECUTE, "stopped"): ERROR,
    (VERIFY, "verified"): PLAN,
    (VERIFY, "observe_failed"): ERROR,
    (VERIFY, "arm_error"): ERROR,
    (VERIFY, "stopped"): ERROR,
}

MAX_EXECUTED_MOVES = 60
# Un cub întreg are fața cel puțin cât cea nominală (size_ratio ≥ ~1,1 pe masă); revărsarea fețelor unei stive
# înalte vecine apare ca fragment (0,3..0,9 pe datele sintetice).
FRAGMENT_SIZE_RATIO = 0.95


@dataclass
class MoveRecord:
    index: int
    move: str
    purpose: str
    outcome: str
    detail: str = ""
    started_s: float = 0.0
    duration_s: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {"index": self.index, "move": self.move, "purpose": self.purpose, "outcome": self.outcome,
                "detail": self.detail, "started_s": round(self.started_s, 2), "duration_s": round(self.duration_s, 2)}


@dataclass
class RunResult:
    task: str
    goal: str
    final_state: str = OBSERVE
    goal_reached: bool = False
    budget_exhausted: bool = False
    unmet: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    moves: List[MoveRecord] = field(default_factory=list)
    transitions: List[Tuple[str, str, str]] = field(default_factory=list)
    error: Optional[str] = None
    elapsed_s: float = 0.0
    first_plan: List[str] = field(default_factory=list)
    first_plan_estimated_s: float = 0.0
    last_plan: List[str] = field(default_factory=list)
    observations: int = 0
    arm_commands: int = 0
    state: Optional[SceneState] = None

    def to_dict(self) -> Dict[str, Any]:
        counts: Dict[str, int] = {}
        for record in self.moves:
            counts[record.outcome] = counts.get(record.outcome, 0) + 1
        return {
            "task": self.task, "goal": self.goal, "final_state": self.final_state, "goal_reached": self.goal_reached,
            "budget_exhausted": self.budget_exhausted, "unmet": list(self.unmet), "warnings": list(self.warnings),
            "error": self.error, "elapsed_s": round(self.elapsed_s, 2), "moves_attempted": len(self.moves),
            "move_outcomes": counts, "observations": self.observations, "arm_commands": self.arm_commands,
            "first_plan": list(self.first_plan), "first_plan_estimated_s": self.first_plan_estimated_s,
            "last_plan": list(self.last_plan), "moves": [m.to_dict() for m in self.moves],
            "transitions": [list(t) for t in self.transitions],
        }


class TaskRunner:
    def __init__(
        self,
        goal: Goal,
        config: AppConfig,
        planner: Planner,
        observer: LocationObserver,
        clock: Any,
        budget: TimeBudget,
        session: Optional[ArmSession] = None,
        store: Optional[SceneStore] = None,
        initial_state: Optional[SceneState] = None,
        execute: bool = True,
        task: str = "",
        on_step: Optional[Callable[[str, Optional[Move]], None]] = None,
        max_moves: int = MAX_EXECUTED_MOVES,
    ) -> None:
        self.goal = goal
        self.config = config
        self.system = config.system
        self.planner = planner
        self.observer = observer
        self.clock = clock
        self.budget = budget
        self.session = session
        self.store = store
        self.execute = execute
        self.task = task
        self.on_step = on_step or (lambda event, move: None)
        self.max_moves = max_moves
        self.memory = initial_state or SceneState.new("run-{}".format(task or "task"))
        # Cu o memorie dată și fără execuție (doar planificare) nu se observă nimic; altfel se observă tot,
        # iar memoria dată completează ce nu se vede (nivelurile ascunse, zonele).
        self.pending_observe: List[str] = (
            [] if initial_state is not None and not execute and initial_state.locations
            else list(self.system.planning.active_locations))
        self.avoid_sources: Set[SlotRef] = set()
        self.avoid_targets: Set[SlotRef] = set()
        self.pick_attempts: Dict[SlotRef, int] = {}
        self.state = OBSERVE
        self.result = RunResult(task, goal.describe())
        self._move: Optional[Move] = None
        self._before: Optional[SceneState] = None
        self._outcome = ""
        self._final_checked = False

    # ------------------------------------------------------------------ infrastructură
    def _transition(self, event: str) -> None:
        key = (self.state, event)
        if key not in TRANSITIONS:
            raise BrainError("tranziție nepermisă: {} --{}-->".format(self.state, event))
        target = TRANSITIONS[key]
        self.result.transitions.append((self.state, event, target))
        log_event(logger, "transition", task=self.task, state=self.state, action=event, result=target)
        self.state = target

    def _warn(self, text: str) -> None:
        if text not in self.result.warnings:
            self.result.warnings.append(text)

    def _save(self) -> None:
        if self.store is not None:
            self.store.save(self.memory, overwrite=True, replace_invalid=True)

    def _observe_locations(self, names: List[str], touched: Tuple[SlotRef, ...] = ()) -> None:
        """Observă locațiile și le îmbină cu memoria. `touched` = sloturile ultimei mutări: doar ele se pot fi
        schimbat; o celulă neatinsă, vecină cu o stivă înaltă și văzută degradat, își păstrează memoria."""
        before = self._before
        min_confidence = self.system.planning.min_confidence
        for name in names:
            observation = self.observer.observe(name)
            observed = location_from_observation(observation)
            expected = self.memory.locations.get(name)
            previous = before.locations.get(name) if before is not None else None
            merged = merge_location(observed, expected, previous)
            loc = self.system.location(name)
            cells = []
            for cell, seen in zip(merged.cells, observation.cells):
                prior = expected.cell(cell.coord) if expected is not None else None
                known = prior is not None and bool(prior.stack)  # memoria știe înălțimea: are prioritate
                untouched = SlotRef(name, cell.coord) not in touched
                degraded = any(r in GHOST_REASONS for r in seen.reasons) or (
                    cell.occupancy is not Occupancy.EMPTY and seen.color_confidence < min_confidence)
                ratio = seen.metrics.get("size_ratio")
                fragment = ratio is not None and ratio < FRAGMENT_SIZE_RATIO  # o bucată de față, nu un cub întreg
                if prior is not None and (degraded and (untouched or fragment)) and _near_tall_stack(expected, cell.coord) and (
                        prior.stack or cell.occupancy is not Occupancy.EMPTY):
                    log_event(logger, "parallax_ghost", task=self.task, state=self.state, action=name, cell=str(cell.coord),
                              reasons=list(seen.reasons), kept=len(prior.stack))
                    cells.append(replace(prior, reasons=(PARALLAX_GHOST,)))
                    continue
                estimate = estimated_height(seen.metrics.get("size_ratio"), loc.camera_height_mm, self.system.cube.size_mm)
                cells.append(apply_height_hint(cell, estimate, known))
            merged = replace(merged, cells=tuple(cells))
            self.memory = self.memory.with_location(merged)



    # ------------------------------------------------------------------ bucla
    def run(self) -> RunResult:
        while self.state not in (DONE, ERROR):
            try:
                if self.state == OBSERVE:
                    self._do_observe()
                elif self.state == PLAN:
                    self._do_plan()
                elif self.state == EXECUTE:
                    self._do_execute()
                elif self.state == VERIFY:
                    self._do_verify()
            except StopRequested as exc:
                self.result.error = str(exc)
                self._transition("stopped")
            except ObservationFailed as exc:
                self.result.error = str(exc)
                self._transition("observe_failed")
            except ArmError as exc:
                self.result.error = "{}: {}".format(type(exc).__name__, exc)
                if isinstance(exc, ArmCommunicationError):
                    self.result.error += " (starea brațului e necunoscută; nu s-a trimis home)"
                self._transition("arm_error")
        self._save()
        self.result.final_state = self.state
        self.result.elapsed_s = self.budget.elapsed()
        self.result.observations = self.observer.observations
        self.result.arm_commands = self.session.commands if self.session is not None else 0
        self.result.state = self.memory
        log_event(logger, "task_end", task=self.task, state=self.state, result="goal_reached" if self.result.goal_reached else "not_reached",
                  error=self.result.error, duration_ms=round(self.result.elapsed_s * 1000, 1))
        return self.result

    def _do_observe(self) -> None:
        names, self.pending_observe = self.pending_observe, []
        self._observe_locations(names)
        self._save()
        self.on_step("observed", None)
        self._transition("observed")

    def _do_plan(self) -> None:
        plan = self.planner.plan(self.goal, self.memory, frozenset(self.avoid_sources), frozenset(self.avoid_targets))
        explanation = plan.explanation()
        if not self.result.first_plan and (plan.moves or plan.complete or plan.unmet):
            self.result.first_plan = explanation
            self.result.first_plan_estimated_s = plan.estimated_s
        self.result.last_plan = explanation
        self.result.unmet = list(plan.unmet)
        for text in plan.warnings:
            self._warn(text)
        log_event(logger, "plan", task=self.task, state=PLAN, result="complete" if plan.complete else "partial",
                  moves=len(plan.moves), estimated_s=plan.estimated_s, unmet=list(plan.unmet))
        if not plan.moves:
            if (self.system.planning.camera_verify == "end" and self.result.moves and not self._final_checked
                    and self.execute):
                self._final_checked = True
                self.pending_observe = list(self.system.planning.active_locations)
                self._before = None
                self._transition("final_check")
                return
            self.result.goal_reached = plan.complete
            self._transition("goal_reached" if plan.complete else "nothing_doable")
            return
        if not self.execute:
            self._transition("dry_run")
            return
        if len(self.result.moves) >= self.max_moves:
            self.result.error = "limita de {} mutări executate a fost atinsă".format(self.max_moves)
            self._transition("step_limit")
            return
        move = plan.moves[0]
        if not self.budget.can_afford(move.estimated_s):
            self.result.budget_exhausted = True
            self._warn("bugetul de timp nu mai permite o mutare ({:.0f} s rămase, mutarea ~{:.0f} s, rezervă {:.0f} s)".format(
                self.budget.remaining(), move.estimated_s, self.budget.reserve_s))
            self._transition("budget_exhausted")
            return
        self._move = move
        self._transition("move_selected")

    # ------------------------------------------------------------------ execuție
    def _recover_pose(self, slot: SlotRef) -> None:
        """După un False / MotionAborted brațul poate fi în altă poziție: trece printr-o poză sigură."""
        assert self.session is not None
        if not self.session.position_uncertain:
            return
        pose = None if slot.is_zone else self.system.location(slot.location).observe_pose
        self.session.go_pose(pose or "home")

    def _do_execute(self) -> None:
        assert self._move is not None and self.session is not None
        move = self._move
        self._before = self.memory
        started = self.budget.elapsed()
        record = MoveRecord(len(self.result.moves) + 1, _move_text(move), move.purpose, "", started_s=started)
        self.result.moves.append(record)
        pick, place = move.pick, move.place
        self._recover_pose(pick.slot)
        try:
            ok = self.session.pick(pick.slot.location, _cell(pick.slot), pick.yaw_deg, pick.level, pick.uv, pick.open_mm)
        except MotionAborted as exc:
            ok = False
            record.detail = "pick oprit de serviciu: {}".format(exc)
        if not ok:
            record.detail = record.detail or "pick inaccesibil: {}".format(self.session.last_reason)
            self._finish_record(record, "unreachable_pick")
            self.avoid_sources.add(pick.slot)
            self._warn("{} ocolit: {}".format(pick.slot, record.detail))
            self._event("unreachable")
            return
        try:
            ok = self.session.place(place.slot.location, _cell(place.slot), place.level, place.yaw_deg, place.release_mm,
                                    place.uv)
        except MotionAborted as exc:
            ok = False
            record.detail = "place oprit de serviciu: {}".format(exc)
        if not ok:
            record.detail = record.detail or "place inaccesibil: {}".format(self.session.last_reason)
            self.avoid_targets.add(place.slot)
            self._warn("{} ocolit ca destinație: {}".format(place.slot, record.detail))
            self._recover_pose(pick.slot)
            if not self.session.place(pick.slot.location, _cell(pick.slot), pick.level, 0.0, place.release_mm):  # înapoi
                raise ArmCommunicationError("cubul ținut nu poate fi pus nici înapoi pe {}: intervenție manuală".format(pick.slot))
            self._finish_record(record, "unreachable_place")
            self._event("unreachable")
            return
        self.memory = apply_move(self.memory, move)
        self._finish_record(record, "ok")
        self._event("executed")

    def _finish_record(self, record: MoveRecord, outcome: str) -> None:
        record.outcome = outcome
        record.duration_s = self.budget.elapsed() - record.started_s
        self._outcome = outcome
        self.on_step("executed", self._move)

    def _event(self, name: str) -> None:
        self._transition(name)

    def _count_miss(self, slot: SlotRef) -> None:
        count = self.pick_attempts.get(slot, 0) + 1
        self.pick_attempts[slot] = count
        if count >= self.system.planning.max_pick_attempts:
            self.avoid_sources.add(slot)
            self._warn("{}: cub neprins de {} ori; ocolit".format(slot, count))

    # ------------------------------------------------------------------ verificare
    def _do_verify(self) -> None:
        assert self._move is not None and self._before is not None
        move = self._move
        outcome = self._outcome
        each_move = self.system.planning.camera_verify == "each_move"
        touched = sorted({s.location for s in (move.pick.slot, move.place.slot) if not s.is_zone})
        if move.place.slot.is_zone:
            touched = list(self.system.planning.active_locations)  # zonele nu se văd: un cub căzut poate ateriza oriunde
        if each_move or outcome not in ("ok",):
            expected_grids = {name: loc for name, loc in self.memory.locations.items()}
            try:
                self._observe_locations(touched, (move.pick.slot, move.place.slot))
            except ObservationFailed as exc:
                if outcome != "ok":
                    raise
                # De ex. un turn înalt sau un caroiaj plin acoperă liniile: mutarea a raportat succes, memoria rămâne.
                self._warn("verificare vizuală imposibilă după mutarea {}: {}; memoria păstrează starea așteptată".format(
                    len(self.result.moves), exc))
                self.result.moves[-1].detail = "neverificat vizual"
            else:
                if outcome == "ok":
                    self._check_executed_move(move, expected_grids, touched)
        self._save()
        self.on_step("verified", move)
        self._transition("verified")

    def _check_executed_move(self, move: Move, expected_grids: Dict[str, Any], observed: List[str]) -> None:
        """Compară ce s-a văzut cu ce se aștepta: pick ratat, cub căzut, zonă neconfirmată."""
        assert self._before is not None
        record = self.result.moves[-1]
        source, target = move.pick.slot, move.place.slot
        color = move.cube.color
        before_source = get_stack(self._before, source)
        now_source = get_stack(self.memory, source)
        before_target = get_stack(self._before, target)
        now_target = get_stack(self.memory, target)
        target_unchanged = not target.is_zone and len(now_target) == len(before_target) and (
            not now_target or now_target[-1].color is before_target[-1].color)
        source_shows_cube = bool(now_source) and now_source[-1].color is color
        missed = source_shows_cube and (len(now_source) == len(before_source) or target_unchanged)
        if missed:
            record.outcome = "pick_missed"
            record.detail = "reobservare: cubul a rămas pe {}".format(source)
            self._count_miss(source)
            # Două cuburi de aceeași culoare unul peste altul arată la fel de sus: înălțimea vine din memorie.
            self.memory = set_stack(self.memory, source, before_source[:-1] + (now_source[-1],))
            if target.is_zone:
                self.memory = set_stack(self.memory, target, before_target)
            return
        if not target.is_zone:
            if len(now_target) != move.place.level + 1 or now_target[-1].color is not color:
                record.outcome = "mismatch"
                record.detail = "reobservare: pe {} nu e cubul {} la nivelul {}".format(target, color.value, move.place.level)
                remaining = [n for n in self.system.planning.active_locations if n not in observed]
                if remaining:
                    self._observe_locations(remaining, (move.pick.slot, move.place.slot))
            return
        # Ținta e un punct (zonă/tampon), invizibil camerei: dacă un cub de culoarea mutată a apărut pe o celulă
        # care trebuia să fie goală, el a căzut pe drum și nu e în zonă.
        landed = _appeared(expected_grids, self.memory.locations, color)
        if landed and get_stack(self.memory, target) and get_stack(self.memory, target)[-1].color is color:
            self.memory = set_stack(self.memory, target, get_stack(self.memory, target)[:-1])
            record.outcome = "dropped"
            record.detail = "reobservare: cubul {} a căzut pe {}, nu e în {}".format(color.value, ", ".join(landed), target)


def _near_tall_stack(location: Any, coord: Any, height: int = 2) -> bool:
    """Fața unei stive înalte vecine e mărită de paralaxă și se poate revărsa în această celulă."""
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            r, c = coord.row + dr, coord.col + dc
            if (dr or dc) and 0 <= r < location.rows and 0 <= c < location.cols:
                if len(location.cells[r * location.cols + c].stack) >= height:
                    return True
    return False


def _appeared(expected: Any, observed: Any, color: Any) -> List[str]:
    """Celulele așteptate goale pe care acum stă un cub de culoarea dată."""
    found = []
    for name, loc in observed.items():
        before = expected.get(name)
        if before is None:
            continue
        for cell in loc.cells:
            if cell.top is not None and cell.top.color is color and not before.cell(cell.coord).stack:
                found.append("{}{}".format(name, cell.coord))
    return found


def _cell(slot: SlotRef) -> Optional[Tuple[int, int]]:
    return None if slot.cell is None else slot.cell.as_tuple()


def _move_text(move: Move) -> str:
    return "{} {} (niv. {}) -> {} (niv. {})".format(move.cube.color.value, move.pick.slot, move.pick.level,
                                                  move.place.slot, move.place.level)


def plan_summary(plan: Plan) -> Dict[str, Any]:
    return {"complete": plan.complete, "moves": len(plan.moves), "estimated_s": plan.estimated_s,
            "unmet": list(plan.unmet), "warnings": list(plan.warnings), "explanation": plan.explanation()}
