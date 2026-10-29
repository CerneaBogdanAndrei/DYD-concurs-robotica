"""Îmbinarea observației cu memoria, sesiunea cu brațul, bugetul, verificarea prin proximitate și tabelul FSM."""

import dataclasses

import pytest

from brain.adapters.clock import ManualClock
from brain.domain.cube import Color, Occupancy
from brain.domain.errors import SafetyRejected
from brain.domain.grid import GridCoord
from brain.domain.plan import CubeRef, Move, PickAction, PlaceAction, SlotRef
from brain.domain.proximity import ProximityReading, SensorStatus, Validity
from brain.domain.scene_state import CellState, CubeState, LocationState, SceneState
from brain.orchestration import fsm
from brain.orchestration.memory_merge import (
    HEIGHT_FROM_SIZE, HEIGHT_UNCERTAIN, apply_height_hint, apply_move, estimated_height, get_stack, merge_cell,
)
from brain.orchestration.proximity_check import CUBE_BELOW, HELD, NOT_HELD, NOTHING_BELOW, UNKNOWN, ProximityVerifier
from brain.orchestration.session import ArmSession, DurationJournal, StopRequested, TimeBudget

RED, BLUE, GREEN = (CubeState(c, 0.9) for c in (Color.RED, Color.BLUE, Color.GREEN))
HIDDEN = CubeState(Color.UNKNOWN, 0.0)


def _occupied(*stack):
    return CellState(GridCoord(0, 0), Occupancy.OCCUPIED, 0.9, tuple(stack))


def test_merge_keeps_hidden_levels_from_memory():
    assert merge_cell(_occupied(BLUE), (RED, BLUE), ()).stack == (RED, BLUE)  # vârful confirmat
    assert merge_cell(_occupied(RED), (RED, BLUE), ()).stack == (RED,)  # vârful a dispărut
    assert merge_cell(_occupied(RED), (GREEN,), (RED, BLUE, RED)).stack == (RED, BLUE, RED)  # mutarea n-a avut loc
    empty = CellState(GridCoord(0, 0), Occupancy.EMPTY, 0.9)
    assert merge_cell(empty, (RED,), ()).stack == ()
    odd = merge_cell(_occupied(GREEN), (RED,), (BLUE,))  # cub necunoscut apărut deasupra
    assert odd.stack == (RED, GREEN) and HEIGHT_UNCERTAIN in odd.reasons
    assert merge_cell(_occupied(GREEN), (HIDDEN,), ()).stack == (GREEN,)  # cubul dedus din înălțime e dezvelit


def test_height_hint_from_apparent_size():
    assert estimated_height(250.0 / 220.0, 250.0, 30.0) == pytest.approx(1.0)
    assert estimated_height(250.0 / 190.0, 250.0, 30.0) == pytest.approx(2.0)
    cell = _occupied(RED)
    two = apply_height_hint(cell, 2.1, memory_known=False)
    assert two.stack == (HIDDEN, RED) and HEIGHT_FROM_SIZE in two.reasons
    assert apply_height_hint(cell, 2.1, memory_known=True) is cell  # memoria are prioritate
    assert apply_height_hint(cell, 1.1, memory_known=False) is cell
    assert HEIGHT_UNCERTAIN in apply_height_hint(cell, 3.0, memory_known=False).reasons


def _state():
    cells = [CellState(GridCoord(r, c), Occupancy.EMPTY, 1.0) for r in range(3) for c in range(3)]
    cells[0] = _occupied(RED, BLUE)
    return SceneState.new("t").with_location(LocationState("medium", 3, 3, tuple(cells)))


def _move(source, target, level=1, place_level=0, color=Color.BLUE):
    return Move(CubeRef(source, level, color), PickAction(source, level), PlaceAction(target, place_level), "goal", "test",
                0.0, True, 10.0)


def test_apply_move_to_cell_and_zone():
    source = SlotRef("medium", GridCoord(0, 0))
    state = apply_move(_state(), _move(source, SlotRef("zone_blue")))
    assert get_stack(state, source) == (RED,)
    assert [c.color for c in get_stack(state, SlotRef("zone_blue"))] == [Color.BLUE]
    state = apply_move(state, _move(source, SlotRef("medium", GridCoord(2, 2)), level=0, color=Color.RED))
    assert state.location("medium").cell(GridCoord(0, 0)).occupancy is Occupancy.EMPTY
    assert get_stack(state, SlotRef("medium", GridCoord(2, 2)))[0].uv_top is None  # poziția nouă nu e încă văzută


def test_time_budget():
    clock = ManualClock()
    budget = TimeBudget(clock, 300.0, 20.0)
    assert budget.can_afford(270.0) and not budget.can_afford(281.0)
    clock.sleep(250.0)
    assert budget.remaining() == pytest.approx(50.0) and not budget.can_afford(31.0) and not budget.exceeded()
    clock.sleep(51.0)
    assert budget.exceeded()


class _Arm:
    mode = "simulation"
    position_uncertain = False
    last_reason = None

    def __init__(self, clock):
        self.clock = clock

    def go_pose(self, name, speed="fast"):
        self.clock.sleep(1.2)

    def pick(self, *args, **kwargs):
        self.clock.sleep(4.5)
        return False


def test_arm_session_mode_journal_and_stop(tmp_path):
    clock = ManualClock()
    journal = DurationJournal(tmp_path / "t.jsonl")
    stop = {"now": False}
    session = ArmSession(_Arm(clock), clock, journal, "simulation", lambda: stop["now"])
    session.connect()
    session.go_pose("observe_medium")
    assert session.pick("medium", (0, 0), 0.0, 0, None) is False
    assert [(r.method, r.duration_s, r.result) for r in journal.records] == [("go_pose", 1.2, "ok"), ("pick", 4.5, "false")]
    assert len((tmp_path / "t.jsonl").read_text(encoding="utf-8").splitlines()) == 2
    stop["now"] = True
    with pytest.raises(StopRequested):
        session.go_pose("home")
    with pytest.raises(SafetyRejected):
        ArmSession(_Arm(clock), clock, journal, "hardware").connect()  # simularea nu trece drept braț real


class _Sensor:
    def __init__(self, readings):
        self.readings = list(readings)

    def read(self):
        value = self.readings.pop(0) if len(self.readings) > 1 else self.readings[0]
        if isinstance(value, Validity):
            return ProximityReading(None, value, SensorStatus.OK, 0.0, 1)
        return ProximityReading(float(value), Validity.VALID, SensorStatus.OK, 0.0, 1)


@pytest.mark.parametrize("readings, s, expected", [
    ([Validity.BELOW_RANGE] * 5, 25.0, HELD),
    ([55, 56, 54, 55, 57], 25.0, CUBE_BELOW),
    ([85, 86, 84, 85, 85], 25.0, NOTHING_BELOW),
    ([140] * 5, 25.0, NOT_HELD),
    ([55] * 5, None, NOT_HELD),  # s necalibrat: doar ținut / neținut
    ([Validity.TIMEOUT] * 3 + [55] * 2, 25.0, UNKNOWN),
])
def test_proximity_verifier(app_config, readings, s, expected):
    config = dataclasses.replace(app_config.system.proximity, samples=5)
    verifier = ProximityVerifier(_Sensor(readings), config, 30.0, s)
    assert verifier.after_place() == expected


def test_fsm_transition_table_is_closed():
    states = {fsm.OBSERVE, fsm.PLAN, fsm.EXECUTE, fsm.VERIFY, fsm.DONE, fsm.ERROR}
    sources = {source for source, _ in fsm.TRANSITIONS}
    targets = set(fsm.TRANSITIONS.values())
    assert sources | targets <= states
    assert not sources & {fsm.DONE, fsm.ERROR}  # stări finale
    assert targets >= {fsm.DONE, fsm.ERROR, fsm.EXECUTE, fsm.VERIFY, fsm.PLAN}
    for state in (fsm.OBSERVE, fsm.EXECUTE, fsm.VERIFY):  # orice eroare de braț duce în ERROR, fără home
        assert fsm.TRANSITIONS[(state, "arm_error")] == fsm.ERROR
        assert fsm.TRANSITIONS[(state, "stopped")] == fsm.ERROR
