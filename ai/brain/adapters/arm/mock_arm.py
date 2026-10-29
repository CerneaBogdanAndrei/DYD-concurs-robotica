"""MockArm: implementarea simulată a `ArmInterface`, legată de `SimWorld`.

Ca serviciul real, nu are senzor de prindere: `pick` întoarce `True` și când gheara a ratat cubul;
doar o țintă inaccesibilă dă `False`. Erorile se pot injecta determinist (`FaultSpec`).
Toate comenzile sunt înregistrate în `log`. Duratele sunt simulate prin ceasul injectat.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from brain.adapters.arm import validation as v
from brain.domain.errors import ArmCommunicationError, ArmTimeout, CalibrationMissing, InvalidCommand
from brain.domain.grasp import Footprint, GripperGeometry, convex_overlap, finger_polygons
from brain.ports.arm import MODE_SIMULATION
from brain.ports.sensors import Clock
from brain.simulation.sim_world import GRASPED, UNREACHABLE, SimWorld

FAULT_KINDS = ("timeout", "comm_error", "unreachable", "grasp_fail", "drop", "calibration")


@dataclass(frozen=True)
class FaultSpec:
    """Eroare injectată pe metoda `method`: la apelul numărul `call` (1, 2, ...) sau cu `probability`."""

    method: str
    kind: str
    call: Optional[int] = None
    probability: float = 0.0

    def __post_init__(self) -> None:
        if self.kind not in FAULT_KINDS:
            raise ValueError("tip de eroare necunoscut: {} (permise: {})".format(self.kind, ", ".join(FAULT_KINDS)))
        if self.call is None and not 0.0 < self.probability <= 1.0:
            raise ValueError("FaultSpec cere `call` sau o probabilitate în (0, 1]")


@dataclass(frozen=True)
class SimDurations:
    """Durate SIMULATE (s), nu măsurători. Referința Python estimează ~9-10 s pentru pick + place."""

    go_pose_s: float = 1.2
    home_s: float = 1.2
    gripper_s: float = 0.4
    pick_s: float = 4.5
    place_s: float = 4.5
    beep_s: float = 0.2


@dataclass
class CommandRecord:
    method: str
    args: Dict[str, Any]
    started: float
    duration: float = 0.0
    result: Any = None
    error: Optional[str] = None


class MockArm:
    mode = MODE_SIMULATION

    def __init__(
        self,
        world: SimWorld,
        poses: Iterable[str],
        clock: Clock,
        durations: Optional[SimDurations] = None,
        faults: Sequence[FaultSpec] = (),
        seed: int = 0,
        gripper: Optional[GripperGeometry] = None,
    ) -> None:
        self.world = world
        self.gripper = gripper  # None: fără verificarea degetelor
        self.collisions: List[str] = []
        self.poses = set(poses) | {"home"}
        self.clock = clock
        self.durations = durations or SimDurations()
        self.faults = list(faults)
        self.rng = np.random.default_rng(seed)
        self.last_reason: Optional[str] = None
        self.position_uncertain = False  # ca la TeamArm: True după False sau o eroare cu stare necunoscută
        self.current_pose: Optional[str] = None
        self.joints: List[float] = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        self.gripper_open = False
        self.log: List[CommandRecord] = []
        self._calls: Dict[str, int] = {}

    # ------------------------------------------------------------------ infrastructură
    def _fault(self, method: str) -> Optional[str]:
        count = self._calls.get(method, 0) + 1
        self._calls[method] = count
        for fault in self.faults:
            if fault.method != method:
                continue
            if fault.call is not None and fault.call == count:
                return fault.kind
            if fault.call is None and self.rng.random() < fault.probability:
                return fault.kind
        return None

    def _record(self, method: str, args: Dict[str, Any]) -> CommandRecord:
        record = CommandRecord(method, args, self.clock.monotonic())
        self.log.append(record)
        return record

    def _finish(self, record: CommandRecord, seconds: float, result: Any = None) -> Any:
        self.clock.sleep(seconds)
        record.duration = seconds
        record.result = result
        return result

    def _raise(self, record: CommandRecord, kind: str, method: str) -> None:
        if kind in ("timeout", "comm_error"):
            self.position_uncertain = True
        if kind == "timeout":
            record.error = "timeout"
            raise ArmTimeout("{}: timeout simulat (starea fizică necunoscută)".format(method))
        if kind == "comm_error":
            record.error = "comm_error"
            raise ArmCommunicationError("{}: eroare de comunicație simulată".format(method))
        if kind == "calibration":
            record.error = "calibration"
            raise CalibrationMissing("{}: calibrare lipsă (simulat)".format(method))

    def _check_site(self, location: str, cell: Optional[Tuple[int, int]]) -> None:
        if not self.world.knows(location):
            raise CalibrationMissing("locul '{}' nu e calibrat (npm run calib -- celule/zona {})".format(location, location))
        if location in self.world.grids:
            if cell is None:
                raise InvalidCommand("locul '{}' e un caroiaj: cell e obligatoriu".format(location))
            grid = self.world.grids[location]
            if not (cell[0] < grid.rows and cell[1] < grid.cols):
                raise InvalidCommand("cell {} în afara caroiajului {}".format(cell, location))
        elif cell is not None:
            raise InvalidCommand("locul '{}' e o zonă: cell trebuie să lipsească".format(location))

    def _check_fingers(self, method: str, location: str, cell: Optional[Tuple[int, int]], level: int, yaw_deg: float,
                       uv: Optional[Tuple[float, float]], opening: Optional[float]) -> None:
        """Degetele, la deschiderea cu care coboară (pick) sau se deschid (place), ating un cub vecin?

        Doar o constatare în `world.events` și `collisions` (fără efect fizic). Fără deschidere cerută se
        presupune gheara deschisă la maximum, ca serviciul necalibrat (GRIP_OPEN). Axa de închidere: axa a.
        """
        gripper = self.gripper
        if gripper is None or cell is None or location not in self.world.grids:
            return
        grid = self.world.grids[location]
        size, cube = grid.cell_mm, self.world.cube_mm
        center = (uv[0] * size, uv[1] * size) if uv is not None else ((cell[1] + 0.5) * size, (cell[0] + 0.5) * size)
        fingers = finger_polygons(center, yaw_deg, "a", opening or gripper.max_open_mm, gripper.finger_mm, cube)
        for other, stack in sorted(grid.stacks.items()):
            if other == cell or len(stack) <= level:
                continue
            top = stack[-1]
            neighbor = Footprint(((other[1] + 0.5) * size + top.offset_mm[0], (other[0] + 0.5) * size + top.offset_mm[1]),
                               top.yaw_deg, cube)
            if any(convex_overlap(finger, neighbor.corners(), tolerance_mm=0.05) for finger in fingers):
                text = "{} {}{}: degetul ghearei ({:.0f} mm) atinge cubul de pe {}{}".format(
                    method, location, cell, opening or gripper.max_open_mm, location, other)
                self.collisions.append(text)
                self.world.events.append(text)

    # ------------------------------------------------------------------ ArmInterface
    def home(self) -> None:
        record = self._record("home", {})
        kind = self._fault("home")
        if kind:
            self._raise(record, kind, "home")
        self.current_pose = "home"
        self.position_uncertain = False
        self.world.hover = None
        self.joints[:5] = [90.0] * 5
        self._finish(record, self.durations.home_s)

    def go_pose(self, name: str, speed: str = "fast") -> None:
        v.check_speed(speed)
        record = self._record("go_pose", {"name": name, "speed": speed})
        if name not in self.poses:
            record.error = "calibration"
            raise CalibrationMissing("poza '{}' nu e învățată (npm run calib -- poza {})".format(name, name))
        kind = self._fault("go_pose")
        if kind:
            self._raise(record, kind, "go_pose")
        self.current_pose = name
        self.position_uncertain = False
        self.world.hover = None
        factor = 2.0 if speed == "slow" else 1.0
        self._finish(record, self.durations.go_pose_s * factor)

    def move_joints(self, angles: List[float], time_ms: int) -> None:
        values = v.check_angles(angles)
        v.check_time_ms(time_ms)
        record = self._record("move_joints", {"angles": values, "time_ms": time_ms})
        kind = self._fault("move_joints")
        if kind:
            self._raise(record, kind, "move_joints")
        self.joints[: len(values)] = values
        self.current_pose = None
        self.world.hover = None
        self._finish(record, time_ms / 1000.0)

    def open_gripper(self, full: bool = True) -> None:
        record = self._record("open_gripper", {"full": full})
        kind = self._fault("open_gripper")
        if kind:
            self._raise(record, kind, "open_gripper")
        if self.world.held is not None:
            self.world.drop_held_in_transit()
        self.gripper_open = True
        self._finish(record, self.durations.gripper_s)

    def close_gripper(self) -> None:
        record = self._record("close_gripper", {})
        kind = self._fault("close_gripper")
        if kind:
            self._raise(record, kind, "close_gripper")
        self.gripper_open = False
        self._finish(record, self.durations.gripper_s)

    def pick(
        self,
        location: str,
        cell: Optional[Tuple[int, int]] = None,
        yaw_deg: float = 0.0,
        level: int = 0,
        uv: Optional[Tuple[float, float]] = None,
        open_mm: Optional[float] = None,
    ) -> bool:
        location = v.check_location(location)
        cell = v.check_cell(cell)
        args = {"location": location, "cell": cell, "yaw_deg": v.check_yaw(yaw_deg), "level": v.check_level(level),
                "uv": v.check_uv(uv, cell), "open_mm": v.check_opening(open_mm, "open_mm")}
        self._check_site(location, cell)
        record = self._record("pick", args)
        self._check_fingers("pick", location, cell, args["level"], args["yaw_deg"], args["uv"], open_mm)
        kind = self._fault("pick")
        if kind in ("comm_error", "calibration"):
            self._raise(record, kind, "pick")
        if kind == "unreachable":
            self.last_reason = "punct inaccesibil (simulat)"
            self.position_uncertain = True
            return self._finish(record, 0.05, False)
        outcome = self.world.pick(location, cell, level, args["yaw_deg"], args["uv"], force_miss=kind == "grasp_fail",
                                  open_mm=args["open_mm"])
        self.current_pose = None
        if outcome == UNREACHABLE:
            self.last_reason = "punct inaccesibil {}{}".format(location, cell or "")
            self.position_uncertain = True
            return self._finish(record, 0.05, False)
        if kind == "drop" and outcome == GRASPED:
            self.world.drop_held_in_transit()
        self.gripper_open = False
        self.last_reason = None
        self._finish(record, self.durations.pick_s, True)
        if kind == "timeout":  # mișcarea s-a executat, dar răspunsul s-a pierdut: stare necunoscută
            self._raise(record, kind, "pick")
        return True

    def place(self, location: str, cell: Optional[Tuple[int, int]] = None, level: int = 0, yaw_deg: float = 0.0,
              release_mm: Optional[float] = None, uv: Optional[Tuple[float, float]] = None) -> bool:
        location = v.check_location(location)
        cell = v.check_cell(cell)
        args = {"location": location, "cell": cell, "level": v.check_level(level), "yaw_deg": v.check_yaw(yaw_deg),
                "release_mm": v.check_opening(release_mm, "release_mm"), "uv": v.check_uv(uv, cell)}
        self._check_site(location, cell)
        record = self._record("place", args)
        if self.world.held is not None:
            self._check_fingers("place", location, cell, args["level"], args["yaw_deg"], args["uv"], release_mm)
        kind = self._fault("place")
        if kind in ("comm_error", "calibration"):
            self._raise(record, kind, "place")
        if kind == "unreachable":
            self.last_reason = "punct inaccesibil (simulat)"
            self.position_uncertain = True
            return self._finish(record, 0.05, False)
        if kind == "drop":
            self.world.drop_held_in_transit()
        outcome = self.world.place(location, cell, level, args["yaw_deg"], args["uv"])
        self.current_pose = None
        if outcome == UNREACHABLE:
            self.last_reason = "punct inaccesibil {}{}".format(location, cell or "")
            self.position_uncertain = True
            return self._finish(record, 0.05, False)
        self.gripper_open = True
        self.last_reason = None
        self._finish(record, self.durations.place_s, True)
        if kind == "timeout":
            self._raise(record, kind, "place")
        return True

    def read_joints(self) -> List[float]:
        record = self._record("read_joints", {})
        kind = self._fault("read_joints")
        if kind:
            self._raise(record, "comm_error" if kind not in ("timeout", "calibration") else kind, "read_joints")
        return self._finish(record, 0.02, list(self.joints))

    def beep(self, count: int = 1, duration_ms: int = 100) -> None:
        v.check_beep(count, duration_ms)
        record = self._record("beep", {"count": count, "duration_ms": duration_ms})
        self._finish(record, count * self.durations.beep_s)
