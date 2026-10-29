"""ArmSession: toate comenzile către braț trec pe aici (verificarea modului, durate, log, oprire la cerere).

- la pornire verifică modul: simularea nu poate fi confundată cu brațul real (și invers);
- fiecare comandă e cronometrată pe ceasul injectat și scrisă în jurnalul de durate (`report timings`);
- o cerere de oprire (Ctrl+C) e respectată ÎNTRE comenzi: comanda curentă se termină, nu se trimite `home`.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from brain.domain.errors import ArmError, SafetyRejected
from brain.logging_setup import log_event
from brain.ports.arm import ArmInterface
from brain.ports.sensors import Clock

logger = logging.getLogger("brain.orchestration.session")


class StopRequested(ArmError):
    """Utilizatorul a cerut oprirea (Ctrl+C): nicio comandă nouă, fără `home` automat."""


@dataclass
class DurationRecord:
    method: str
    duration_s: float
    result: str
    simulated: bool
    detail: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {"method": self.method, "duration_s": round(self.duration_s, 4), "result": self.result,
                "simulated": self.simulated, "detail": self.detail}


class DurationJournal:
    """Durata fiecărei comenzi și observări. Opțional scris în fișier JSON Lines (o linie per înregistrare)."""

    def __init__(self, path: Optional[Path] = None, simulated: bool = True) -> None:
        self.path = Path(path) if path is not None else None
        self.simulated = simulated
        self.records: List[DurationRecord] = []
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)

    def add(self, method: str, duration_s: float, result: str, detail: str = "") -> None:
        record = DurationRecord(method, float(duration_s), result, self.simulated, detail)
        self.records.append(record)
        if self.path is not None:
            with open(self.path, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(record.to_dict(), ensure_ascii=False) + "\n")


class ArmSession:
    def __init__(
        self,
        arm: ArmInterface,
        clock: Clock,
        journal: DurationJournal,
        expected_mode: str,
        stop_requested: Callable[[], bool] = lambda: False,
        task: str = "",
    ) -> None:
        self.arm = arm
        self.clock = clock
        self.journal = journal
        self.expected_mode = expected_mode
        self.stop_requested = stop_requested
        self.task = task
        self.commands = 0
        self.status: Dict[str, Any] = {}

    def connect(self) -> Dict[str, Any]:
        """Citește starea serviciului (dacă adaptorul o oferă) și refuză un mod diferit de cel cerut."""
        connect = getattr(self.arm, "connect", None)
        if callable(connect):
            self.status = connect() or {}
        mode = getattr(self.arm, "mode", None)
        if mode != self.expected_mode:
            raise SafetyRejected(
                "brațul raportează modul '{}', dar rularea cere '{}' (simulare vs. braț real); oprit înainte de orice mișcare".format(
                    mode, self.expected_mode))
        log_event(logger, "arm_connected", task=self.task, action="connect", result=mode)
        return self.status

    @property
    def position_uncertain(self) -> bool:
        return bool(getattr(self.arm, "position_uncertain", False))

    @property
    def last_reason(self) -> Optional[str]:
        return getattr(self.arm, "last_reason", None)

    def call(self, method: str, *args: Any, **kwargs: Any) -> Any:
        if self.stop_requested():
            raise StopRequested("oprire cerută de utilizator înainte de {}".format(method))
        started = self.clock.monotonic()
        detail = ", ".join([repr(a) for a in args] + ["{}={!r}".format(k, v) for k, v in kwargs.items()])
        try:
            result = getattr(self.arm, method)(*args, **kwargs)
        except ArmError as exc:
            duration = self.clock.monotonic() - started
            self.journal.add(method, duration, type(exc).__name__, detail)
            log_event(logger, "arm_command", level=logging.WARNING, task=self.task, state="EXECUTE", action=method,
                      duration_ms=round(duration * 1000, 1), result="error", error="{}: {}".format(type(exc).__name__, exc))
            raise
        duration = self.clock.monotonic() - started
        self.commands += 1
        outcome = "ok" if result is None or result is True else ("false" if result is False else "ok")
        self.journal.add(method, duration, outcome, detail)
        log_event(logger, "arm_command", task=self.task, state="EXECUTE", action=method, duration_ms=round(duration * 1000, 1),
                  result=outcome, detail=detail, reason=self.last_reason if result is False else None)
        return result

    # Scurtături, ca orchestrarea să citească natural.
    def go_pose(self, name: str, speed: str = "fast") -> None:
        self.call("go_pose", name, speed)

    def pick(self, location: str, cell: Any, yaw_deg: float, level: int, uv: Any, open_mm: Optional[float] = None) -> bool:
        extra = {} if open_mm is None else {"open_mm": open_mm}
        return bool(self.call("pick", location, cell, yaw_deg=yaw_deg, level=level, uv=uv, **extra))

    def place(self, location: str, cell: Any, level: int, yaw_deg: float = 0.0, release_mm: Optional[float] = None,
              uv: Any = None) -> bool:
        extra: Dict[str, Any] = {} if release_mm is None else {"release_mm": release_mm}
        if uv is not None:
            extra["uv"] = uv
        return bool(self.call("place", location, cell, level=level, yaw_deg=yaw_deg, **extra))

    def beep(self, count: int = 1) -> None:
        self.call("beep", count)


class TimeBudget:
    """Bugetul de timp al sarcinii (implicit 5 minute), pe ceasul injectat: în simulare trece instant."""

    def __init__(self, clock: Clock, limit_s: float, reserve_s: float) -> None:
        self.clock = clock
        self.limit_s = float(limit_s)
        self.reserve_s = float(reserve_s)
        self.started = clock.monotonic()

    def elapsed(self) -> float:
        return self.clock.monotonic() - self.started

    def remaining(self) -> float:
        return self.limit_s - self.elapsed()

    def can_afford(self, estimated_s: float) -> bool:
        """O mutare nouă începe doar dacă se termină (estimat) înainte de limită, cu rezerva păstrată."""
        return self.elapsed() + estimated_s + self.reserve_s <= self.limit_s

    def exceeded(self) -> bool:
        return self.elapsed() > self.limit_s
