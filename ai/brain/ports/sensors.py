"""Porturile pentru senzori și timp: proximitate, cameră, ceas."""

from __future__ import annotations

from typing import Any, List

from brain.domain.proximity import ProximityReading, SensorSpec

try:
    from typing import Protocol
except ImportError:  # pragma: no cover
    from typing_extensions import Protocol  # type: ignore


class ProximitySensorInterface(Protocol):
    """Senzor de distanță. `read()` nu aruncă excepții pentru probleme de măsurare: le raportează în
    `ProximityReading.validity` (timeout, deconectare, în afara domeniului, date invalide)."""

    def spec(self) -> SensorSpec:
        ...

    def read(self) -> ProximityReading:
        ...


class FrameSource(Protocol):
    """Sursă de cadre BGR uint8 (cameră USB, fișiere, cameră sintetică)."""

    def grab(self, count: int) -> List[Any]:
        ...


class Clock(Protocol):
    """Ceas injectabil: timp real pe robot, timp simulat în teste (bugetul de 5 minute rulează instant)."""

    def monotonic(self) -> float:
        ...

    def sleep(self, seconds: float) -> None:
        ...
