"""Implementările ceasului: real (robot) și manual (simulare deterministă)."""

from __future__ import annotations

import time


class SystemClock:
    def monotonic(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            time.sleep(seconds)


class ManualClock:
    """Timp simulat: `sleep` avansează instant ceasul. Duratele simulate rămân deterministe."""

    def __init__(self, start: float = 0.0) -> None:
        self._now = float(start)

    def monotonic(self) -> float:
        return self._now

    def sleep(self, seconds: float) -> None:
        if seconds < 0:
            raise ValueError("durata nu poate fi negativă")
        self._now += float(seconds)

    def advance(self, seconds: float) -> None:
        self.sleep(seconds)
