"""Logging structurat (JSON Lines): un eveniment pe linie, cu câmpurile cerute de proceduri.

Câmpuri standard: ts, level, logger, event, mode, task, state, action, duration_ms, result, error,
confidence. Logurile nu înlocuiesc starea persistentă (`SceneState`).
"""

from __future__ import annotations

import datetime as _dt
import json
import logging
import sys
from typing import Any, Optional

STANDARD_FIELDS = ("mode", "task", "state", "action", "duration_ms", "result", "error", "confidence")
ROOT_LOGGER = "brain"


class JsonLinesFormatter(logging.Formatter):
    def __init__(self, mode: str) -> None:
        super().__init__()
        self.mode = mode

    def format(self, record: logging.LogRecord) -> str:
        timestamp = _dt.datetime.fromtimestamp(record.created, tz=_dt.timezone.utc)
        payload = {
            "ts": timestamp.isoformat(timespec="milliseconds").replace("+00:00", "Z"),
            "level": record.levelname,
            "logger": record.name,
            "event": record.getMessage(),
            "mode": self.mode,
        }
        payload.update(getattr(record, "fields", {}) or {})
        if record.exc_info:
            payload["error"] = self.formatException(record.exc_info)
        return json.dumps(payload, ensure_ascii=False, default=str)


def configure_logging(level: str = "WARNING", log_file: Optional[str] = None, mode: str = "offline") -> None:
    """Consola primește nivelul cerut; fișierul (dacă e dat) primește INFO și peste."""
    logger = logging.getLogger(ROOT_LOGGER)
    logger.handlers.clear()
    logger.setLevel(logging.DEBUG)
    logger.propagate = False
    formatter = JsonLinesFormatter(mode)
    console = logging.StreamHandler(sys.stderr)
    console.setLevel(getattr(logging, level.upper(), logging.WARNING))
    console.setFormatter(formatter)
    logger.addHandler(console)
    if log_file:
        from pathlib import Path

        Path(log_file).parent.mkdir(parents=True, exist_ok=True)
        handler = logging.FileHandler(log_file, encoding="utf-8")
        handler.setLevel(logging.INFO)
        handler.setFormatter(formatter)
        logger.addHandler(handler)


def log_event(logger: logging.Logger, event: str, level: int = logging.INFO, **fields: Any) -> None:
    logger.log(level, event, extra={"fields": fields})
