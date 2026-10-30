"""Sortarea din pagina previzualizării: „Fă poza” (pasul 1, harta culorilor) și „Start” (pașii 2-4 la rând) pornesc
scriptul de sortare (`scripts/calib_live/autocal/sortare_cuburi.py`) ca proces separat, cu Python-ul acestui proces.

Pagina citește starea la ~1,5 s: dacă rulează, jurnalul (ultimele linii), harta din pasul 1 (`harta_culori.json` și
poza ei, `harta_culori.jpg`) și celulele puse (`sortare_stare.json`). „Oprește” trimite SIGINT scriptului: comanda
curentă a brațului se termină în serviciu, apoi scriptul se oprește. O singură rulare odată.
"""

from __future__ import annotations

import json
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

from brain.adapters.preview_server import ApiError

ACTIONS = {"poza": "pas1", "start": "tot"}  # butonul din pagină -> pasul scriptului
TEXT = {"poza": "pasul 1: poza și harta culorilor", "start": "pașii 2-4: cuburile din afară puse pe caroiaj"}
LOG_LINES = 60


def default_script() -> Path:
    """Scriptul de sortare din acest repo (ai/scripts/calib_live/autocal/sortare_cuburi.py)."""
    return Path(__file__).resolve().parents[2] / "scripts" / "calib_live" / "autocal" / "sortare_cuburi.py"


def _read_json(path: Path) -> Optional[Dict[str, Any]]:
    try:
        with open(path, encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return None  # lipsește sau e scris chiar acum


class SortingRunner:
    """Rulează scriptul de sortare pentru butoanele paginii și îi raportează starea."""

    def __init__(self, script: Path, log_path: Path, python: str = sys.executable) -> None:
        self.script = Path(script)
        self.folder = self.script.parent
        self.log_path = Path(log_path)
        self.python = python
        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._action: Optional[str] = None
        self._started: Optional[float] = None
        self._ended: Optional[float] = None
        self._code: Optional[int] = None
        self._stopping = False

    def _running(self) -> bool:
        if self._proc is None:
            return False
        code = self._proc.poll()
        if code is None:
            return True
        if self._ended is None:
            self._code, self._ended = code, time.time()
        return False

    def post(self, action: str) -> Dict[str, Any]:
        """Ruta POST /sortare/<action>: `poza`, `start` sau `stop`."""
        if action == "stop":
            return self.stop()
        return self.start(action)

    def start(self, action: str) -> Dict[str, Any]:
        if action not in ACTIONS:
            raise ApiError(404, "acțiune necunoscută: {}".format(action))
        with self._lock:
            if self._running():
                raise ApiError(409, "rulează deja: {}".format(TEXT[self._action or "start"]))
            if not self.script.exists():
                raise ApiError(500, "lipsește scriptul de sortare: {}".format(self.script))
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.log_path, "w", encoding="utf-8") as log:
                self._proc = subprocess.Popen(
                    [self.python, "-u", str(self.script), ACTIONS[action]], cwd=str(self.folder), stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True)
            self._action, self._started, self._ended, self._code, self._stopping = action, time.time(), None, None, False
        return self.status()

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            if self._running() and self._proc is not None:
                self._proc.send_signal(signal.SIGINT)
                self._stopping = True
        return self.status()

    def _log_tail(self) -> list:
        try:
            with open(self.log_path, encoding="utf-8", errors="replace") as handle:
                return handle.read().splitlines()[-LOG_LINES:]
        except OSError:
            return []

    def status(self) -> Dict[str, Any]:
        with self._lock:
            running = self._running()
            run = None if self._action is None else {
                "action": self._action, "text": TEXT[self._action], "running": running, "stopping": self._stopping,
                "started_at": self._started, "ended_at": self._ended, "exit_code": self._code}
        harta = _read_json(self.folder / "harta_culori.json")
        stare = _read_json(self.folder / "sortare_stare.json") or {}
        return {
            "running": running,
            "run": run,
            "log": self._log_tail(),
            "map": None if harta is None else {"at": harta.get("at"), "cells": harta.get("cells", {}),
                                               "image": (self.folder / "harta_culori.jpg").exists()},
            "filled": {k: {"color": v.get("color"), "good": v.get("good")} for k, v in (stare.get("filled") or {}).items()},
        }

    def map_image(self) -> Optional[bytes]:
        try:
            return (self.folder / "harta_culori.jpg").read_bytes()
        except OSError:
            return None
