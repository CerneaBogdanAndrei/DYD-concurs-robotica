"""Stocarea atomică a `SceneState` într-un fișier JSON.

Scrierea: fișier temporar în același director → flush + fsync → `os.replace` (atomic pe același
volum). Un fișier existent nu e suprascris fără `overwrite=True`, iar un fișier existent invalid
(corupt sau cu altă versiune de schemă) nu e suprascris fără `replace_invalid=True`: poate conține
memoria unei probe care trebuie recuperată manual.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any

from brain.domain.errors import (
    SceneStateCorrupt,
    SceneStateError,
    SceneStateExists,
    SceneStateIncompatible,
    SceneStateNotFound,
)
from brain.domain.scene_state import SceneState
from brain.memory.codec import scene_from_dict, scene_to_dict

_REPLACE_RETRIES = 5
_REPLACE_DELAY_S = 0.05


class SceneStore:
    def __init__(self, path: Any) -> None:
        self.path = Path(path)

    def exists(self) -> bool:
        return self.path.is_file()

    def load(self) -> SceneState:
        if not self.path.is_file():
            raise SceneStateNotFound("fișierul de stare nu există: {}".format(self.path))
        try:
            text = self.path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError) as exc:
            raise SceneStateCorrupt("{}: nu poate fi citit: {}".format(self.path, exc)) from None
        try:
            data = json.loads(text)
        except json.JSONDecodeError as exc:
            raise SceneStateCorrupt("{}: JSON invalid: {}".format(self.path, exc)) from None
        return scene_from_dict(data, source=str(self.path))

    def save(self, state: SceneState, overwrite: bool = False, replace_invalid: bool = False) -> Path:
        if self.path.exists():
            if not overwrite:
                raise SceneStateExists(
                    "{} există deja; folosește overwrite=True pentru a-l înlocui".format(self.path)
                )
            try:
                self.load()
            except (SceneStateCorrupt, SceneStateIncompatible) as exc:
                if not replace_invalid:
                    raise type(exc)(
                        "refuz să suprascriu un fișier de stare invalid (poate conține date de recuperat): {}".format(exc)
                    ) from None
        try:
            payload = json.dumps(scene_to_dict(state), ensure_ascii=False, indent=2, allow_nan=False)
        except ValueError as exc:
            raise SceneStateError("starea nu poate fi serializată: {}".format(exc)) from None
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(".{}.{}.tmp".format(self.path.name, os.getpid()))
        try:
            with open(temporary, "w", encoding="utf-8", newline="\n") as handle:
                handle.write(payload)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            self._replace(temporary)
            self._fsync_directory()
        finally:
            if temporary.exists():
                try:
                    temporary.unlink()
                except OSError:
                    pass
        return self.path

    def _replace(self, temporary: Path) -> None:
        # Pe Windows, os.replace eșuează temporar dacă alt proces (antivirus, editor) ține fișierul deschis.
        for attempt in range(_REPLACE_RETRIES):
            try:
                os.replace(str(temporary), str(self.path))
                return
            except PermissionError:
                if attempt == _REPLACE_RETRIES - 1:
                    raise
                time.sleep(_REPLACE_DELAY_S)

    def _fsync_directory(self) -> None:
        if os.name != "posix":
            return
        fd = os.open(str(self.path.parent), os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
