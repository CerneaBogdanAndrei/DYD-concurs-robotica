"""Acces validat la dicționare YAML, cu mesaje care numesc fișierul, cheia și valoarea."""

from __future__ import annotations

import math
from typing import Any, Iterable, List, Mapping, Optional, Sequence

from brain.domain.errors import ConfigError

_MISSING = object()


class Section:
    """Vedere asupra unei secțiuni YAML care își cunoaște fișierul și calea completă."""

    def __init__(self, data: Any, file: str, path: str = "") -> None:
        if not isinstance(data, Mapping):
            raise ConfigError(file, path or "<rădăcină>", data, "trebuie să fie un dicționar")
        self.data = data
        self.file = file
        self.path = path

    def key_path(self, key: str) -> str:
        return "{}.{}".format(self.path, key) if self.path else key

    def error(self, key: str, value: Any, message: str) -> ConfigError:
        return ConfigError(self.file, self.key_path(key), value, message)

    def raw(self, key: str, default: Any = _MISSING) -> Any:
        if key in self.data:
            return self.data[key]
        if default is _MISSING:
            raise self.error(key, None, "cheie obligatorie lipsă")
        return default

    def has(self, key: str) -> bool:
        return key in self.data

    def section(self, key: str, default: Any = _MISSING) -> "Section":
        value = self.raw(key, default)
        return Section(value, self.file, self.key_path(key))

    def number(
        self,
        key: str,
        default: Any = _MISSING,
        minimum: Optional[float] = None,
        maximum: Optional[float] = None,
        exclusive_minimum: bool = False,
    ) -> float:
        value = self.raw(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise self.error(key, value, "trebuie să fie un număr finit")
        check_range(self, key, value, minimum, maximum, exclusive_minimum)
        return float(value)

    def integer(
        self, key: str, default: Any = _MISSING, minimum: Optional[int] = None, maximum: Optional[int] = None
    ) -> int:
        value = self.raw(key, default)
        if isinstance(value, bool) or not isinstance(value, int):
            raise self.error(key, value, "trebuie să fie un număr întreg")
        check_range(self, key, value, minimum, maximum, False)
        return int(value)

    def boolean(self, key: str, default: Any = _MISSING) -> bool:
        value = self.raw(key, default)
        if not isinstance(value, bool):
            raise self.error(key, value, "trebuie să fie true sau false")
        return value

    def string(self, key: str, default: Any = _MISSING, choices: Optional[Sequence[str]] = None) -> str:
        value = self.raw(key, default)
        if not isinstance(value, str) or not value:
            raise self.error(key, value, "trebuie să fie un șir nevid")
        if choices is not None and value not in choices:
            raise self.error(key, value, "valoare permisă: {}".format(", ".join(choices)))
        return value

    def optional_string(self, key: str) -> Optional[str]:
        value = self.raw(key, None)
        if value is None:
            return None
        return self.string(key)

    def sequence(self, key: str, default: Any = _MISSING, min_length: int = 0) -> List[Any]:
        value = self.raw(key, default)
        if not isinstance(value, (list, tuple)):
            raise self.error(key, value, "trebuie să fie o listă")
        if len(value) < min_length:
            raise self.error(key, value, "lista trebuie să aibă cel puțin {} elemente".format(min_length))
        return list(value)

    def reject_unknown(self, allowed: Iterable[str]) -> None:
        """Cheile necunoscute sunt aproape mereu greșeli de scriere; le respingem explicit."""
        allowed_set = set(allowed)
        for key in self.data:
            if key not in allowed_set:
                raise self.error(str(key), self.data[key], "cheie necunoscută (permise: {})".format(", ".join(sorted(allowed_set))))


def check_range(
    section: Section,
    key: str,
    value: float,
    minimum: Optional[float],
    maximum: Optional[float],
    exclusive_minimum: bool,
) -> None:
    if minimum is not None:
        if exclusive_minimum and not value > minimum:
            raise section.error(key, value, "trebuie să fie > {}".format(minimum))
        if not exclusive_minimum and value < minimum:
            raise section.error(key, value, "trebuie să fie >= {}".format(minimum))
    if maximum is not None and value > maximum:
        raise section.error(key, value, "trebuie să fie <= {}".format(maximum))
