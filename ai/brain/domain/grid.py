"""Coordonate logice în caroiaj."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator, Tuple

from brain.domain.errors import DomainValidationError


@dataclass(frozen=True, order=True)
class GridCoord:
    """Poziție logică (row, col) într-un caroiaj; (0,0) = stânga-sus în imaginea redresată."""

    row: int
    col: int

    def __post_init__(self) -> None:
        for name in ("row", "col"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise DomainValidationError("GridCoord.{} trebuie să fie int, nu {!r}".format(name, value))
            if value < 0:
                raise DomainValidationError("GridCoord.{} trebuie să fie >= 0, nu {}".format(name, value))

    def as_tuple(self) -> Tuple[int, int]:
        return (self.row, self.col)

    def within(self, rows: int, cols: int) -> bool:
        return self.row < rows and self.col < cols

    def __str__(self) -> str:
        return "({},{})".format(self.row, self.col)


def iter_coords(rows: int, cols: int) -> Iterator[GridCoord]:
    """Toate coordonatele unui caroiaj, în ordine row-major."""
    if rows <= 0 or cols <= 0:
        raise DomainValidationError("dimensiuni de caroiaj invalide: {}x{}".format(rows, cols))
    for row in range(rows):
        for col in range(cols):
            yield GridCoord(row, col)
