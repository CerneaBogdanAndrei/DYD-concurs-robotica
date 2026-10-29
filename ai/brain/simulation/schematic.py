"""Randarea schematică 2D a lumii simulate (vedere de sus) și animația GIF a unei rulări.

Fiecare șablon e desenat ca grilă; o celulă arată culoarea cubului din vârf, înălțimea stivei și benzi
cu culorile nivelurilor de dedesubt. Zonele hărții sunt pătrate sub șabloane. Desenul folosește OpenCV
(dependență existentă); GIF-ul cere Pillow (opțional, extra `gif`), altfel se salvează doar PNG-uri.
Textul e doar ASCII (`cv2.putText`).
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Sequence

import cv2
import numpy as np

from brain.simulation.sim_world import SimWorld

CELL_PX = 46
GAP_PX = 28
MARGIN_PX = 16
HEADER_PX = 34
ZONE_PX = 54
_BGR = {
    "red": (40, 40, 220), "yellow": (40, 210, 235), "green": (60, 170, 60), "blue": (200, 110, 40),
    "orange": (40, 140, 245), "desaturated": (170, 170, 170),
}
_BACKGROUND = (238, 238, 236)
_INK = (40, 40, 40)


def _color(name: str):
    return _BGR.get(name, (150, 150, 150))


def render_schematic(world: SimWorld, caption: str = "", highlight: Sequence[str] = ()) -> np.ndarray:
    """Imaginea BGR a stării curente. `highlight`: chei „loc:r,c” sau nume de zone, conturate cu roșu."""
    grids = list(world.grids.values())
    width = MARGIN_PX * 2 + sum(g.cols * CELL_PX for g in grids) + GAP_PX * max(0, len(grids) - 1)
    rows_px = max((g.rows * CELL_PX for g in grids), default=0)
    zone_band = ZONE_PX + 30 if world.zones else 0
    height = HEADER_PX + rows_px + 24 + zone_band + MARGIN_PX
    width = max(width, MARGIN_PX * 2 + len(world.zones) * (ZONE_PX + 16))
    image = np.full((height, width, 3), _BACKGROUND, dtype=np.uint8)
    cv2.putText(image, _ascii(caption)[:90], (MARGIN_PX, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _INK, 1, cv2.LINE_AA)
    x0 = MARGIN_PX
    marked = set(highlight)
    for grid in grids:
        y0 = HEADER_PX
        for r in range(grid.rows):
            for c in range(grid.cols):
                x, y = x0 + c * CELL_PX, y0 + r * CELL_PX
                cv2.rectangle(image, (x, y), (x + CELL_PX, y + CELL_PX), _INK, 1)
                stack = grid.stacks.get((r, c)) or []
                _draw_stack(image, x, y, CELL_PX, [cube.color for cube in stack])
                if "{}:{},{}".format(grid.name, r, c) in marked:
                    cv2.rectangle(image, (x + 1, y + 1), (x + CELL_PX - 1, y + CELL_PX - 1), (0, 0, 255), 2)
        cv2.putText(image, grid.name, (x0, y0 + grid.rows * CELL_PX + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, _INK, 1,
                    cv2.LINE_AA)
        x0 += grid.cols * CELL_PX + GAP_PX
    if world.zones:
        y = HEADER_PX + rows_px + 28
        for index, zone in enumerate(world.zones.values()):
            x = MARGIN_PX + index * (ZONE_PX + 16)
            cv2.rectangle(image, (x, y), (x + ZONE_PX, y + ZONE_PX), _INK, 1)
            _draw_stack(image, x, y, ZONE_PX, [cube.color for cube in zone.stack])
            if zone.name in marked:
                cv2.rectangle(image, (x + 1, y + 1), (x + ZONE_PX - 1, y + ZONE_PX - 1), (0, 0, 255), 2)
            cv2.putText(image, zone.name.replace("zone_", "z_"), (x, y + ZONE_PX + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        _INK, 1, cv2.LINE_AA)
    if world.held is not None:
        cv2.putText(image, "in gheara: {}".format(world.held.color), (width - 170, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    _color(world.held.color), 1, cv2.LINE_AA)
    return image


def _draw_stack(image: np.ndarray, x: int, y: int, size: int, colors: List[str]) -> None:
    if not colors:
        return
    pad = 6
    cv2.rectangle(image, (x + pad, y + pad), (x + size - pad, y + size - pad - 8), _color(colors[-1]), -1)
    band = max(2, (size - 2 * pad) // max(1, len(colors)))
    for index, name in enumerate(colors):  # benzi de jos în sus: nivelurile ascunse
        bx = x + pad + index * band
        cv2.rectangle(image, (bx, y + size - pad - 6), (min(bx + band - 1, x + size - pad), y + size - pad), _color(name), -1)
    if len(colors) > 1:
        cv2.putText(image, str(len(colors)), (x + size // 2 - 5, y + size // 2 + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(image, str(len(colors)), (x + size // 2 - 5, y + size // 2 + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    _INK, 1, cv2.LINE_AA)


def _ascii(text: str) -> str:
    table = str.maketrans("ăâîșşțţĂÂÎȘŞȚŢ", "aaissttAAISSTT")
    return text.translate(table).encode("ascii", "replace").decode("ascii")


def save_animation(frames: Sequence[np.ndarray], path: Path, frame_ms: int = 700) -> Path:
    """GIF cu Pillow dacă e instalat; altfel PNG-uri numerotate lângă `path`. Întoarce calea scrisă."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not frames:
        raise ValueError("nicio imagine pentru animație")
    try:
        from PIL import Image  # type: ignore
    except ImportError:
        folder = path.with_suffix("")
        folder.mkdir(parents=True, exist_ok=True)
        for index, frame in enumerate(frames):
            cv2.imwrite(str(folder / "frame_{:03d}.png".format(index)), frame)
        return folder
    height = max(f.shape[0] for f in frames)
    width = max(f.shape[1] for f in frames)
    images = []
    for frame in frames:
        canvas = np.full((height, width, 3), _BACKGROUND, dtype=np.uint8)
        canvas[: frame.shape[0], : frame.shape[1]] = frame
        images.append(Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)))
    durations = [frame_ms] * len(images)
    durations[-1] = frame_ms * 4
    images[0].save(str(path), save_all=True, append_images=images[1:], duration=durations, loop=0)
    return path


class Recorder:
    """Colectează imagini schematice în timpul unei rulări (apelat de FSM la fiecare pas)."""

    def __init__(self, world: SimWorld, title: str, limit: int = 200) -> None:
        self.world = world
        self.title = title
        self.limit = limit
        self.frames: List[np.ndarray] = [render_schematic(world, "{}: start".format(title))]

    def __call__(self, event: str, move: Optional[object]) -> None:
        if len(self.frames) >= self.limit or event != "verified" or move is None:
            return
        pick = getattr(move, "pick")
        place = getattr(move, "place")
        marks = [_mark(pick.slot), _mark(place.slot)]
        caption = "{}: {} {} -> {}".format(self.title, getattr(move, "cube").color.value, pick.slot, place.slot)
        self.frames.append(render_schematic(self.world, caption, marks))

    def finish(self, caption: str) -> None:
        self.frames.append(render_schematic(self.world, caption))


def _mark(slot: object) -> str:
    cell = getattr(slot, "cell")
    location = getattr(slot, "location")
    return location if cell is None else "{}:{},{}".format(location, cell.row, cell.col)
