"""Randarea unei locații a lumii simulate, văzută din poza ei de observare (camera sintetică).

Aceeași aproximare ca generatorul de scene: masa e desenată de sus, fețele de sus ale cuburilor sunt
deplasate de paralaxă (factor H/(H-h) față de nadir, cu h = înălțimea vârfului stivei), apoi totul e
proiectat în imagine printr-o omografie fixă per poză. Șabloanele vecine de pe foaia A4 pot apărea
parțial în cadru. Culoarea fiecărui cub e stabilă între observări (derivată din identificatorul lui).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from brain.configuration.models import GridGeometry
from brain.simulation.scene_gen import (
    _AMBIGUOUS_HSV,
    _BASE_HSV,
    RenderSettings,
    _Canvas,
    _draw_cubes,
    _draw_grid,
    _hsv_to_bgr,
)
from brain.simulation.sim_world import SimGrid


@dataclass(frozen=True)
class NeighborGrid:
    """Un alt șablon de pe foaie, în coordonatele mm ale grilei randate (colțul stânga-sus al lui)."""

    origin_mm: Tuple[float, float]
    rows: int
    cols: int
    cell_mm: float
    line_width_mm: float


@dataclass(frozen=True)
class SimCamera:
    """Camera unei poze de observare. Valori sintetice, nu calibrări."""

    image_size: Tuple[int, int]
    quad_px: Tuple[Tuple[float, float], ...]  # unde ajung colțurile grilei (centrele liniilor) în imagine
    camera_height_mm: float
    nadir_mm: Tuple[float, float]  # punctul de sub cameră, în mm în cadrul grilei
    gradient: Tuple[float, float]
    cast: Tuple[float, float, float]
    exposure: float
    blur_sigma: float
    noise_sigma: float


def make_camera(
    seed: int,
    geometry: GridGeometry,
    rows: int,
    cols: int,
    camera_height_mm: float,
    nadir_uv: Tuple[float, float],
    image_size: Tuple[int, int] = (640, 480),
) -> SimCamera:
    """Cameră plauzibilă pentru o poză care centrează șablonul: rotație și perspectivă mici."""
    rng = np.random.default_rng(seed)
    width, height = image_size
    side = float(rng.uniform(230.0, 300.0))
    gw, gh = side * cols / max(rows, cols), side * rows / max(rows, cols)
    theta = math.radians(float(rng.uniform(-8.0, 8.0)))
    center = np.array([width / 2.0, height / 2.0]) + rng.uniform(-20.0, 20.0, size=2)
    c, s = math.cos(theta), math.sin(theta)
    quad = []
    for u, v in ((-gw / 2, -gh / 2), (gw / 2, -gh / 2), (gw / 2, gh / 2), (-gw / 2, gh / 2)):
        jitter = rng.uniform(-0.025, 0.025, size=2) * side
        quad.append((float(center[0] + u * c + v * s + jitter[0]), float(center[1] - u * s + v * c + jitter[1])))
    return SimCamera(
        image_size=image_size,
        quad_px=tuple(quad),
        camera_height_mm=float(camera_height_mm),
        nadir_mm=(nadir_uv[0] * geometry.cell_mm, nadir_uv[1] * geometry.cell_mm),
        gradient=tuple(float(g) for g in rng.uniform(-0.08, 0.08, size=2)),  # type: ignore[arg-type]
        cast=tuple(float(g) for g in rng.uniform(0.95, 1.05, size=3)),  # type: ignore[arg-type]
        exposure=float(rng.uniform(0.94, 1.03)),
        blur_sigma=float(rng.uniform(0.0, 0.6)),
        noise_sigma=float(rng.uniform(1.5, 3.0)),
    )


def cube_bgr(color: str, uid: int) -> Tuple[int, int, int]:
    rng = np.random.default_rng(10007 * uid + 17)
    if color in _BASE_HSV:
        h, s, v = _BASE_HSV[color]
        return _hsv_to_bgr(h + int(rng.integers(-3, 4)), s + int(rng.integers(-20, 21)), v + int(rng.integers(-15, 16)))
    h, s, v = _AMBIGUOUS_HSV.get(color, (None, 0, 200))
    return _hsv_to_bgr(int(rng.integers(0, 180)) if h is None else h, s, v)


def render_location(
    grid: SimGrid,
    geometry: GridGeometry,
    camera: SimCamera,
    frames: int,
    rng: np.random.Generator,
    neighbors: Sequence[NeighborGrid] = (),
    settings: Optional[RenderSettings] = None,
) -> List[np.ndarray]:
    settings = settings or RenderSettings()
    table = np.array([205, 208, 210], dtype=np.float32)
    canvas = _Canvas(geometry, grid.rows, grid.cols, settings, table)
    _draw_grid(canvas)
    for neighbor in neighbors:
        _draw_neighbor(canvas, neighbor)

    cube_mm = geometry.cube_mm
    drawn = []
    ks = []
    for (row, col), stack in sorted(grid.stacks.items()):
        if not stack:
            continue
        top_cube = stack[-1]
        center = ((col + 0.5) * geometry.cell_mm + top_cube.offset_mm[0], (row + 0.5) * geometry.cell_mm + top_cube.offset_mm[1])
        bgr = cube_bgr(top_cube.color, top_cube.uid)
        drawn.append((row, col, top_cube.yaw_deg, center, top_cube.face, bgr, False, top_cube.icon_shape))
        top_height = len(stack) * cube_mm
        ks.append(camera.camera_height_mm / (camera.camera_height_mm - top_height))
    if drawn:
        _draw_cubes(canvas, drawn, np.array(camera.nadir_mm), ks[0], settings, ks=ks)
    canvas.image = cv2.GaussianBlur(canvas.image, (0, 0), 0.9)

    grid_mm = np.array([[0, 0], [grid.cols * geometry.cell_mm, 0], [grid.cols * geometry.cell_mm, grid.rows * geometry.cell_mm],
                        [0, grid.rows * geometry.cell_mm]])
    homography = cv2.getPerspectiveTransform(canvas.to_px(grid_mm).astype(np.float32), np.array(camera.quad_px, dtype=np.float32))
    width, height = camera.image_size
    base = cv2.warpPerspective(canvas.image, homography, (width, height), flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=tuple(float(c) for c in table))
    ys, xs = np.mgrid[0:height, 0:width].astype(np.float32)
    field = 1.0 + camera.gradient[0] * (xs - width / 2.0) / width + camera.gradient[1] * (ys - height / 2.0) / height
    base = base * field[..., None] * np.array(camera.cast, dtype=np.float32).reshape(1, 1, 3) * camera.exposure
    if camera.blur_sigma > 0.3:
        base = cv2.GaussianBlur(base, (0, 0), camera.blur_sigma)
    result = []
    for _ in range(frames):
        noisy = base + rng.normal(0.0, camera.noise_sigma, size=base.shape).astype(np.float32)
        result.append(np.clip(np.rint(noisy), 0, 255).astype(np.uint8))
    return result


def _draw_neighbor(canvas: _Canvas, neighbor: NeighborGrid) -> None:
    w = neighbor.line_width_mm / 2.0
    ox, oy = neighbor.origin_mm
    width_mm, height_mm = neighbor.cols * neighbor.cell_mm, neighbor.rows * neighbor.cell_mm
    ink = (32.0, 32.0, 34.0)
    for k in range(neighbor.cols + 1):
        x = ox + k * neighbor.cell_mm
        canvas.fill(np.array([[x - w, oy - w], [x + w, oy - w], [x + w, oy + height_mm + w], [x - w, oy + height_mm + w]]), ink)
    for k in range(neighbor.rows + 1):
        y = oy + k * neighbor.cell_mm
        canvas.fill(np.array([[ox - w, y - w], [ox + width_mm + w, y - w], [ox + width_mm + w, y + w], [ox - w, y + w]]), ink)
