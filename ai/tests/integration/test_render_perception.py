"""Randarea unei locații (cu paralaxă, opțional cu șabloane vecine) → viziunea reală: grilă, celule, precizie uv."""

import numpy as np
import pytest

from brain.domain.cube import Occupancy
from brain.domain.plan import correct_parallax
from brain.perception.pipeline import VisionPipeline
from brain.simulation.render import NeighborGrid, make_camera, render_location
from brain.simulation.sim_world import SimCube, SimGrid

COLORS = ("red", "yellow", "green", "blue")


def _scene(system, name, seed, neighbors=False):
    loc = system.location(name)
    rng = np.random.default_rng(1000 + seed)
    grid = SimGrid(name, 3, 3, loc.cell_mm)
    uid = 1
    max_off = max(0.0, (loc.cell_mm - 30.0) / 2.0 - 1.0)
    for r in range(3):
        for c in range(3):
            if rng.random() < 0.45:
                continue
            for _ in range(2 if rng.random() < 0.15 else 1):
                grid.stack((r, c)).append(SimCube(uid, str(rng.choice(COLORS)), "full" if rng.random() < 0.6 else "icon",
                                                  float(rng.uniform(-12, 12)),
                                                  tuple(float(v) for v in rng.uniform(-max_off, max_off, size=2))))
                uid += 1
    items = []
    if neighbors:
        others = [system.location(n) for n in ("small", "medium", "large") if n != name]
        gap = float(rng.uniform(12.0, 25.0))
        items = [NeighborGrid((-gap - 3 * others[0].cell_mm, 0.0), 3, 3, others[0].cell_mm, others[0].line_width_mm),
                 NeighborGrid((3 * loc.cell_mm + gap, 0.0), 3, 3, others[1].cell_mm, others[1].line_width_mm)]
    camera = make_camera(seed, system.grid_geometry(name), 3, 3, loc.camera_height_mm, loc.nadir_uv)
    frames = render_location(grid, system.grid_geometry(name), camera, 5, np.random.default_rng(seed), neighbors=items)
    return grid, camera, frames


def _check(system, pipeline, name, seed, neighbors=False):
    loc = system.location(name)
    grid, camera, frames = _scene(system, name, seed, neighbors)
    obs = pipeline.analyze(frames, name).observation
    assert obs.actionable, obs.reasons
    corner = float(np.max(np.linalg.norm(np.array(obs.geometry.corners_px) - np.array(camera.quad_px), axis=1)))
    assert corner <= 4.0, "colț {:.1f}px".format(corner)
    errors = []
    for cell in obs.cells:
        stack = grid.stacks.get((cell.coord.row, cell.coord.col)) or []
        if not stack:
            assert cell.occupancy is Occupancy.EMPTY, cell
            continue
        top = stack[-1]
        assert cell.occupancy is Occupancy.OCCUPIED and cell.color.value == top.color, (cell.coord, cell.color, top.color)
        uv = correct_parallax(cell.uv_top, len(stack) * 30.0, loc.camera_height_mm, loc.nadir_uv)
        true_uv = (cell.coord.col + 0.5 + top.offset_mm[0] / loc.cell_mm, cell.coord.row + 0.5 + top.offset_mm[1] / loc.cell_mm)
        errors.append(float(np.hypot(uv[0] - true_uv[0], uv[1] - true_uv[1])) * loc.cell_mm)
    return errors


@pytest.fixture(scope="module")
def pipeline(app_config):
    return VisionPipeline(app_config.system, app_config.colors)


@pytest.mark.parametrize("name", ["small", "medium", "large"])
def test_three_sizes_single_sheet(app_config, pipeline, name):
    errors = []
    for seed in (2, 3, 4):
        errors += _check(app_config.system, pipeline, name, seed)
    assert errors and max(errors) < 7.0  # toleranța de prindere a simulării e 7 mm
    assert float(np.median(errors)) < 1.0


@pytest.mark.parametrize("name, seed", [("large", 2), ("large", 3), ("small", 2)])
def test_neighbor_templates_do_not_confuse_the_grid(app_config, pipeline, name, seed):
    _check(app_config.system, pipeline, name, seed, neighbors=True)
