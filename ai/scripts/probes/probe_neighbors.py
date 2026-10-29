"""Sondă: render_location (cu șabloane vecine) → VisionPipeline, pe small/medium/large.

Verifică: grila corectă aleasă (eroare colț), ocupare + culoare per celulă, eroarea uv (mm) după
corecția de paralaxă. Stive de 1-2 cuburi incluse (paralaxă mai mare la vârf).
"""
import sys

import numpy as np

from brain.configuration.loader import default_config_dir, load_app_config
from brain.domain.cube import Occupancy
from brain.domain.plan import correct_parallax
from brain.perception.pipeline import VisionPipeline
from brain.simulation.render import NeighborGrid, make_camera, render_location
from brain.simulation.sim_world import SimCube, SimGrid

COLORS = ("red", "yellow", "green", "blue")
cfg = load_app_config(default_config_dir())
system = cfg.system
pipe = VisionPipeline(system, cfg.colors)
seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 12
total = ok_all = 0
uv_errors = {}
for name in ("small", "medium", "large"):
    loc = system.location(name)
    geometry = system.grid_geometry(name)
    size = 3 * loc.cell_mm
    others = [system.location(n) for n in ("small", "medium", "large") if n != name]
    for seed in range(seeds):
        rng = np.random.default_rng(1000 + seed)
        grid = SimGrid(name, 3, 3, loc.cell_mm)
        uid = 1
        max_off = max(0.0, (loc.cell_mm - 30.0) / 2.0 - 1.0)
        for r in range(3):
            for c in range(3):
                if rng.random() < 0.45:
                    continue
                height = 2 if rng.random() < 0.15 else 1
                for _ in range(height):
                    grid.stack((r, c)).append(SimCube(uid, str(rng.choice(COLORS)), "full" if rng.random() < 0.6 else "icon",
                                                      float(rng.uniform(-12, 12)),
                                                      tuple(float(v) for v in rng.uniform(-max_off, max_off, size=2))))
                    uid += 1
        # Vecini: celelalte două șabloane, la stânga și la dreapta, cu un spațiu de 12-25 mm.
        gap = float(rng.uniform(12.0, 25.0))
        left, right = others
        neighbors = [
            NeighborGrid((-gap - 3 * left.cell_mm, float(rng.uniform(-20, 20))), 3, 3, left.cell_mm, left.line_width_mm),
            NeighborGrid((size + gap, float(rng.uniform(-20, 20))), 3, 3, right.cell_mm, right.line_width_mm),
        ]
        camera = make_camera(seed, geometry, 3, 3, loc.camera_height_mm, loc.nadir_uv)
        frames = render_location(grid, geometry, camera, 5, np.random.default_rng(seed), neighbors=neighbors)
        result = pipe.analyze(frames, name)
        obs = result.observation
        total += 1
        problems = []
        if not obs.actionable:
            problems.append("neacționabil: {}".format(",".join(obs.reasons)))
        else:
            err = float(np.max(np.linalg.norm(np.array(obs.geometry.corners_px) - np.array(camera.quad_px), axis=1)))
            if err > 4.0:
                problems.append("colț {:.1f}px".format(err))
            for cell in obs.cells:
                stack = grid.stacks.get((cell.coord.row, cell.coord.col)) or []
                if not stack:
                    if cell.occupancy is not Occupancy.EMPTY:
                        problems.append("{} {} (gol)".format(cell.coord, cell.occupancy.value))
                    continue
                top = stack[-1]
                if cell.occupancy is not Occupancy.OCCUPIED or cell.color.value != top.color:
                    problems.append("{} {}/{} (adevăr {})".format(cell.coord, cell.occupancy.value, cell.color.value, top.color))
                    continue
                uv = correct_parallax(cell.uv_top, len(stack) * 30.0, loc.camera_height_mm, loc.nadir_uv)
                true_uv = (cell.coord.col + 0.5 + top.offset_mm[0] / loc.cell_mm, cell.coord.row + 0.5 + top.offset_mm[1] / loc.cell_mm)
                uv_errors.setdefault(name, []).append(float(np.hypot(uv[0] - true_uv[0], uv[1] - true_uv[1])) * loc.cell_mm)
        ok_all += not problems
        if problems:
            print("{:<6} seed {:>2}: {}".format(name, seed, "; ".join(problems)))
print("scene corecte: {}/{}".format(ok_all, total))
for name, errs in uv_errors.items():
    e = np.array(errs)
    print("eroare uv {:<6}: mediană {:.2f} mm, p95 {:.2f} mm, max {:.2f} mm (n={})".format(
        name, float(np.median(e)), float(np.percentile(e, 95)), float(e.max()), len(e)))
