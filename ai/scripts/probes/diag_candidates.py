"""Diagnostic: candidații detect_grid_lines pentru o scenă din selftest (eroare de colț per candidat)."""
import sys

import numpy as np

from brain.configuration.loader import default_config_dir, load_app_config
from brain.perception import grid_lines as gl
from brain.perception.photometry import aggregate_frames
from brain.perception.pipeline import VisionPipeline
from brain.simulation.scene_gen import generate_scene

seed = int(sys.argv[1])
difficulty = sys.argv[2]
location = sys.argv[3] if len(sys.argv) > 3 else "medium"
cfg = load_app_config(default_config_dir())
system = cfg.system
loc = system.location(location)
scene = generate_scene(seed, system.grid_geometry(location), kind="positive", difficulty=difficulty, frames=5,
                       rows=3, cols=3, aruco=None)
pipe = VisionPipeline(system, cfg.colors, method="lines")
image = aggregate_frames(scene.frames)
truth = np.array(scene.truth.grid_corners_px)

# Reproduce pornirile din detect_grid_lines, cu etichete.
import cv2
from brain.perception.geometry import dark_mask, order_corners

h, w = image.shape[:2]
mask = dark_mask(image, min(h, w) * system.vision.lines.adaptive_block_fraction, system.vision.lines.adaptive_c,
                 system.vision.lines.dark_chroma_max)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
for iterative in (True, False):
    coarse = gl._coarse_quad(mask, system.vision.lines, iterative)
    if coarse is None:
        print("iterative={} : fără quad".format(iterative))
        continue
    quad, hull = coarse
    for label, start in (("hull", quad), ("minAreaRect", order_corners(cv2.boxPoints(cv2.minAreaRect(hull)).astype(np.float64)))):
        center = hull.reshape(-1, 2).mean(axis=0)
        region = np.zeros_like(mask)
        cv2.fillConvexPoly(region, np.rint((hull.reshape(-1, 2) - center) * 1.2 + center).astype(np.int32), 255)
        refined = gl._refine_by_profiles(cv2.bitwise_and(mask, region), start, 3, 3)
        start_err = float(np.max(np.linalg.norm(start - truth, axis=1)))
        if refined is None:
            print("iter={:<5} start={:<11} start_err={:6.1f}px -> rafinare eșuată".format(str(iterative), label, start_err))
            continue
        pts, grid_pts, residual = refined
        geom = pipe._evaluate(image, loc, pts, grid_pts * system.vision.rectified_cell_px, {"line_residual_cells": residual})
        err = float(np.max(np.linalg.norm(geom.grid_corners - truth, axis=1))) if geom.grid_corners is not None else float("nan")
        print("iter={:<5} start={:<11} start_err={:6.1f}px -> ok={} err={:5.2f}px rezid={:.4f} struct={} min_sup={} {}".format(
            str(iterative), label, start_err, geom.ok, err, residual, geom.metrics.get("structure_score"),
            geom.metrics.get("min_line_support"), geom.reasons))
res = pipe.analyze(scene.frames, location)
sel = np.array(res.observation.geometry.corners_px)
print("selectat: actionable={} err={:.2f}px metrics={}".format(
    res.observation.actionable, float(np.max(np.linalg.norm(sel - truth, axis=1))) if len(sel) else float("nan"),
    res.observation.geometry.metrics))
