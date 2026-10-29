"""Experiment: o a doua trecere de rafinare (pornind de la colțurile rafinate) reduce eroarea candidaților?"""
import sys

import cv2
import numpy as np

from brain.configuration.loader import default_config_dir, load_app_config
from brain.perception import grid_lines as gl
from brain.perception.geometry import dark_mask, order_corners
from brain.perception.photometry import aggregate_frames
from brain.simulation.scene_gen import generate_scene

cfg = load_app_config(default_config_dir())
system = cfg.system
for spec in sys.argv[1:]:
    seed, difficulty, location = spec.split(":")
    scene = generate_scene(int(seed), system.grid_geometry(location), kind="positive", difficulty=difficulty, frames=5,
                           rows=3, cols=3, aruco=None)
    image = aggregate_frames(scene.frames)
    truth = np.array(scene.truth.grid_corners_px)
    h, w = image.shape[:2]
    lines = system.vision.lines
    mask = cv2.morphologyEx(dark_mask(image, min(h, w) * lines.adaptive_block_fraction, lines.adaptive_c, lines.dark_chroma_max),
                            cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    quad, hull = gl._coarse_quad(mask, lines, True)
    center = hull.reshape(-1, 2).mean(axis=0)
    region = np.zeros_like(mask)
    cv2.fillConvexPoly(region, np.rint((hull.reshape(-1, 2) - center) * 1.2 + center).astype(np.int32), 255)
    m = cv2.bitwise_and(mask, region)
    for label, start in (("hull", quad), ("minAreaRect", order_corners(cv2.boxPoints(cv2.minAreaRect(hull)).astype(np.float64)))):
        errs = []
        current = start
        for k in range(4):
            refined = gl._refine_by_profiles(m, current, 3, 3)
            if refined is None:
                errs.append("eșec")
                break
            pts, _, residual = refined
            current = gl.corners_from_points(pts, 3, 3)
            errs.append("{:.2f}px/{:.4f}".format(float(np.max(np.linalg.norm(current - truth, axis=1))), residual))
        print("{} {:<12} treceri: {}".format(spec, label, "  ".join(errs)))
