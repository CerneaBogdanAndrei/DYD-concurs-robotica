import dataclasses

import numpy as np
import pytest

from brain.domain.cube import Occupancy, yaw_difference_deg
from brain.domain.errors import PerceptionInputError
from brain.perception.pipeline import VisionPipeline
from brain.simulation.scene_gen import NEGATIVE_KINDS, generate_scene

CORNER_TOLERANCE_PX = 4.0


@pytest.fixture(scope="module")
def pipeline(app_config):
    return VisionPipeline(app_config.system, app_config.colors)


def _corner_error(observation, truth):
    return float(np.max(np.linalg.norm(np.array(observation.geometry.corners_px) - np.array(truth.grid_corners_px), axis=1)))


@pytest.mark.parametrize("seed", range(900, 912))
def test_positive_scenes(pipeline, app_config, seed):
    scene = generate_scene(seed, app_config.system.grid_geometry("medium"), frames=5)
    observation = pipeline.analyze(scene.frames, "medium").observation
    assert observation.actionable, observation.reasons
    assert _corner_error(observation, scene.truth) < CORNER_TOLERANCE_PX
    for cell in observation.cells:
        cube = scene.truth.cube_at(cell.coord.row, cell.coord.col)
        if cube is None:
            assert cell.occupancy is Occupancy.EMPTY, (cell.coord, cell.reasons)
        else:
            assert cell.occupancy is Occupancy.OCCUPIED, (cell.coord, cell.reasons)
            assert cell.color.value == cube.color
            if cell.yaw_deg is not None:
                assert yaw_difference_deg(cell.yaw_deg, cube.yaw_deg) < 5.0


@pytest.mark.parametrize("kind", NEGATIVE_KINDS)
def test_negative_scenes_are_rejected(pipeline, app_config, kind):
    for seed in (11, 12):
        scene = generate_scene(seed, app_config.system.grid_geometry("medium"), kind=kind, frames=3)
        observation = pipeline.analyze(scene.frames, "medium").observation
        assert not observation.actionable
        assert observation.reasons
        assert observation.cells == ()


def test_aruco_detection(app_config):
    aruco = app_config.system.vision.aruco
    pipeline = VisionPipeline(app_config.system, app_config.colors, method="aruco")
    scene = generate_scene(31, app_config.system.grid_geometry("medium"), frames=3, aruco=aruco)
    observation = pipeline.analyze(scene.frames, "medium").observation
    assert observation.actionable, observation.reasons
    assert _corner_error(observation, scene.truth) < CORNER_TOLERANCE_PX
    missing = generate_scene(32, app_config.system.grid_geometry("medium"), kind="missing_marker", frames=3, aruco=aruco)
    rejected = pipeline.analyze(missing.frames, "medium").observation
    assert not rejected.actionable
    assert any(reason.startswith("markers_missing") for reason in rejected.reasons)


@pytest.mark.parametrize("seed, difficulty, location", [(4200187, "hard", "medium"), (4200228, "nominal", "small")])
def test_best_fit_wins_among_candidates_of_same_grid(pipeline, app_config, seed, difficulty, location):
    # Regresii (selftest seed 42): mai mulți candidați pentru aceeași grilă. Era ales unul cu eroare de 5-6 px
    # pentru că era cu câțiva px mai aproape de centru (hard) sau avea un reziduu mai mic doar prin zgomot (small).
    scene = generate_scene(seed, app_config.system.grid_geometry(location), difficulty=difficulty, frames=5)
    observation = pipeline.analyze(scene.frames, location).observation
    assert observation.actionable, observation.reasons
    assert _corner_error(observation, scene.truth) < CORNER_TOLERANCE_PX


def test_occluded_corner_is_safe(pipeline, app_config):
    for seed in range(40, 46):
        scene = generate_scene(seed, app_config.system.grid_geometry("medium"), kind="occluded_corner", frames=3)
        observation = pipeline.analyze(scene.frames, "medium").observation
        if observation.actionable:
            assert _corner_error(observation, scene.truth) < CORNER_TOLERANCE_PX


def test_orientation_quarter_turn_rotates_indexing(app_config):
    scene = generate_scene(905, app_config.system.grid_geometry("medium"), frames=3)
    base = VisionPipeline(app_config.system, app_config.colors).analyze(scene.frames, "medium").observation
    locations = dict(app_config.system.locations)
    locations["medium"] = dataclasses.replace(locations["medium"], orientation_quarter_turns=1)
    rotated_system = dataclasses.replace(app_config.system, locations=locations)
    rotated = VisionPipeline(rotated_system, app_config.colors).analyze(scene.frames, "medium").observation
    assert rotated.actionable
    for cell in base.cells:
        # np.rot90 (antiorar): celula (r, c) ajunge la (n-1-c, r)
        moved = rotated.cells[(2 - cell.coord.col) * 3 + cell.coord.row]
        assert (moved.occupancy, moved.color) == (cell.occupancy, cell.color)


def test_invalid_inputs(pipeline):
    with pytest.raises(PerceptionInputError):
        pipeline.analyze([], "medium")
    with pytest.raises(PerceptionInputError):
        pipeline.analyze([np.zeros((480, 640, 3), np.uint8)], "necunoscut")
