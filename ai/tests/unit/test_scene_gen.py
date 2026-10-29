import numpy as np
import pytest

from brain.simulation.scene_gen import ALL_KINDS, generate_scene


def test_same_seed_is_reproducible(app_config):
    a = generate_scene(123, app_config.system.grid_geometry("medium"), frames=2)
    b = generate_scene(123, app_config.system.grid_geometry("medium"), frames=2)
    assert a.truth == b.truth
    assert all(np.array_equal(x, y) for x, y in zip(a.frames, b.frames))


def test_different_seeds_differ(app_config):
    a = generate_scene(1, app_config.system.grid_geometry("medium"), frames=1)
    b = generate_scene(2, app_config.system.grid_geometry("medium"), frames=1)
    assert not np.array_equal(a.frames[0], b.frames[0])


def test_frames_differ_only_by_noise(app_config):
    scene = generate_scene(5, app_config.system.grid_geometry("medium"), frames=3)
    diff = np.abs(scene.frames[0].astype(int) - scene.frames[1].astype(int))
    assert 0 < diff.mean() < 8


def test_truth_is_consistent(app_config):
    for seed in range(20):
        truth = generate_scene(seed, app_config.system.grid_geometry("medium"), frames=1).truth
        cells = [(c.row, c.col) for c in truth.cubes]
        assert len(cells) == len(set(cells)) <= 9
        assert all(-45.0 <= c.yaw_deg < 45.0 and c.top_face in ("full", "icon") for c in truth.cubes)
        assert truth.expect_actionable is True and len(truth.grid_corners_px) == 4


def test_kinds_and_validation(app_config):
    grid = app_config.system.grid_geometry("medium")
    for kind in ALL_KINDS:
        aruco = app_config.system.vision.aruco if kind == "missing_marker" else None
        scene = generate_scene(7, grid, kind=kind, frames=1, aruco=aruco)
        assert scene.truth.kind == kind
        assert scene.frames[0].shape == (480, 640, 3)
    with pytest.raises(ValueError):
        generate_scene(1, grid, kind="nimic")
    with pytest.raises(ValueError):
        generate_scene(1, grid, kind="missing_marker")
    with pytest.raises(ValueError):
        generate_scene(1, grid, difficulty="extrem")
