"""Calibrarea obiectivului (tabla de șah) și corecția distorsiunii."""

import shutil

import cv2
import numpy as np
import pytest

from brain.cli.main import EXIT_OK, main
from brain.configuration import load_app_config
from brain.configuration.models import LensConfig
from brain.domain.errors import ConfigError
from brain.perception.image_io import write_image
from brain.perception.lens import Undistorter, calibrate, chessboard_object_points, find_chessboard

K = np.array([[420.0, 0.0, 322.0], [0.0, 418.0, 236.0], [0.0, 0.0, 1.0]])
DIST = np.array([-0.32, 0.11, 0.001, -0.0005, -0.015])  # butoi, ca la un obiectiv cu unghi larg
SIZE = (640, 480)


def _views(count, seed=4):
    rng = np.random.default_rng(seed)
    board = chessboard_object_points((9, 6), 25.0)
    objects, images = [], []
    for _ in range(count):
        rvec = rng.uniform(-0.45, 0.45, size=3)
        tvec = np.array([rng.uniform(-110, -10), rng.uniform(-90, -10), rng.uniform(330, 470)])
        points, _ = cv2.projectPoints(board, rvec, tvec, K, DIST)
        objects.append(board)
        images.append(points.reshape(-1, 2).astype(np.float32))
    return objects, images


def test_calibration_recovers_intrinsics_from_projections():
    objects, images = _views(14)
    matrix, dist, rms = calibrate(objects, images, SIZE)
    assert rms < 0.05
    assert np.allclose(matrix[0, 0], K[0, 0], rtol=0.02) and np.allclose(matrix[1, 1], K[1, 1], rtol=0.02)
    assert abs(matrix[0, 2] - K[0, 2]) < 5 and abs(matrix[1, 2] - K[1, 2]) < 5
    assert abs(dist[0] - DIST[0]) < 0.03


def test_undistorter_straightens_a_distorted_point():
    lens = LensConfig(SIZE, tuple(tuple(r) for r in K), tuple(DIST), 0.1, True)
    undistort = Undistorter(lens)
    ideal = np.array([[[-0.55, -0.40]]])  # direcție normalizată spre colțul imaginii (unde distorsiunea e mare)
    distorted, _ = cv2.projectPoints(np.array([[ideal[0, 0, 0], ideal[0, 0, 1], 1.0]]), np.zeros(3), np.zeros(3), K, DIST)
    image = np.full((SIZE[1], SIZE[0], 3), 255, np.uint8)
    x, y = distorted.reshape(2)
    cv2.circle(image, (int(round(x)), int(round(y))), 3, (0, 0, 0), -1)
    corrected = undistort(image)
    ys, xs = np.nonzero(corrected[..., 0] < 128)
    new_matrix, _ = cv2.getOptimalNewCameraMatrix(K, DIST, SIZE, 0.0, SIZE)
    expected = (new_matrix[0, 0] * ideal[0, 0, 0] + new_matrix[0, 2], new_matrix[1, 1] * ideal[0, 0, 1] + new_matrix[1, 2])
    assert abs(xs.mean() - expected[0]) < 1.5 and abs(ys.mean() - expected[1]) < 1.5
    assert abs(x - expected[0]) > 10  # distorsiunea chiar era mare acolo


def _render_board(homography):
    squares = 40
    board = np.full((7 * squares + 80, 10 * squares + 80), 255, np.uint8)
    for r in range(7):
        for c in range(10):
            if (r + c) % 2 == 0:
                board[40 + r * squares:40 + (r + 1) * squares, 40 + c * squares:40 + (c + 1) * squares] = 0
    image = cv2.warpPerspective(board, homography, SIZE, borderValue=255)
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def _pose_homography(rvec, tvec):
    """Tabla (px ai imaginii tablei) → imagine, pentru o poză 3D reală a tablei și camera K fără distorsiune."""
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, np.float64))
    plane = K @ np.column_stack([rotation[:, 0], rotation[:, 1], tvec])
    scale = 25.0 / 40.0  # 40 px ai imaginii tablei = un pătrat de 25 mm; colțul (0,0) e la (80, 80) px
    board_to_mm = np.array([[scale, 0, -80 * scale], [0, scale, -80 * scale], [0, 0, 1]])
    return plane @ board_to_mm


def test_calib_camera_cli_writes_lens_and_pipeline_uses_it(tmp_path, config_dir):
    rng = np.random.default_rng(9)
    frames = tmp_path / "chess"
    for k in range(10):
        rvec = rng.uniform(-0.5, 0.5, size=3)
        tvec = np.array([rng.uniform(-120, -70), rng.uniform(-90, -40), rng.uniform(380, 480)])
        image = _render_board(_pose_homography(rvec, tvec))
        assert find_chessboard(image, (9, 6)) is not None
        write_image(frames / "board_{}.png".format(k), image)
    copy = tmp_path / "config"
    shutil.copytree(str(config_dir), str(copy))
    code = main(["--config-dir", str(copy), "calib", "camera", "--frames", str(frames / "*.png"),
                 "--pattern", "9x6", "--square-mm", "25", "--write"])
    assert code == EXIT_OK
    lens = load_app_config(copy).system.lens
    assert lens is not None and lens.image_size == SIZE and lens.calibrated
    assert lens.rms_px < 0.5 and abs(lens.camera_matrix[0][0] - K[0, 0]) < 0.05 * K[0, 0]
    assert abs(lens.dist_coeffs[0]) < 0.1  # imagini fără distorsiune
    assert load_app_config(config_dir).system.lens is None  # fără lens.yaml: fără corecție


def test_calib_camera_refuses_a_bad_calibration(tmp_path, config_dir):
    rng = np.random.default_rng(9)
    frames = tmp_path / "chess"
    for k in range(8):  # vederi inconsistente (patrulatere aleatoare, nu poze 3D ale aceleiași camere)
        src = np.array([[0, 0], [480, 0], [480, 360], [0, 360]], np.float32)
        dst = np.array([[90, 70], [550, 60], [560, 420], [80, 410]], np.float32) + rng.uniform(-45, 45, (4, 2)).astype(np.float32)
        write_image(frames / "board_{}.png".format(k), _render_board(cv2.getPerspectiveTransform(src, dst)))
    copy = tmp_path / "config"
    shutil.copytree(str(config_dir), str(copy))
    code = main(["--config-dir", str(copy), "calib", "camera", "--frames", str(frames / "*.png"),
                 "--square-mm", "25", "--write"])
    assert code != EXIT_OK and not (copy / "lens.yaml").exists()


def test_invalid_lens_file_names_the_key(tmp_path, config_dir):
    copy = tmp_path / "config"
    shutil.copytree(str(config_dir), str(copy))
    (copy / "lens.yaml").write_text("schema_version: 1\ncalibrated: true\nimage_size: [640, 480]\n"
                                    "camera_matrix: [[0, 0, 320], [0, 400, 240], [0, 0, 1]]\n"
                                    "dist_coeffs: [0, 0, 0, 0, 0]\nrms_px: 0.3\n", encoding="utf-8")
    with pytest.raises(ConfigError) as exc_info:
        load_app_config(copy)
    assert exc_info.value.key == "camera_matrix"
