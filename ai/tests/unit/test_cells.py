import math

import cv2
import numpy as np
import pytest

from brain.domain.cube import Color, Occupancy, TopFace, normalize_yaw_deg, yaw_difference_deg
from brain.domain.grid import GridCoord
from brain.perception.cells import (
    REASON_DARK_OBJECT,
    REASON_FAINT_COLOR,
    REASON_SEPARATED,
    REASON_SPLIT,
    CellAnalyzer,
    dominant_orientation,
)
from brain.perception.color import HueClassifier

CELL = 100
CUBE = 75.0


def _square(center, side, yaw_deg):
    """Pătrat rotit antiorar pe ecran (convenția de yaw a sistemului)."""
    a = math.radians(yaw_deg)
    c, s = math.cos(a), math.sin(a)
    h = side / 2
    return np.array([(center[0] + u * c + v * s, center[1] - u * s + v * c) for u, v in ((-h, -h), (h, -h), (h, h), (-h, h))])


def _bgr(h, s, v):
    return tuple(int(x) for x in cv2.cvtColor(np.array([[[h, s, v]]], np.uint8), cv2.COLOR_HSV2BGR)[0, 0])


def _fill(image, pts, color):
    cv2.fillPoly(image, [np.rint(pts * 16).astype(np.int32).reshape(-1, 1, 2)], color, cv2.LINE_AA, 4)


def _board():
    image = np.full((300, 300, 3), (205, 207, 209), dtype=np.uint8)
    for k in range(4):
        p = min(k * CELL, 299)
        cv2.line(image, (p, 0), (p, 299), (32, 32, 32), 6)
        cv2.line(image, (0, p), (299, p), (32, 32, 32), 6)
    return image


def _cube(image, cell, yaw, color, face="full", offset=(0.0, 0.0)):
    center = ((cell[1] + 0.5) * CELL + offset[0], (cell[0] + 0.5) * CELL + offset[1])
    _fill(image, _square(center, CUBE, yaw), color)
    if face == "icon":
        _fill(image, _square(center, CUBE - 12, yaw), (236, 236, 236))
        cv2.circle(image, (int(center[0]), int(center[1])), 12, (45, 45, 48), -1)


@pytest.fixture(scope="module")
def analyzer(app_config):
    system = app_config.system
    return CellAnalyzer(system.vision.cells, HueClassifier(app_config.colors), CELL, system.cube_px("medium"))


def _by_coord(results):
    return {r.observation.coord.as_tuple(): r.observation for r in results}


def test_empty_board(analyzer):
    cells = _by_coord(analyzer.analyze(_board(), 3, 3))
    assert all(c.occupancy is Occupancy.EMPTY and c.occupancy_confidence >= 0.9 for c in cells.values())


def test_full_and_icon_faces(analyzer):
    board = _board()
    _cube(board, (1, 1), 20.0, _bgr(0, 200, 190))
    _cube(board, (0, 2), -30.0, _bgr(110, 200, 170), face="icon")
    cells = _by_coord(analyzer.analyze(board, 3, 3))
    red, blue = cells[(1, 1)], cells[(0, 2)]
    assert red.occupancy is Occupancy.OCCUPIED and red.color is Color.RED and red.top_face is TopFace.FULL
    assert yaw_difference_deg(red.yaw_deg, 20.0) < 1.5
    assert blue.color is Color.BLUE and blue.top_face is TopFace.ICON
    assert yaw_difference_deg(blue.yaw_deg, -30.0) < 1.5
    assert sum(c.occupancy is Occupancy.EMPTY for c in cells.values()) == 7


def test_desaturated_cube_is_unknown_not_empty(analyzer):
    board = _board()
    _cube(board, (2, 0), 10.0, _bgr(60, 45, 190))
    cell = _by_coord(analyzer.analyze(board, 3, 3))[(2, 0)]
    assert cell.occupancy is Occupancy.UNKNOWN
    assert REASON_FAINT_COLOR in cell.reasons


def test_dark_object_is_unknown(analyzer):
    board = _board()
    cv2.circle(board, (250, 250), 14, (40, 40, 40), -1)
    cell = _by_coord(analyzer.analyze(board, 3, 3))[(2, 2)]
    assert cell.occupancy is Occupancy.UNKNOWN
    assert REASON_DARK_OBJECT in cell.reasons


def test_touching_cubes_are_separated(analyzer):
    board = _board()
    green = _bgr(62, 180, 150)
    _cube(board, (1, 0), 35.0, green, offset=(8.0, 0.0))
    _cube(board, (1, 1), -20.0, green, offset=(-8.0, 0.0))
    cells = _by_coord(analyzer.analyze(board, 3, 3))
    for coord, yaw in (((1, 0), 35.0), ((1, 1), -20.0)):
        cell = cells[coord]
        assert cell.occupancy is Occupancy.OCCUPIED and cell.color is Color.GREEN
        assert REASON_SEPARATED in cell.reasons
        assert cell.yaw_deg is not None and yaw_difference_deg(cell.yaw_deg, yaw) < 1.5


def test_fused_cubes_without_neck_fall_back_to_cell_cut(analyzer):
    """Două cuburi aliniate și suprapuse formează un dreptunghi: forma nu le poate separa."""
    board = _board()
    green = _bgr(62, 180, 150)
    _cube(board, (1, 0), 0.0, green, offset=(15.0, 0.0))
    _cube(board, (1, 1), 0.0, green, offset=(-15.0, 0.0))
    cells = _by_coord(analyzer.analyze(board, 3, 3))
    for coord in ((1, 0), (1, 1)):
        cell = cells[coord]
        assert cell.occupancy is Occupancy.OCCUPIED and cell.color is Color.GREEN
        assert REASON_SPLIT in cell.reasons
        assert cell.occupancy_confidence <= 0.7


@pytest.mark.parametrize("yaw", [-40.0, -12.0, 0.0, 7.5, 33.0])
def test_dominant_orientation(yaw):
    mask = np.zeros((200, 200), np.uint8)
    _fill(mask, _square((100, 100), 90, yaw), 255)
    theta, concentration = dominant_orientation(mask)
    assert yaw_difference_deg(normalize_yaw_deg(-theta), yaw) < 1.0
    assert concentration > 0.9
