import math

import numpy as np
import pytest

from brain.perception import geometry as geo


def _square(center=(320, 240), side=200, angle_deg=0.0):
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    h = side / 2
    return np.array([(center[0] + u * c - v * s, center[1] + u * s + v * c) for u, v in ((-h, -h), (h, -h), (h, h), (-h, h))])


@pytest.mark.parametrize("angle", [-30, -10, 0, 12, 30])
def test_order_corners_tl_tr_br_bl(angle):
    square = _square(angle_deg=angle)
    shuffled = square[[2, 0, 3, 1]]
    ordered = geo.order_corners(shuffled)
    assert np.allclose(ordered, square)


def test_validate_quad(app_config):
    th = app_config.system.vision.geometry
    shape = (480, 640)
    assert geo.validate_quad(_square(), shape, th) == []
    assert geo.validate_quad(np.array([[0, 0], [100, 100], [200, 200], [300, 300]]), shape, th) == [geo.REASON_NOT_CONVEX]
    assert geo.REASON_TOO_SMALL in geo.validate_quad(_square(side=40), shape, th)
    assert geo.REASON_TOUCHES_BORDER in geo.validate_quad(_square(center=(600, 240)), shape, th)
    trapezoid = np.array([[220, 140], [420, 140], [330, 340], [310, 340]], dtype=float)
    assert geo.REASON_PERSPECTIVE in geo.validate_quad(trapezoid, shape, th)
    skewed = np.array([[200, 140], [400, 140], [560, 340], [360, 340]], dtype=float)  # unghiuri de ~51°/129°
    skewed[2:, 0] += 30
    assert geo.REASON_ANGLE in geo.validate_quad(skewed, shape, th)


def test_fit_homography_round_trip():
    src = _square(angle_deg=8)
    dst = np.array([[0, 0], [300, 0], [300, 300], [0, 300]], dtype=float)
    h = geo.fit_homography(src, dst)
    assert np.allclose(geo.transform_points(h, src), dst, atol=1e-3)  # getPerspectiveTransform lucrează în float32
    assert geo.normalized_condition_number(h, src, dst) < 2.0
    many_src = np.vstack([src, src.mean(axis=0, keepdims=True)])
    many_dst = np.vstack([dst, [[150, 150]]])
    assert np.allclose(geo.transform_points(geo.fit_homography(many_src, many_dst), src), dst, atol=1e-3)
    assert geo.fit_homography(src[:3], dst[:3]) is None
