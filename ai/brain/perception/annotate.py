"""Imaginea adnotată pentru verificare vizuală (text ASCII: `cv2.putText` nu suportă diacritice)."""

from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np

from brain.domain.cube import Color, Occupancy, TopFace
from brain.perception.geometry import transform_points
from brain.perception.pipeline import VisionResult

_COLOR_BGR = {
    Color.RED: (40, 40, 220),
    Color.YELLOW: (0, 200, 230),
    Color.GREEN: (40, 170, 40),
    Color.BLUE: (200, 90, 30),
    Color.UNKNOWN: (128, 128, 128),
}
_COLOR_CODE = {Color.RED: "R", Color.YELLOW: "Y", Color.GREEN: "G", Color.BLUE: "B", Color.UNKNOWN: "?"}
_FACE_CODE = {TopFace.FULL: "F", TopFace.ICON: "I", TopFace.UNKNOWN: ""}
_OK = (60, 180, 60)
_BAD = (40, 40, 220)
_FONT = cv2.FONT_HERSHEY_SIMPLEX


def _text(image: np.ndarray, text: str, org: Tuple[int, int], scale: float = 0.45, color=(255, 255, 255)) -> None:
    cv2.putText(image, text, org, _FONT, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, org, _FONT, scale, color, 1, cv2.LINE_AA)


def _unrotate(point: Tuple[float, float], size: float, turns: int) -> Tuple[float, float]:
    """Inversează `np.rot90(k=turns)` pentru un punct din imaginea logică (pătrată, latură `size`)."""
    x, y = point
    for _ in range(turns % 4):
        x, y = size - y, x
    return x, y


def _cell_label(result: VisionResult, index: int) -> Tuple[str, str, Tuple[int, int, int]]:
    obs = result.observation.cells[index]
    if obs.occupancy is Occupancy.EMPTY:
        return "--", "{:.2f}".format(obs.occupancy_confidence), (200, 200, 200)
    if obs.occupancy is Occupancy.UNKNOWN:
        return "??", "unk", _BAD
    code = _COLOR_CODE[obs.color] + _FACE_CODE[obs.top_face]
    yaw = "yaw?" if obs.yaw_deg is None else "{:+.0f}".format(obs.yaw_deg)
    return code, "{:.2f} {}".format(min(obs.occupancy_confidence, obs.color_confidence), yaw), _COLOR_BGR[obs.color]


def render_annotation(result: VisionResult) -> np.ndarray:
    obs = result.observation
    if result.aggregated is not None:
        left = result.aggregated.copy()
    else:
        left = np.full((480, 640, 3), 64, dtype=np.uint8)
    status_color = _OK if obs.actionable else _BAD
    if obs.geometry.corners_px:
        pts = np.array(obs.geometry.corners_px, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(left, [pts], True, status_color, 2, cv2.LINE_AA)

    right: Optional[np.ndarray] = None
    if obs.actionable and result.rectified is not None and result.homography is not None:
        right = result.rectified.copy()
        m = result.rectified_margin
        cell = (right.shape[1] - 2 * m) // obs.cols
        inverse = np.linalg.inv(result.homography)
        for k in range(obs.cols + 1):
            cv2.line(right, (m + k * cell, m), (m + k * cell, m + obs.rows * cell), (90, 90, 90), 1)
        for k in range(obs.rows + 1):
            cv2.line(right, (m, m + k * cell), (m + obs.cols * cell, m + k * cell), (90, 90, 90), 1)
        for index, cell_result in enumerate(result.cell_results):
            coord = cell_result.observation.coord
            code, detail, color = _cell_label(result, index)
            x0, y0 = m + coord.col * cell, m + coord.row * cell
            if cell_result.box is not None:
                cv2.polylines(right, [np.rint(cell_result.box).astype(np.int32).reshape(-1, 1, 2)], True, color, 2)
            _text(right, "{} {}".format(coord.row, coord.col), (x0 + 4, y0 + 14), 0.4, (180, 180, 180))
            _text(right, code, (x0 + 30, y0 + 55), 0.6, color)
            _text(right, detail, (x0 + 4, y0 + cell - 6), 0.38)
            center = _unrotate((x0 + 0.5 * cell, y0 + 0.5 * cell), float(right.shape[0]), result.quarter_turns)
            px = transform_points(inverse, np.array([[center[0] - m, center[1] - m]]))[0]
            _text(left, code, (int(px[0]) - 10, int(px[1]) + 5), 0.5, color)

    header = "{}  {}".format(obs.location, "OK" if obs.actionable else "NOT ACTIONABLE")
    if obs.reasons:
        header += "  " + ",".join(obs.reasons)
    bar = np.full((28, left.shape[1], 3), 30, dtype=np.uint8)
    _text(bar, header[:110], (6, 19), 0.5, status_color)
    left = np.vstack([bar, left])
    if right is None:
        return left
    panel = np.full((left.shape[0], right.shape[1] + 10, 3), 30, dtype=np.uint8)
    h = min(right.shape[0], panel.shape[0] - 28)
    panel[28:28 + h, 10:10 + right.shape[1]] = right[:h]
    return np.hstack([left, panel])
