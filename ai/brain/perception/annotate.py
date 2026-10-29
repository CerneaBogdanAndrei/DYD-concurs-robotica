"""Imaginea adnotată pentru verificare vizuală (text ASCII: `cv2.putText` nu suportă diacritice).

Pe lângă culoare și yaw, fiecare cub are latura măsurată (mm) și spațiul liber până la vecini, pe axele lui:
verde = degetul ghearei încape (`GripperGeometry.side_space_mm`), roșu = nu încape. Pe cadrul camerei, conturul
cubului are culoarea stării prinderii: verde = pe centru, galben = cu offset, portocaliu = degetele încap pe o
singură axă, roșu = pe niciuna.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from brain.domain.cube import Color, Occupancy, TopFace
from brain.domain.grasp import GAP_METRIC_KEYS, Footprint, GripperGeometry, clearances_from_metrics, plan_grasp
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
_GRASP_BGR = {"centered": (60, 200, 60), "offset": (0, 210, 230), "risky": (0, 140, 255), "blocked": (40, 40, 230)}
_NEUTRAL = (230, 230, 230)
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


def grasp_status(result: VisionResult, index: int, gripper: Optional[GripperGeometry]) -> Optional[str]:
    """Starea prinderii cubului din celula `index` (vezi `GraspPlan.status`), din spațiile măsurate."""
    obs = result.observation.cells[index]
    if gripper is None or obs.occupancy is not Occupancy.OCCUPIED or result.cube_mm is None:
        return None
    target = Footprint((0.0, 0.0), float(obs.yaw_deg or 0.0), result.cube_mm)
    return plan_grasp(target, clearances_from_metrics(obs.metrics), gripper, 1.0).status


def _to_camera(result: VisionResult, size: float, points: np.ndarray) -> np.ndarray:
    """Puncte din imaginea redresată (orientarea logică, cu margine) → cadrul camerei."""
    m = result.rectified_margin
    assert result.homography is not None
    plain = np.array([_unrotate((float(x), float(y)), size, result.quarter_turns) for x, y in points]) - m
    return transform_points(np.linalg.inv(result.homography), plain)


def _axes(yaw_deg: Optional[float]) -> Tuple[np.ndarray, np.ndarray]:
    psi = math.radians(float(yaw_deg or 0.0))
    return np.array([math.cos(psi), -math.sin(psi)]), np.array([math.sin(psi), math.cos(psi)])


def _draw_gaps(image: np.ndarray, result: VisionResult, index: int, box: np.ndarray, need_mm: Optional[float]) -> None:
    """Spațiile libere: o liniuță de la mijlocul fiecărei fețe spre vecin, cu valoarea în mm."""
    obs = result.observation.cells[index]
    if result.px_per_mm is None:
        return
    center = box.mean(axis=0)
    half = float(np.linalg.norm(np.roll(box, -1, axis=0) - box, axis=1).mean()) / 2.0
    a, b = _axes(obs.yaw_deg)
    for side, key in GAP_METRIC_KEYS:
        gap = obs.metrics.get(key)
        if gap is None:
            continue
        direction = (a if side[0] == "a" else b) * (1.0 if side[1] == "+" else -1.0)
        start = center + direction * half
        end = start + direction * max(gap * result.px_per_mm, 2.0)
        color = _NEUTRAL if need_mm is None else (_OK if gap >= need_mm else _BAD)
        cv2.line(image, (int(start[0]), int(start[1])), (int(end[0]), int(end[1])), color, 2, cv2.LINE_AA)
        normal = np.array([-direction[1], direction[0]])
        label = (start + end) / 2.0 + normal * 9.0 + np.array([-5.0, 4.0])
        _text(image, "{:.0f}".format(gap), (int(label[0]), int(label[1])), 0.33, color)


def overlay_items(result: VisionResult, gripper: Optional[GripperGeometry] = None) -> List[Dict[str, Any]]:
    """Conturul fiecărui cub în cadrul camerei, cu eticheta (cod, laturile, cel mai mic spațiu), desenat și peste
    fluxul live până la analiza următoare. Coordonatele sunt ale cadrului analizat (îndreptat, dacă există
    calibrarea obiectivului)."""
    obs = result.observation
    if not obs.actionable or result.rectified is None or result.homography is None:
        return []
    size = float(result.rectified.shape[0])
    items = []
    for index, cell_result in enumerate(result.cell_results):
        cell = obs.cells[index]
        if cell_result.box is None or cell.occupancy is not Occupancy.OCCUPIED:
            continue
        points = _to_camera(result, size, np.asarray(cell_result.box, dtype=np.float64))
        status = grasp_status(result, index, gripper)
        gaps = [cell.metrics[key] for _, key in GAP_METRIC_KEYS if key in cell.metrics]
        label = ""  # scurt, ca etichetele cuburilor vecine să nu se suprapună: „29x30 7” = laturile, spațiul minim
        if "side_w_mm" in cell.metrics:
            label = "{:.0f}x{:.0f}".format(cell.metrics["side_w_mm"], cell.metrics["side_h_mm"])
        if gaps:
            label += " {:.0f}".format(min(gaps))
        items.append({"poly": [[round(float(x), 1), round(float(y), 1)] for x, y in points],
                      "color": list(_GRASP_BGR.get(status or "", _COLOR_BGR[cell.color])), "label": label})
    return items


def draw_overlay(image: np.ndarray, items: Sequence[Dict[str, Any]]) -> None:
    """Desenează `overlay_items` pe un cadru (pe loc)."""
    for item in items:
        pts = np.rint(np.array(item["poly"], dtype=np.float64)).astype(np.int32).reshape(-1, 1, 2)
        color = tuple(int(c) for c in item["color"])
        cv2.polylines(image, [pts], True, color, 2, cv2.LINE_AA)
        flat = pts.reshape(-1, 2)
        if item["label"]:
            _text(image, str(item["label"]), (int(flat[:, 0].mean()) - 20, int(flat[:, 1].min()) - 5), 0.35, color)


def render_annotation(result: VisionResult, gripper: Optional[GripperGeometry] = None) -> np.ndarray:
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
            metrics = cell_result.observation.metrics
            if "side_w_mm" in metrics:
                _text(right, "{:.0f}x{:.0f}mm".format(metrics["side_w_mm"], metrics["side_h_mm"]), (x0 + 4, y0 + 75), 0.35)
            if cell_result.box is not None:
                _draw_gaps(right, result, index, np.asarray(cell_result.box, dtype=np.float64),
                           None if gripper is None else gripper.side_space_mm)
            center = _unrotate((x0 + 0.5 * cell, y0 + 0.5 * cell), float(right.shape[0]), result.quarter_turns)
            px = transform_points(inverse, np.array([[center[0] - m, center[1] - m]]))[0]
            _text(left, code, (int(px[0]) - 10, int(px[1]) + 5), 0.5, color)
        draw_overlay(left, overlay_items(result, gripper))

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
