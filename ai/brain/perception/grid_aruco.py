"""Detecția opțională a caroiajului prin markeri ArUco (doar dacă regulamentul permite markeri).

Compatibil cu API-ul vechi (OpenCV < 4.7: `detectMarkers`) și cu cel nou (`ArucoDetector`).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from brain.configuration.models import ArucoConfig
from brain.domain.errors import PerceptionInputError

REASON_MARKERS_MISSING = "markers_missing"
REASON_MARKERS_DUPLICATE = "markers_duplicate"


@dataclass(frozen=True)
class MarkerDetection:
    """`centers`: centrele markerilor în ordinea din config (TL, TR, BR, BL), în px."""

    ok: bool
    centers: Optional[np.ndarray]
    found_ids: Tuple[int, ...]
    reasons: Tuple[str, ...]


def get_dictionary(name: str) -> Any:
    aruco = getattr(cv2, "aruco", None)
    if aruco is None:
        raise PerceptionInputError("OpenCV nu are modulul aruco (instalează opencv-contrib-python)")
    constant = getattr(aruco, name, None)
    if constant is None:
        raise PerceptionInputError("dicționar ArUco necunoscut: {}".format(name))
    return aruco.getPredefinedDictionary(constant)


def detect_markers(gray: np.ndarray, dictionary: Any) -> Tuple[List[np.ndarray], Optional[np.ndarray]]:
    aruco = cv2.aruco
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)
    else:  # OpenCV < 4.7
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)
    return list(corners), ids


def render_marker(dictionary_name: str, marker_id: int, size_px: int) -> np.ndarray:
    dictionary = get_dictionary(dictionary_name)
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)
    return cv2.aruco.drawMarker(dictionary, marker_id, size_px)  # OpenCV < 4.7


def detect_grid_markers(image: np.ndarray, config: ArucoConfig) -> MarkerDetection:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids = detect_markers(gray, get_dictionary(config.dictionary))
    found: Dict[int, List[np.ndarray]] = {}
    if ids is not None:
        for marker_corners, marker_id in zip(corners, ids.ravel().tolist()):
            found.setdefault(int(marker_id), []).append(np.asarray(marker_corners).reshape(4, 2).mean(axis=0))
    found_ids = tuple(sorted(found))
    missing = [m for m in config.marker_ids if m not in found]
    duplicate = [m for m in config.marker_ids if len(found.get(m, [])) > 1]
    reasons: List[str] = []
    if missing:
        reasons.append("{}:{}".format(REASON_MARKERS_MISSING, ",".join(str(m) for m in missing)))
    if duplicate:
        reasons.append("{}:{}".format(REASON_MARKERS_DUPLICATE, ",".join(str(m) for m in duplicate)))
    if reasons:
        return MarkerDetection(False, None, found_ids, tuple(reasons))
    centers = np.array([found[m][0] for m in config.marker_ids], dtype=np.float64)
    return MarkerDetection(True, centers, found_ids, ())
