"""Distorsiunea obiectivului: calibrarea cu o tablă de șah și corecția cadrelor.

Camera de pe braț are un obiectiv cu unghi larg (110°): liniile drepte ale caroiajului apar curbe spre margini,
iar o omografie (care presupune o cameră ideală) le potrivește prost. Cu parametrii intrinseci (matricea camerei
și coeficienții de distorsiune) fiecare cadru se îndreaptă înainte de detecția grilei.

Calibrarea: 10-20 de poze cu o tablă de șah tipărită, plată, din unghiuri și poziții diferite, care acoperă și
marginile imaginii (`brain camera capture`, apoi `brain calib camera`).
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from brain.configuration.models import LensConfig
from brain.domain.errors import PerceptionInputError

SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
MIN_CALIBRATION_FRAMES = 6


def chessboard_object_points(pattern: Tuple[int, int], square_mm: float) -> np.ndarray:
    """Colțurile interioare ale tablei (coloane x rânduri), în mm, pe planul z = 0."""
    cols, rows = pattern
    grid = np.zeros((rows * cols, 3), np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(square_mm)
    return grid


def find_chessboard(image: np.ndarray, pattern: Tuple[int, int]) -> Optional[np.ndarray]:
    """Colțurile interioare ale tablei de șah (subpixel), sau None dacă tabla nu e găsită complet."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not found or corners is None:
        return None
    return cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), SUBPIX_CRITERIA)


def calibrate(
    object_points: Sequence[np.ndarray], image_points: Sequence[np.ndarray], image_size: Tuple[int, int]
) -> Tuple[np.ndarray, np.ndarray, float]:
    """Matricea camerei, coeficienții de distorsiune (k1, k2, p1, p2, k3) și eroarea RMS de reproiecție (px)."""
    if len(image_points) < MIN_CALIBRATION_FRAMES:
        raise PerceptionInputError("prea puține poze cu tabla găsită: {} (minim {})".format(
            len(image_points), MIN_CALIBRATION_FRAMES))
    objects = [np.asarray(p, np.float32).reshape(-1, 1, 3) for p in object_points]
    images = [np.asarray(p, np.float32).reshape(-1, 1, 2) for p in image_points]
    rms, matrix, dist, _, _ = cv2.calibrateCamera(objects, images, tuple(int(v) for v in image_size), None, None)
    return matrix, dist.reshape(-1)[:5], float(rms)


class Undistorter:
    """Îndreaptă cadrele cu parametrii obiectivului. Hărțile de remapare se calculează o dată per rezoluție."""

    def __init__(self, lens: LensConfig, alpha: float = 0.0) -> None:
        self.lens = lens
        self.alpha = float(alpha)  # 0 = doar pixeli valizi (fără colțuri negre), 1 = tot câmpul
        self._maps: Dict[Tuple[int, int], Tuple[np.ndarray, np.ndarray]] = {}
        self._params: Dict[Tuple[int, int], Tuple[np.ndarray, np.ndarray, np.ndarray]] = {}

    def _params_for(self, width: int, height: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(matricea camerei, distorsiunea, matricea cadrului îndreptat) la rezoluția dată."""
        key = (width, height)
        if key not in self._params:
            matrix = np.array(self.lens.camera_matrix, np.float64)
            calib_w, calib_h = self.lens.image_size
            if (width, height) != (calib_w, calib_h):  # aceeași cameră la altă rezoluție: scalăm matricea
                matrix = matrix.copy()
                matrix[0] *= width / float(calib_w)
                matrix[1] *= height / float(calib_h)
            dist = np.array(self.lens.dist_coeffs, np.float64)
            new_matrix, _ = cv2.getOptimalNewCameraMatrix(matrix, dist, (width, height), self.alpha, (width, height))
            self._params[key] = (matrix, dist, new_matrix)
        return self._params[key]

    def _maps_for(self, width: int, height: int) -> Tuple[np.ndarray, np.ndarray]:
        key = (width, height)
        if key not in self._maps:
            matrix, dist, new_matrix = self._params_for(width, height)
            self._maps[key] = cv2.initUndistortRectifyMap(matrix, dist, None, new_matrix, (width, height), cv2.CV_16SC2)
        return self._maps[key]

    def distort_points(self, points: np.ndarray, width: int, height: int) -> np.ndarray:
        """Puncte din cadrul îndreptat → cadrul brut al camerei (desenele peste fluxul live, care nu e îndreptat)."""
        matrix, dist, new_matrix = self._params_for(width, height)
        pts = np.asarray(points, np.float64).reshape(-1, 2)
        normalized = (np.linalg.inv(new_matrix) @ np.column_stack([pts, np.ones(len(pts))]).T).T
        rays = np.column_stack([normalized[:, 0] / normalized[:, 2], normalized[:, 1] / normalized[:, 2], np.ones(len(pts))])
        projected, _ = cv2.projectPoints(rays.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), matrix, dist)
        return projected.reshape(-1, 2)

    def __call__(self, frame: np.ndarray) -> np.ndarray:
        height, width = frame.shape[:2]
        map1, map2 = self._maps_for(width, height)
        return cv2.remap(frame, map1, map2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)

    def undistort_all(self, frames: Sequence[np.ndarray]) -> List[np.ndarray]:
        return [self(frame) for frame in frames]
