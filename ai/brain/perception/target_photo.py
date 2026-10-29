"""Ținta S1 dintr-o poză (de exemplu de la telefon): aranjamentul de pe un caroiaj fotografiat din altă parte.

Aceeași detecție ca pe braț (`VisionPipeline`), cu trei diferențe:
- fără corecția obiectivului camerei de pe braț (poza vine de la alt aparat);
- fără geometria pozei de observare (nadir, înălțimea camerei): vederea e tratată ca de sus, deci centrul feței
  de sus e luat drept centrul bazei. Fotografiați cât mai de sus, perpendicular pe foaie;
- un singur cadru; pragurile de calitate a imaginii se aplică la fel.
Contează doar culoarea feței de sus. Celulele nesigure rămân de completat de mână în pagină.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from brain.configuration.models import ColorsConfig, SystemConfig
from brain.domain.cube import KNOWN_COLORS, Occupancy
from brain.domain.errors import PerceptionInputError
from brain.perception.annotate import render_annotation
from brain.perception.cells import CellAnalyzer
from brain.perception.pipeline import VisionPipeline

MAX_SIDE_PX = 1280  # pozele de telefon (12 MP) se micșorează: detecția nu câștigă nimic peste atât

EMPTY = "empty"
UNKNOWN = "unknown"


def decode_image(data: bytes) -> np.ndarray:
    """Octeții unei imagini (JPEG, PNG, ...) → BGR. Orientarea EXIF a telefonului e aplicată de OpenCV."""
    if not data:
        raise PerceptionInputError("fișierul e gol")
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise PerceptionInputError("fișierul nu e o imagine citibilă (JPEG sau PNG)")
    return image


def downscale(image: np.ndarray, max_side: int = MAX_SIDE_PX) -> Tuple[np.ndarray, Optional[Tuple[int, int]]]:
    """Micșorează la `max_side` pe latura mare; întoarce și dimensiunea originală, dacă s-a micșorat."""
    height, width = image.shape[:2]
    side = max(height, width)
    if side <= max_side:
        return image, None
    scale = max_side / float(side)
    small = cv2.resize(image, (max(1, int(round(width * scale))), max(1, int(round(height * scale)))),
                       interpolation=cv2.INTER_AREA)
    return small, (width, height)


class TopViewPipeline(VisionPipeline):
    """Detecția pentru o poză din altă parte: fără obiectivul brațului, fără nadir/înălțime, un singur cadru."""

    def __init__(self, system: SystemConfig, colors: ColorsConfig, location: str) -> None:
        loc = system.location(location)
        locations = dict(system.locations)
        locations[location] = dataclasses.replace(loc, expected_corners_px=None)  # colțurile sunt ale camerei brațului
        vision = dataclasses.replace(system.vision, frames_to_aggregate=1, min_valid_frames=1)
        super().__init__(dataclasses.replace(system, lens=None, vision=vision, locations=locations), colors)

    def _analyzer(self, location: str) -> CellAnalyzer:
        if location not in self._analyzers:  # fără nadir: fața de sus nu e deplasată și nici mărită
            self._analyzers[location] = CellAnalyzer(
                self.system.vision.cells, self.classifier, self.system.vision.rectified_cell_px,
                self.system.cube_px(location))
        return self._analyzers[location]


@dataclass(frozen=True)
class TargetCell:
    value: str  # red / yellow / green / blue / empty / unknown
    confidence: float
    uncertain: bool
    reasons: Tuple[str, ...] = ()


@dataclass
class TargetDetection:
    location: str
    rows: int
    cols: int
    actionable: bool
    reasons: Tuple[str, ...]  # codurile viziunii (de ce nu s-a găsit grila)
    cells: List[List[TargetCell]]
    annotated: np.ndarray
    image_size: Tuple[int, int]
    scaled_from: Optional[Tuple[int, int]] = None
    uncertain: List[str] = field(default_factory=list)  # celulele de verificat, ex. "r0c1"


def detect_target(pipeline: TopViewPipeline, image: np.ndarray, location: str, min_confidence: float,
                  scaled_from: Optional[Tuple[int, int]] = None) -> TargetDetection:
    """Aranjamentul din poză: per celulă culoarea feței de sus, „gol” sau „necunoscut”, cu încrederea."""
    result = pipeline.analyze([image], location)
    obs = result.observation
    cells: List[List[TargetCell]] = [[TargetCell(UNKNOWN, 0.0, True) for _ in range(obs.cols)] for _ in range(obs.rows)]
    uncertain: List[str] = []
    for cell in obs.cells:
        if cell.occupancy is Occupancy.EMPTY:
            target = TargetCell(EMPTY, round(cell.occupancy_confidence, 2), cell.occupancy_confidence < min_confidence,
                                cell.reasons)
        elif cell.occupancy is Occupancy.OCCUPIED and cell.color in KNOWN_COLORS:
            confidence = min(cell.color_confidence, cell.occupancy_confidence)
            target = TargetCell(cell.color.value, round(confidence, 2), confidence < min_confidence, cell.reasons)
        else:
            target = TargetCell(UNKNOWN, 0.0, True, cell.reasons)
        cells[cell.coord.row][cell.coord.col] = target
    for r, row in enumerate(cells):
        for c, target in enumerate(row):
            if target.uncertain:
                uncertain.append("r{}c{}".format(r, c))
    return TargetDetection(obs.location, obs.rows, obs.cols, obs.actionable, tuple(obs.reasons), cells,
                           render_annotation(result), (int(image.shape[1]), int(image.shape[0])), scaled_from,
                           uncertain if obs.actionable else [])
