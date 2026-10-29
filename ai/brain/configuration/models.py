"""Modelele de configurare (valori deja validate)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from brain.domain.cube import Color

DETECTION_METHODS = ("lines", "aruco")


@dataclass(frozen=True)
class GridGeometry:
    """Dimensiuni nominale în mm. `calibrated=False` până la măsurarea fizică."""

    cell_mm: float
    line_width_mm: float
    cube_mm: float
    calibrated: bool


@dataclass(frozen=True)
class CubeConfig:
    size_mm: float
    calibrated: bool


@dataclass(frozen=True)
class LocationConfig:
    """Un șablon 3x3. Numele trebuie să coincidă cu locul calibrat în serviciul de mișcare (calib.json)."""

    name: str
    rows: int
    cols: int
    cell_mm: float
    line_width_mm: float
    orientation_quarter_turns: int
    observe_pose: Optional[str]
    camera_height_mm: float
    nadir_uv: Tuple[float, float]
    expected_corners_px: Optional[Tuple[Tuple[float, float], ...]]


@dataclass(frozen=True)
class QualityThresholds:
    min_mean_brightness: float
    max_mean_brightness: float
    max_saturated_fraction: float
    max_dark_fraction: float
    min_contrast: float


@dataclass(frozen=True)
class LineDetectionConfig:
    adaptive_block_fraction: float
    adaptive_c: float
    dark_chroma_max: float
    min_component_fraction: float


@dataclass(frozen=True)
class ArucoConfig:
    dictionary: str
    marker_ids: Tuple[int, int, int, int]
    marker_centers_mm: Tuple[Tuple[float, float], ...]
    require_line_structure: bool


@dataclass(frozen=True)
class GeometryThresholds:
    min_area_fraction: float
    border_margin_px: float
    max_opposite_side_ratio: float
    max_adjacent_side_ratio: float
    min_interior_angle_deg: float
    max_interior_angle_deg: float
    max_condition_number: float
    min_structure_score: float
    min_line_support: float


@dataclass(frozen=True)
class WhiteBalanceConfig:
    enabled: bool
    max_gain: float
    min_reference_pixels: int


@dataclass(frozen=True)
class CellAnalysisConfig:
    roi_margin_px: int
    min_component_px: int
    occupied_min_fraction: float
    empty_max_fraction: float
    edge_gradient_threshold: float
    edge_density_max: float
    dark_object_contrast: float
    dark_object_min_px: int
    faint_saturation_min: int
    faint_fraction_max: float
    off_center_px: float
    ring_width_fraction: float
    min_color_pixels: int
    yaw_min_confidence: float
    top_face_full_min_fill: float
    top_face_icon_max_fill: float


@dataclass(frozen=True)
class VisionConfig:
    rectified_cell_px: int
    detection_method: str
    frames_to_aggregate: int
    min_valid_frames: int
    quality: QualityThresholds
    lines: LineDetectionConfig
    aruco: Optional[ArucoConfig]
    geometry: GeometryThresholds
    white_balance: WhiteBalanceConfig
    cells: CellAnalysisConfig


@dataclass(frozen=True)
class MemoryConfig:
    path: str


ARM_BACKENDS = ("sim", "team")


@dataclass(frozen=True)
class ArmConfig:
    backend: str
    team_url: str
    timeout_s: float
    calib_json: Optional[str]
    settle_s: float


PROXIMITY_MOUNTS = ("coaxial", "none")


@dataclass(frozen=True)
class ProximityConfig:
    """Senzorul de proximitate. Modelul și montajul sunt de confirmat; pragurile critice nu sunt validate."""

    enabled: bool
    model: str
    min_range_mm: float
    max_range_mm: float
    resolution_mm: float
    max_rate_hz: float
    beam_half_angle_deg: float
    samples: int
    mount: str
    sensor_above_jaw_mm: Optional[float]
    hover_above_target_mm: float
    tolerance_mm: float
    required: bool
    stop_distance_mm: Optional[float]
    calibrated: bool


@dataclass(frozen=True)
class TimingConfig:
    """Estimări inițiale (NEMĂSURATE) — se înlocuiesc cu duratele măsurate la rulări."""

    task_limit_s: float
    reserve_s: float
    observe_s: float
    pick_s: float
    place_s: float
    measured: bool


@dataclass(frozen=True)
class PreviewConfig:
    """Previzualizarea camerei prin HTTP (Pi fără ecran): flux MJPEG + ultima analiză adnotată."""

    host: str
    port: int
    max_fps: float
    jpeg_quality: int
    analyze_interval_s: float
    save_dir: str


MATCH_FACE_POLICIES = ("prefer", "require", "ignore")
CAMERA_VERIFY_POLICIES = ("each_move", "end")


@dataclass(frozen=True)
class CostConfig:
    radial_deg_per_mm: float
    adjacent_stack_penalty_deg: float
    location_switch_deg: float


@dataclass(frozen=True)
class PlanningConfig:
    max_tower_level: int
    min_confidence: float
    match_top_face: str
    active_locations: Tuple[str, ...]
    staging_locations: Tuple[str, ...]
    buffer_points: Tuple[str, ...]
    buffer_max_level: int
    max_pick_attempts: int
    camera_verify: str
    max_observe_attempts: int
    cost: CostConfig


@dataclass(frozen=True)
class SystemConfig:
    cube: CubeConfig
    locations: Dict[str, LocationConfig]
    vision: VisionConfig
    memory: MemoryConfig
    arm: ArmConfig
    proximity: ProximityConfig
    timing: TimingConfig
    planning: PlanningConfig
    preview: PreviewConfig
    source_file: str

    def px_per_mm(self, location: str) -> float:
        """Scara imaginii redresate a unei locații (px/mm)."""
        return self.vision.rectified_cell_px / self.location(location).cell_mm

    def cube_px(self, location: str) -> float:
        """Latura nominală a cubului în imaginea redresată a locației (fără paralaxă)."""
        return self.cube.size_mm * self.px_per_mm(location)

    def grid_geometry(self, location: str) -> GridGeometry:
        loc = self.location(location)
        return GridGeometry(loc.cell_mm, loc.line_width_mm, self.cube.size_mm, self.cube.calibrated)

    def location(self, name: str) -> LocationConfig:
        try:
            return self.locations[name]
        except KeyError:
            raise KeyError(
                "locația '{}' nu e definită în {} (disponibile: {})".format(
                    name, self.source_file, ", ".join(sorted(self.locations))
                )
            ) from None


@dataclass(frozen=True)
class ColorClass:
    color: Color
    hue_ranges: Tuple[Tuple[int, int], ...]


@dataclass(frozen=True)
class ColorsConfig:
    """Praguri HSV în convenția OpenCV: H în [0, 179], S și V în [0, 255]."""

    saturation_min: int
    value_min: int
    min_dominance: float
    classes: Tuple[ColorClass, ...]
    calibrated: bool
    source_file: str


@dataclass(frozen=True)
class ZoneConfig:
    """O zonă a hărții = un loc de tip „point” din calib.json (npm run calib -- zona NUME)."""

    name: str
    accepts: Tuple[Color, ...]


UNKNOWN_COLOR_POLICIES = ("leave", "staging")


@dataclass(frozen=True)
class SortMapConfig:
    zones: Tuple[ZoneConfig, ...]
    max_level: int
    unknown_color_policy: str
    calibrated: bool
    source_file: str

    def zone_for(self, color: Color) -> Optional[ZoneConfig]:
        for zone in self.zones:
            if color in zone.accepts:
                return zone
        return None
