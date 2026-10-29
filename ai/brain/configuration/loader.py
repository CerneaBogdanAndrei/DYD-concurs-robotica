"""Citirea fișierelor YAML și construirea modelelor de configurare validate."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import yaml

from brain.configuration.models import (
    ARM_BACKENDS,
    CAMERA_VERIFY_POLICIES,
    DETECTION_METHODS,
    MATCH_FACE_POLICIES,
    UNKNOWN_COLOR_POLICIES,
    ArmConfig,
    ArucoConfig,
    CellAnalysisConfig,
    ColorClass,
    ColorsConfig,
    CostConfig,
    CubeConfig,
    GeometryThresholds,
    LineDetectionConfig,
    LensConfig,
    LocationConfig,
    MemoryConfig,
    PlanningConfig,
    PreviewConfig,
    QualityThresholds,
    SortMapConfig,
    SystemConfig,
    TimingConfig,
    VisionConfig,
    WhiteBalanceConfig,
    ZoneConfig,
)
from brain.configuration.validation import Section
from brain.domain.cube import KNOWN_COLORS, Color
from brain.domain.errors import ConfigError

SYSTEM_SCHEMA_VERSION = 2
COLORS_SCHEMA_VERSION = 1
SORT_MAP_SCHEMA_VERSION = 1
SYSTEM_FILE = "system.yaml"
COLORS_FILE = "colors.yaml"
SORT_MAP_FILE = "sort_map.yaml"
LENS_FILE = "lens.yaml"  # opțional, scris de `brain calib camera`
LENS_SCHEMA_VERSION = 1

PathLike = Union[str, "os.PathLike[str]"]


@dataclass(frozen=True)
class AppConfig:
    system: SystemConfig
    colors: ColorsConfig
    sort_map: SortMapConfig
    config_dir: str


def default_config_dir() -> Path:
    """`$BRAIN_CONFIG_DIR`, apoi `./config`, apoi `config/` de lângă pachet."""
    env = os.environ.get("BRAIN_CONFIG_DIR")
    if env:
        return Path(env)
    cwd_config = Path.cwd() / "config"
    if cwd_config.is_dir():
        return cwd_config
    return Path(__file__).resolve().parents[2] / "config"


def read_yaml(path: PathLike) -> Section:
    file = str(path)
    try:
        with open(file, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except FileNotFoundError:
        raise ConfigError(file, "<fișier>", None, "fișierul nu există") from None
    except yaml.YAMLError as exc:
        raise ConfigError(file, "<yaml>", None, "YAML invalid: {}".format(exc)) from None
    except OSError as exc:
        raise ConfigError(file, "<fișier>", None, "nu poate fi citit: {}".format(exc)) from None
    return Section(data if data is not None else {}, file)


def _check_schema(root: Section, expected: int) -> None:
    version = root.integer("schema_version")
    if version != expected:
        raise root.error("schema_version", version, "versiune nesuportată (așteptat {})".format(expected))


def load_system_config(path: PathLike) -> SystemConfig:
    root = read_yaml(path)
    root.reject_unknown(
        ["schema_version", "cube", "locations", "vision", "memory", "arm", "timing", "planning", "preview"]
    )
    _check_schema(root, SYSTEM_SCHEMA_VERSION)
    cube_section = root.section("cube")
    cube_section.reject_unknown(["size_mm", "calibrated"])
    cube = CubeConfig(
        size_mm=cube_section.number("size_mm", minimum=5.0, maximum=100.0),
        calibrated=cube_section.boolean("calibrated"),
    )
    locations = _parse_locations(root.section("locations"), cube.size_mm)
    memory_section = root.section("memory")
    memory_section.reject_unknown(["path"])
    planning = _parse_planning(root.section("planning"), locations)
    return SystemConfig(
        cube=cube,
        locations=locations,
        vision=_parse_vision(root.section("vision")),
        memory=MemoryConfig(path=memory_section.string("path")),
        arm=_parse_arm(root.section("arm")),
        timing=_parse_timing(root.section("timing")),
        planning=planning,
        preview=_parse_preview(root.section("preview")),
        source_file=root.file,
        lens=load_lens_config(Path(str(path)).parent / LENS_FILE),
    )


def load_lens_config(path: PathLike) -> Optional[LensConfig]:
    """Parametrii obiectivului, dacă fișierul există; altfel None (cadrele nu se corectează)."""
    if not Path(str(path)).is_file():
        return None
    root = read_yaml(path)
    root.reject_unknown(["schema_version", "image_size", "camera_matrix", "dist_coeffs", "rms_px", "frames_used",
                         "pattern", "square_mm", "calibrated"])
    _check_schema(root, LENS_SCHEMA_VERSION)
    size = root.raw("image_size")
    if not (isinstance(size, list) and len(size) == 2 and all(isinstance(v, int) and v > 0 for v in size)):
        raise root.error("image_size", size, "trebuie [lățime, înălțime] în px")
    matrix = root.raw("camera_matrix")
    if not (isinstance(matrix, list) and len(matrix) == 3 and all(
            isinstance(row, list) and len(row) == 3 and all(isinstance(v, (int, float)) for v in row) for row in matrix)):
        raise root.error("camera_matrix", matrix, "trebuie o matrice 3x3 de numere")
    if not (matrix[0][0] > 0 and matrix[1][1] > 0):
        raise root.error("camera_matrix", matrix, "distanțele focale fx, fy trebuie să fie pozitive")
    dist = root.raw("dist_coeffs")
    if not (isinstance(dist, list) and 4 <= len(dist) <= 8 and all(isinstance(v, (int, float)) for v in dist)):
        raise root.error("dist_coeffs", dist, "trebuie 4..8 numere (k1, k2, p1, p2[, k3...])")
    return LensConfig(
        image_size=(int(size[0]), int(size[1])),
        camera_matrix=tuple(tuple(float(v) for v in row) for row in matrix),
        dist_coeffs=tuple(float(v) for v in dist),
        rms_px=root.number("rms_px", minimum=0.0),
        calibrated=root.boolean("calibrated"),
    )


def _parse_preview(s: Section) -> PreviewConfig:
    s.reject_unknown(["host", "port", "max_fps", "jpeg_quality", "analyze_interval_s", "save_dir"])
    return PreviewConfig(
        host=s.string("host"),
        port=s.integer("port", minimum=1, maximum=65535),
        max_fps=s.number("max_fps", minimum=0.5, maximum=60.0),
        jpeg_quality=s.integer("jpeg_quality", minimum=30, maximum=100),
        analyze_interval_s=s.number("analyze_interval_s", minimum=0.2, maximum=60.0),
        save_dir=s.string("save_dir"),
    )


def _point(section: Section, key: str) -> Tuple[float, float]:
    value = section.raw(key)
    if (
        not isinstance(value, (list, tuple))
        or len(value) != 2
        or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in value)
    ):
        raise section.error(key, value, "trebuie [x, y] numeric")
    return (float(value[0]), float(value[1]))


def _point_list(section: Section, key: str, count: int) -> Tuple[Tuple[float, float], ...]:
    raw = section.sequence(key)
    if len(raw) != count:
        raise section.error(key, raw, "trebuie exact {} puncte [x, y]".format(count))
    points: List[Tuple[float, float]] = []
    for index, point in enumerate(raw):
        if (
            not isinstance(point, (list, tuple))
            or len(point) != 2
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in point)
        ):
            raise section.error("{}[{}]".format(key, index), point, "trebuie [x, y] numeric")
        points.append((float(point[0]), float(point[1])))
    return tuple(points)


def _parse_locations(section: Section, cube_mm: float) -> Dict[str, LocationConfig]:
    if not section.data:
        raise ConfigError(section.file, section.path, {}, "trebuie definită cel puțin o locație")
    result: Dict[str, LocationConfig] = {}
    for name in section.data:
        if not isinstance(name, str) or not name:
            raise section.error(str(name), name, "numele locației trebuie să fie un șir nevid")
        loc = section.section(name)
        loc.reject_unknown(
            [
                "rows",
                "cols",
                "cell_mm",
                "line_width_mm",
                "orientation_quarter_turns",
                "observe_pose",
                "camera_height_mm",
                "nadir_uv",
                "expected_corners_px",
            ]
        )
        rows = loc.integer("rows", minimum=1, maximum=10)
        cols = loc.integer("cols", minimum=1, maximum=10)
        turns = loc.integer("orientation_quarter_turns", default=0, minimum=0, maximum=3)
        if turns % 2 == 1 and rows != cols:
            raise loc.error("orientation_quarter_turns", turns, "rotațiile de 90°/270° cer rows == cols")
        cell = loc.number("cell_mm", minimum=5.0, maximum=200.0)
        if cube_mm >= cell:
            raise loc.error("cell_mm", cell, "celula trebuie să fie mai mare decât cubul ({} mm)".format(cube_mm))
        line = loc.number("line_width_mm", minimum=0.0, exclusive_minimum=True)
        if line >= cell / 4:
            raise loc.error("line_width_mm", line, "prea mare față de cell_mm={}".format(cell))
        height = loc.number("camera_height_mm", minimum=0.0, exclusive_minimum=True)
        if height <= cube_mm * 5:
            raise loc.error("camera_height_mm", height, "camera trebuie să fie mult deasupra unui turn de 4 cuburi")
        nadir = _point(loc, "nadir_uv")
        # Camera poate privi oblic: punctul de sub ea poate fi în afara caroiajului, dar nu oricât de departe.
        if not (-2.0 * cols <= nadir[0] <= 3.0 * cols and -2.0 * rows <= nadir[1] <= 3.0 * rows):
            raise loc.error("nadir_uv", list(nadir), "prea departe de caroiaj (permis: -2..3 caroiaje pe fiecare axă)")
        corners = None
        if loc.raw("expected_corners_px", None) is not None:
            corners = _point_list(loc, "expected_corners_px", 4)
        result[name] = LocationConfig(
            name=name,
            rows=rows,
            cols=cols,
            cell_mm=cell,
            line_width_mm=line,
            orientation_quarter_turns=turns,
            observe_pose=loc.optional_string("observe_pose"),
            camera_height_mm=height,
            nadir_uv=nadir,
            expected_corners_px=corners,
        )
    return result


def _parse_arm(s: Section) -> ArmConfig:
    s.reject_unknown(["backend", "team_url", "timeout_s", "calib_json", "settle_s"])
    url = s.string("team_url")
    if not url.startswith(("http://", "https://")):
        raise s.error("team_url", url, "trebuie să înceapă cu http:// sau https://")
    return ArmConfig(
        backend=s.string("backend", choices=ARM_BACKENDS),
        team_url=url,
        timeout_s=s.number("timeout_s", minimum=1.0, maximum=600.0),
        calib_json=s.optional_string("calib_json"),
        settle_s=s.number("settle_s", minimum=0.0, maximum=10.0),
    )


def _parse_timing(s: Section) -> TimingConfig:
    s.reject_unknown(["task_limit_s", "reserve_s", "observe_s", "pick_s", "place_s", "measured"])
    limit = s.number("task_limit_s", minimum=1.0)
    reserve = s.number("reserve_s", minimum=0.0)
    if reserve >= limit:
        raise s.error("reserve_s", reserve, "trebuie să fie < task_limit_s={}".format(limit))
    return TimingConfig(
        task_limit_s=limit,
        reserve_s=reserve,
        observe_s=s.number("observe_s", minimum=0.0),
        pick_s=s.number("pick_s", minimum=0.0),
        place_s=s.number("place_s", minimum=0.0),
        measured=s.boolean("measured"),
    )


def _parse_planning(s: Section, locations: Dict[str, LocationConfig]) -> PlanningConfig:
    s.reject_unknown(
        [
            "max_tower_level",
            "min_confidence",
            "match_top_face",
            "active_locations",
            "staging_locations",
            "buffer_points",
            "buffer_max_level",
            "max_pick_attempts",
            "camera_verify",
            "max_observe_attempts",
            "cost",
        ]
    )
    active = s.sequence("active_locations", min_length=1)
    for index, name in enumerate(active):
        if name not in locations:
            raise s.error("active_locations[{}]".format(index), name, "locație nedefinită în locations")
    staging = s.sequence("staging_locations")
    for index, name in enumerate(staging):
        if name not in active:
            raise s.error("staging_locations[{}]".format(index), name, "trebuie să fie una din active_locations")
    buffers = s.sequence("buffer_points", default=[])
    for index, name in enumerate(buffers):
        if not isinstance(name, str) or not name or name in locations:
            raise s.error("buffer_points[{}]".format(index), name, "trebuie un nume de punct nevid, diferit de locații")
    if len(set(buffers)) != len(buffers):
        raise s.error("buffer_points", buffers, "nume duplicate")
    cost = s.section("cost")
    cost.reject_unknown(["radial_deg_per_mm", "adjacent_stack_penalty_deg", "location_switch_deg", "adjacent_cube_penalty_deg"])
    return PlanningConfig(
        max_tower_level=s.integer("max_tower_level", minimum=0, maximum=8),
        min_confidence=s.number("min_confidence", minimum=0.0, maximum=1.0),
        match_top_face=s.string("match_top_face", choices=MATCH_FACE_POLICIES),
        active_locations=tuple(active),
        staging_locations=tuple(staging),
        buffer_points=tuple(buffers),
        buffer_max_level=s.integer("buffer_max_level", default=0, minimum=0, maximum=8),
        max_pick_attempts=s.integer("max_pick_attempts", minimum=1, maximum=10),
        camera_verify=s.string("camera_verify", choices=CAMERA_VERIFY_POLICIES),
        max_observe_attempts=s.integer("max_observe_attempts", minimum=1, maximum=10),
        cost=CostConfig(
            radial_deg_per_mm=cost.number("radial_deg_per_mm", minimum=0.0),
            adjacent_stack_penalty_deg=cost.number("adjacent_stack_penalty_deg", minimum=0.0),
            location_switch_deg=cost.number("location_switch_deg", minimum=0.0),
            adjacent_cube_penalty_deg=cost.number("adjacent_cube_penalty_deg", default=0.0, minimum=0.0),
        ),
    )


def _parse_vision(section: Section) -> VisionConfig:
    section.reject_unknown(
        [
            "rectified_cell_px",
            "detection_method",
            "frames_to_aggregate",
            "min_valid_frames",
            "quality",
            "lines",
            "aruco",
            "geometry",
            "white_balance",
            "cells",
        ]
    )
    method = section.string("detection_method", choices=DETECTION_METHODS)
    frames = section.integer("frames_to_aggregate", minimum=1, maximum=30)
    min_valid = section.integer("min_valid_frames", minimum=1)
    if min_valid > frames:
        raise section.error("min_valid_frames", min_valid, "nu poate depăși frames_to_aggregate={}".format(frames))
    aruco = _parse_aruco(section.section("aruco")) if section.has("aruco") and section.raw("aruco") is not None else None
    if method == "aruco" and aruco is None:
        raise section.error("aruco", None, "detection_method=aruco cere secțiunea aruco")
    return VisionConfig(
        rectified_cell_px=section.integer("rectified_cell_px", minimum=40, maximum=400),
        detection_method=method,
        frames_to_aggregate=frames,
        min_valid_frames=min_valid,
        quality=_parse_quality(section.section("quality")),
        lines=_parse_lines(section.section("lines")),
        aruco=aruco,
        geometry=_parse_geometry(section.section("geometry")),
        white_balance=_parse_white_balance(section.section("white_balance")),
        cells=_parse_cells(section.section("cells")),
    )


def _parse_quality(s: Section) -> QualityThresholds:
    s.reject_unknown(
        ["min_mean_brightness", "max_mean_brightness", "max_saturated_fraction", "max_dark_fraction", "min_contrast"]
    )
    low = s.number("min_mean_brightness", minimum=0, maximum=255)
    high = s.number("max_mean_brightness", minimum=0, maximum=255)
    if low >= high:
        raise s.error("max_mean_brightness", high, "trebuie să fie > min_mean_brightness={}".format(low))
    return QualityThresholds(
        min_mean_brightness=low,
        max_mean_brightness=high,
        max_saturated_fraction=s.number("max_saturated_fraction", minimum=0, maximum=1),
        max_dark_fraction=s.number("max_dark_fraction", minimum=0, maximum=1),
        min_contrast=s.number("min_contrast", minimum=0, maximum=255),
    )


def _parse_lines(s: Section) -> LineDetectionConfig:
    s.reject_unknown(["adaptive_block_fraction", "adaptive_c", "dark_chroma_max", "min_component_fraction"])
    return LineDetectionConfig(
        adaptive_block_fraction=s.number("adaptive_block_fraction", minimum=0.005, maximum=0.5),
        adaptive_c=s.number("adaptive_c", minimum=0, maximum=255),
        dark_chroma_max=s.number("dark_chroma_max", minimum=0, maximum=255),
        min_component_fraction=s.number("min_component_fraction", minimum=0, maximum=1),
    )


def _parse_aruco(s: Section) -> ArucoConfig:
    s.reject_unknown(["dictionary", "marker_ids", "marker_centers_mm", "require_line_structure"])
    dictionary = s.string("dictionary")
    if not dictionary.startswith("DICT_"):
        raise s.error("dictionary", dictionary, "trebuie să fie un dicționar OpenCV, de ex. DICT_4X4_50")
    ids_raw = s.sequence("marker_ids")
    if len(ids_raw) != 4 or any(isinstance(i, bool) or not isinstance(i, int) or i < 0 for i in ids_raw):
        raise s.error("marker_ids", ids_raw, "trebuie 4 ID-uri întregi >= 0, în ordinea TL, TR, BR, BL")
    if len(set(ids_raw)) != 4:
        raise s.error("marker_ids", ids_raw, "ID-urile trebuie să fie distincte")
    centers_raw = s.sequence("marker_centers_mm")
    centers: List[Tuple[float, float]] = []
    if len(centers_raw) != 4:
        raise s.error("marker_centers_mm", centers_raw, "trebuie 4 puncte [x, y] în mm")
    for index, point in enumerate(centers_raw):
        if (
            not isinstance(point, (list, tuple))
            or len(point) != 2
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in point)
        ):
            raise s.error("marker_centers_mm[{}]".format(index), point, "trebuie [x, y] numeric")
        centers.append((float(point[0]), float(point[1])))
    return ArucoConfig(
        dictionary=dictionary,
        marker_ids=(ids_raw[0], ids_raw[1], ids_raw[2], ids_raw[3]),
        marker_centers_mm=tuple(centers),
        require_line_structure=s.boolean("require_line_structure"),
    )


def _parse_geometry(s: Section) -> GeometryThresholds:
    s.reject_unknown(
        [
            "min_area_fraction",
            "border_margin_px",
            "max_opposite_side_ratio",
            "max_adjacent_side_ratio",
            "min_interior_angle_deg",
            "max_interior_angle_deg",
            "max_condition_number",
            "min_structure_score",
            "min_line_support",
        ]
    )
    min_angle = s.number("min_interior_angle_deg", minimum=1, maximum=89)
    max_angle = s.number("max_interior_angle_deg", minimum=91, maximum=179)
    return GeometryThresholds(
        min_area_fraction=s.number("min_area_fraction", minimum=0, maximum=1),
        border_margin_px=s.number("border_margin_px", minimum=0),
        max_opposite_side_ratio=s.number("max_opposite_side_ratio", minimum=1),
        max_adjacent_side_ratio=s.number("max_adjacent_side_ratio", minimum=1),
        min_interior_angle_deg=min_angle,
        max_interior_angle_deg=max_angle,
        max_condition_number=s.number("max_condition_number", minimum=1),
        min_structure_score=s.number("min_structure_score", minimum=0, maximum=1),
        min_line_support=s.number("min_line_support", minimum=0, maximum=1),
    )


def _parse_white_balance(s: Section) -> WhiteBalanceConfig:
    s.reject_unknown(["enabled", "max_gain", "min_reference_pixels"])
    return WhiteBalanceConfig(
        enabled=s.boolean("enabled"),
        max_gain=s.number("max_gain", minimum=1.0),
        min_reference_pixels=s.integer("min_reference_pixels", minimum=1),
    )


def _parse_cells(s: Section) -> CellAnalysisConfig:
    s.reject_unknown(
        [
            "roi_margin_px",
            "min_component_px",
            "occupied_min_fraction",
            "empty_max_fraction",
            "edge_gradient_threshold",
            "edge_density_max",
            "dark_object_contrast",
            "dark_object_min_px",
            "faint_saturation_min",
            "faint_fraction_max",
            "off_center_px",
            "ring_width_fraction",
            "min_color_pixels",
            "yaw_min_confidence",
            "top_face_full_min_fill",
            "top_face_icon_max_fill",
        ]
    )
    occupied_min = s.number("occupied_min_fraction", minimum=0, maximum=1, exclusive_minimum=True)
    empty_max = s.number("empty_max_fraction", minimum=0, maximum=1)
    if empty_max >= occupied_min:
        raise s.error("empty_max_fraction", empty_max, "trebuie să fie < occupied_min_fraction={}".format(occupied_min))
    full_min = s.number("top_face_full_min_fill", minimum=0, maximum=1)
    icon_max = s.number("top_face_icon_max_fill", minimum=0, maximum=1)
    if icon_max >= full_min:
        raise s.error("top_face_icon_max_fill", icon_max, "trebuie să fie < top_face_full_min_fill={}".format(full_min))
    return CellAnalysisConfig(
        roi_margin_px=s.integer("roi_margin_px", minimum=0, maximum=45),
        min_component_px=s.integer("min_component_px", minimum=1),
        occupied_min_fraction=occupied_min,
        empty_max_fraction=empty_max,
        edge_gradient_threshold=s.number("edge_gradient_threshold", minimum=0),
        edge_density_max=s.number("edge_density_max", minimum=0, maximum=1),
        dark_object_contrast=s.number("dark_object_contrast", minimum=0, maximum=255),
        dark_object_min_px=s.integer("dark_object_min_px", minimum=1),
        faint_saturation_min=s.integer("faint_saturation_min", minimum=0, maximum=255),
        faint_fraction_max=s.number("faint_fraction_max", minimum=0, maximum=1),
        off_center_px=s.number("off_center_px", minimum=0),
        ring_width_fraction=s.number("ring_width_fraction", minimum=0.02, maximum=0.5),
        min_color_pixels=s.integer("min_color_pixels", minimum=1),
        yaw_min_confidence=s.number("yaw_min_confidence", minimum=0, maximum=1),
        top_face_full_min_fill=full_min,
        top_face_icon_max_fill=icon_max,
    )


def load_colors_config(path: PathLike) -> ColorsConfig:
    root = read_yaml(path)
    root.reject_unknown(["schema_version", "saturation_min", "value_min", "min_dominance", "colors", "calibrated"])
    _check_schema(root, COLORS_SCHEMA_VERSION)
    colors_section = root.section("colors")
    allowed = [c.value for c in KNOWN_COLORS]
    colors_section.reject_unknown(allowed)
    classes: List[ColorClass] = []
    owner: Dict[int, str] = {}
    for color in KNOWN_COLORS:
        entry = colors_section.section(color.value)
        entry.reject_unknown(["hue_ranges"])
        ranges_raw = entry.sequence("hue_ranges", min_length=1)
        ranges: List[Tuple[int, int]] = []
        for index, pair in enumerate(ranges_raw):
            key = "hue_ranges[{}]".format(index)
            if (
                not isinstance(pair, (list, tuple))
                or len(pair) != 2
                or any(isinstance(v, bool) or not isinstance(v, int) for v in pair)
            ):
                raise entry.error(key, pair, "trebuie [min, max] întregi")
            low, high = int(pair[0]), int(pair[1])
            if not (0 <= low <= high <= 179):
                raise entry.error(key, pair, "trebuie 0 <= min <= max <= 179 (H în convenția OpenCV)")
            for hue in range(low, high + 1):
                if hue in owner and owner[hue] != color.value:
                    raise entry.error(key, pair, "se suprapune cu intervalul culorii '{}' (H={})".format(owner[hue], hue))
                owner[hue] = color.value
            ranges.append((low, high))
        classes.append(ColorClass(color=Color(color.value), hue_ranges=tuple(ranges)))
    return ColorsConfig(
        saturation_min=root.integer("saturation_min", minimum=0, maximum=255),
        value_min=root.integer("value_min", minimum=0, maximum=255),
        min_dominance=root.number("min_dominance", minimum=0.0, maximum=1.0, exclusive_minimum=True),
        classes=tuple(classes),
        calibrated=root.boolean("calibrated"),
        source_file=root.file,
    )


def load_sort_map_config(path: PathLike) -> SortMapConfig:
    root = read_yaml(path)
    root.reject_unknown(["schema_version", "zones", "max_level", "unknown_color_policy", "calibrated"])
    _check_schema(root, SORT_MAP_SCHEMA_VERSION)
    zones_section = root.section("zones")
    zones: List[ZoneConfig] = []
    owner: Dict[Color, str] = {}
    allowed = [c.value for c in KNOWN_COLORS]
    for name in zones_section.data:
        zone = zones_section.section(str(name))
        zone.reject_unknown(["accepts"])
        accepts_raw = zone.sequence("accepts", min_length=1)
        accepts: List[Color] = []
        for index, value in enumerate(accepts_raw):
            if value not in allowed:
                raise zone.error("accepts[{}]".format(index), value, "culoare permisă: {}".format(", ".join(allowed)))
            color = Color(value)
            if color in owner:
                raise zone.error("accepts[{}]".format(index), value, "culoarea e deja acceptată de zona '{}'".format(owner[color]))
            owner[color] = str(name)
            accepts.append(color)
        zones.append(ZoneConfig(name=str(name), accepts=tuple(accepts)))
    return SortMapConfig(
        zones=tuple(zones),
        max_level=root.integer("max_level", minimum=0, maximum=8),
        unknown_color_policy=root.string("unknown_color_policy", choices=UNKNOWN_COLOR_POLICIES),
        calibrated=root.boolean("calibrated"),
        source_file=root.file,
    )


def load_app_config(config_dir: Any = None) -> AppConfig:
    directory = Path(config_dir) if config_dir is not None else default_config_dir()
    return AppConfig(
        system=load_system_config(directory / SYSTEM_FILE),
        colors=load_colors_config(directory / COLORS_FILE),
        sort_map=load_sort_map_config(directory / SORT_MAP_FILE),
        config_dir=str(directory),
    )
