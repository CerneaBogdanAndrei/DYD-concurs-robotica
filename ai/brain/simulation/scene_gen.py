"""Generator determinist de scene sintetice: caroiaj, cuburi, perspectivă, paralaxă, zgomot.

Modelul e o aproximare: masa e randată de sus (ortografic), fețele superioare ale cuburilor sunt
deplasate ca și cum ar fi văzute de o cameră aflată la înălțimea `camera_height_mm` deasupra
punctului nadir, apoi întreaga masă e proiectată în imagine printr-o omografie aleatoare.
Același seed produce aceleași imagini pe aceeași versiune de numpy/OpenCV.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from brain.configuration.models import ArucoConfig, GridGeometry
from brain.domain.cube import KNOWN_COLORS, normalize_yaw_deg
from brain.perception.grid_aruco import render_marker

KIND_POSITIVE = "positive"
NEGATIVE_KINDS = ("blank", "no_grid", "overexposed", "underexposed", "grid_out_of_frame", "degenerate")
SPECIAL_KINDS = ("occluded_corner", "ambiguous_color", "missing_marker")
ALL_KINDS = (KIND_POSITIVE,) + NEGATIVE_KINDS + SPECIAL_KINDS
DIFFICULTIES = ("nominal", "hard")

AMBIGUOUS = "ambiguous"

# Culori de bază în HSV OpenCV (H 0..179). Valori sintetice, nu măsurători ale cuburilor reale.
_BASE_HSV = {"red": (0, 200, 190), "yellow": (27, 210, 225), "green": (62, 180, 150), "blue": (110, 200, 165)}
_AMBIGUOUS_HSV = {"orange": (14, 200, 215), "desaturated": (None, 45, 190)}


@dataclass(frozen=True)
class _Params:
    grid_side_px: Tuple[float, float]
    rotation_deg: float
    shift_px: float
    perspective: float
    camera_height_mm: Tuple[float, float]
    nadir_offset_mm: float
    cube_offset_mm: float
    gradient: float
    cast: float
    exposure: Tuple[float, float]
    blur_sigma: Tuple[float, float]
    noise_sigma: Tuple[float, float]
    colored_side_probability: float
    hue_jitter: int
    sat_jitter: int
    val_jitter: int


_PARAMS = {
    "nominal": _Params((250, 320), 10, 30, 0.03, (280, 400), 15, 2.5, 0.10, 0.06, (0.92, 1.05), (0.0, 0.8),
                       (1.5, 3.5), 0.15, 4, 25, 20),
    "hard": _Params((210, 330), 18, 45, 0.06, (230, 400), 30, 3.5, 0.20, 0.12, (0.80, 1.08), (0.0, 1.4),
                    (2.0, 6.0), 0.35, 6, 40, 35),
}


@dataclass(frozen=True)
class RenderSettings:
    image_size: Tuple[int, int] = (640, 480)
    px_per_mm: float = 4.0
    table_margin_mm: float = 70.0
    border_band_mm: float = 2.5
    marker_size_mm: float = 20.0


@dataclass(frozen=True)
class CubeTruth:
    row: int
    col: int
    color: str  # red / yellow / green / blue / ambiguous
    yaw_deg: float  # [-45, 45), sens antiorar văzut de sus
    top_face: str  # full / icon
    center_mm: Tuple[float, float]
    note: str = ""


@dataclass(frozen=True)
class SceneTruth:
    seed: int
    kind: str
    difficulty: str
    use_aruco: bool
    rows: int
    cols: int
    cubes: Tuple[CubeTruth, ...]
    grid_corners_px: Optional[Tuple[Tuple[float, float], ...]]
    expect_actionable: Optional[bool]  # None = oricare e acceptabil, dacă e corect
    camera: Dict[str, float] = field(default_factory=dict)

    def cube_at(self, row: int, col: int) -> Optional[CubeTruth]:
        for cube in self.cubes:
            if cube.row == row and cube.col == col:
                return cube
        return None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class SyntheticScene:
    frames: List[np.ndarray]
    truth: SceneTruth


def _square(center: Sequence[float], side: float, yaw_deg: float) -> np.ndarray:
    """Colțurile unui pătrat rotit cu `yaw_deg` antiorar pe ecran (axa y în jos)."""
    a = math.radians(yaw_deg)
    c, s = math.cos(a), math.sin(a)
    h = side / 2.0
    pts = []
    for u, v in ((-h, -h), (h, -h), (h, h), (-h, h)):
        pts.append((center[0] + u * c + v * s, center[1] - u * s + v * c))
    return np.array(pts, dtype=np.float64)


def _hsv_to_bgr(h: int, s: int, v: int) -> Tuple[int, int, int]:
    pixel = np.array([[[h % 180, int(np.clip(s, 0, 255)), int(np.clip(v, 0, 255))]]], dtype=np.uint8)
    b, g, r = cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0]
    return int(b), int(g), int(r)


class _Canvas:
    """Vedere de sus a mesei, în mm → px (originea în centrul liniei din colțul stânga-sus al grilei)."""

    SHIFT = 4  # sub-pixel pentru fillPoly

    def __init__(self, grid: GridGeometry, rows: int, cols: int, settings: RenderSettings, table_bgr: np.ndarray) -> None:
        self.grid = grid
        self.rows = rows
        self.cols = cols
        self.settings = settings
        self.scale = settings.px_per_mm
        self.margin = settings.table_margin_mm
        width = int(round((cols * grid.cell_mm + 2 * self.margin) * self.scale))
        height = int(round((rows * grid.cell_mm + 2 * self.margin) * self.scale))
        self.image = np.empty((height, width, 3), dtype=np.float32)
        self.image[:] = table_bgr

    def to_px(self, points_mm: np.ndarray) -> np.ndarray:
        return (np.asarray(points_mm, dtype=np.float64) + self.margin) * self.scale

    def fill(self, points_mm: np.ndarray, color: Sequence[float]) -> None:
        pts = np.rint(self.to_px(points_mm) * (1 << self.SHIFT)).astype(np.int32).reshape(-1, 1, 2)
        cv2.fillPoly(self.image, [pts], tuple(float(c) for c in color), cv2.LINE_AA, self.SHIFT)


def _draw_grid(canvas: _Canvas) -> None:
    g = canvas.grid
    w = g.line_width_mm / 2.0
    width_mm = canvas.cols * g.cell_mm
    height_mm = canvas.rows * g.cell_mm
    ink = (32.0, 32.0, 34.0)
    for k in range(canvas.cols + 1):
        x = k * g.cell_mm
        canvas.fill(np.array([[x - w, -w], [x + w, -w], [x + w, height_mm + w], [x - w, height_mm + w]]), ink)
    for k in range(canvas.rows + 1):
        y = k * g.cell_mm
        canvas.fill(np.array([[-w, y - w], [width_mm + w, y - w], [width_mm + w, y + w], [-w, y + w]]), ink)


def _draw_markers(canvas: _Canvas, aruco: ArucoConfig, skip_index: Optional[int]) -> None:
    size_px = int(round(canvas.settings.marker_size_mm * canvas.scale))
    for index, (marker_id, center) in enumerate(zip(aruco.marker_ids, aruco.marker_centers_mm)):
        if index == skip_index:
            continue
        marker = render_marker(aruco.dictionary, marker_id, size_px).astype(np.float32)
        cx, cy = canvas.to_px(np.array([center]))[0]
        x0 = int(round(cx - size_px / 2.0))
        y0 = int(round(cy - size_px / 2.0))
        region = canvas.image[y0:y0 + size_px, x0:x0 + size_px]
        if region.shape[:2] != (size_px, size_px):
            continue
        shade = 30.0 + marker * (canvas.image[y0, x0].mean() - 30.0) / 255.0
        region[:] = shade[..., None]


def _sample_cubes(
    rng: np.random.Generator, grid: GridGeometry, rows: int, cols: int, p: _Params, kind: str
) -> List[Tuple[int, int, float, Tuple[float, float]]]:
    """Alege celule, yaw și deplasare astfel încât amprentele cuburilor să nu se suprapună fizic."""
    total = rows * cols
    count = int(rng.integers(1 if kind == "ambiguous_color" else 0, total + 1))
    cells = rng.permutation(total)[:count]
    placed: List[np.ndarray] = []
    result = []
    for index in sorted(int(c) for c in cells):
        row, col = divmod(index, cols)
        for _ in range(60):
            yaw = float(rng.uniform(-45.0, 45.0))
            offset = rng.uniform(-p.cube_offset_mm, p.cube_offset_mm, size=2)
            center = ((col + 0.5) * grid.cell_mm + offset[0], (row + 0.5) * grid.cell_mm + offset[1])
            footprint = _square(center, grid.cube_mm + 1.0, yaw).astype(np.float32)
            if all(cv2.intersectConvexConvex(footprint, other)[0] <= 0.0 for other in placed):
                placed.append(footprint)
                result.append((row, col, yaw, (float(center[0]), float(center[1]))))
                break
    return result


def generate_scene(
    seed: int,
    grid: GridGeometry,
    kind: str = KIND_POSITIVE,
    difficulty: str = "nominal",
    frames: int = 5,
    rows: int = 3,
    cols: int = 3,
    aruco: Optional[ArucoConfig] = None,
    settings: Optional[RenderSettings] = None,
) -> SyntheticScene:
    """Generează `frames` cadre ale aceleiași scene (diferă doar zgomotul) și adevărul de referință.

    `aruco` != None desenează markerii (necesar pentru `missing_marker`).
    """
    if kind not in ALL_KINDS:
        raise ValueError("tip de scenă necunoscut: {} (permise: {})".format(kind, ", ".join(ALL_KINDS)))
    if difficulty not in DIFFICULTIES:
        raise ValueError("dificultate necunoscută: {}".format(difficulty))
    if frames < 1:
        raise ValueError("frames trebuie să fie >= 1")
    if kind == "missing_marker" and aruco is None:
        raise ValueError("missing_marker cere configurația aruco")
    settings = settings or RenderSettings()
    rng = np.random.default_rng(seed)
    p = _PARAMS[difficulty]
    img_w, img_h = settings.image_size

    table = np.array([205, 208, 210], dtype=np.float32) + rng.uniform(-6, 6, size=3).astype(np.float32)
    canvas = _Canvas(grid, rows, cols, settings, table)
    texture = rng.normal(0.0, 3.0, size=(canvas.image.shape[0] // 8 + 1, canvas.image.shape[1] // 8 + 1)).astype(np.float32)
    texture = cv2.resize(cv2.GaussianBlur(texture, (0, 0), 2.0), (canvas.image.shape[1], canvas.image.shape[0]))
    canvas.image += texture[..., None]

    if kind not in ("blank", "no_grid"):
        _draw_grid(canvas)
    if aruco is not None and kind != "blank":
        skip = int(rng.integers(0, 4)) if kind == "missing_marker" else None
        _draw_markers(canvas, aruco, skip)

    camera_height = float(rng.uniform(*p.camera_height_mm))
    grid_center = np.array([cols * grid.cell_mm / 2.0, rows * grid.cell_mm / 2.0])
    nadir = grid_center + rng.uniform(-p.nadir_offset_mm, p.nadir_offset_mm, size=2)
    k = camera_height / (camera_height - grid.cube_mm)

    cubes: List[CubeTruth] = []
    if kind != "blank":
        sampled = _sample_cubes(rng, grid, rows, cols, p, kind)
        ambiguous_index = int(rng.integers(0, len(sampled))) if kind == "ambiguous_color" and sampled else -1
        drawn = []
        for index, (row, col, yaw, center) in enumerate(sampled):
            face = "full" if rng.random() < 0.5 else "icon"
            if index == ambiguous_index:
                variant = "orange" if rng.random() < 0.5 else "desaturated"
                h, s, v = _AMBIGUOUS_HSV[variant]
                hue = int(rng.integers(0, 180)) if h is None else h
                bgr = _hsv_to_bgr(hue, s, v)
                truth_color, note = AMBIGUOUS, variant
            else:
                name = KNOWN_COLORS[int(rng.integers(0, len(KNOWN_COLORS)))].value
                h, s, v = _BASE_HSV[name]
                hue = h + int(rng.integers(-p.hue_jitter, p.hue_jitter + 1))
                bgr = _hsv_to_bgr(hue, s + int(rng.integers(-p.sat_jitter, p.sat_jitter + 1)),
                                  v + int(rng.integers(-p.val_jitter, p.val_jitter + 1)))
                truth_color, note = name, ""
            colored_side = rng.random() < p.colored_side_probability
            icon_shape = int(rng.integers(0, 3))
            drawn.append((row, col, yaw, center, face, bgr, colored_side, icon_shape))
            cubes.append(CubeTruth(row, col, truth_color, round(normalize_yaw_deg(yaw), 3), face, center, note))
        _draw_cubes(canvas, drawn, nadir, k, settings)

    out_scale = float(np.mean(p.grid_side_px)) / (cols * grid.cell_mm)
    sigma = max(0.0, 0.5 * canvas.scale / out_scale)
    if sigma > 0.3:
        canvas.image = cv2.GaussianBlur(canvas.image, (0, 0), sigma)

    # Cu markeri, grila e micșorată ca markerii din afara ei să rămână în cadru.
    image_quad = _camera_quad(rng, p, kind, img_w, img_h, rows, cols, 0.75 if aruco is not None else 1.0)
    grid_mm = np.array([[0, 0], [cols * grid.cell_mm, 0], [cols * grid.cell_mm, rows * grid.cell_mm], [0, rows * grid.cell_mm]])
    homography = cv2.getPerspectiveTransform(canvas.to_px(grid_mm).astype(np.float32), image_quad.astype(np.float32))
    border_value = tuple(float(c) for c in table)
    base = cv2.warpPerspective(canvas.image, homography, (img_w, img_h), flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=border_value)

    # Colțurile grilei = centrele liniilor de contur, ca în detecție.
    outer_px = cv2.perspectiveTransform(canvas.to_px(grid_mm).reshape(-1, 1, 2), homography).reshape(-1, 2)

    if kind == "occluded_corner":
        corner = outer_px[int(rng.integers(0, 4))]
        axes = (int(rng.uniform(30, 45)), int(rng.uniform(20, 30)))
        angle = float(rng.uniform(0, 180))
        skin = _hsv_to_bgr(12, 120, 190)
        cv2.ellipse(base, (int(corner[0]), int(corner[1])), axes, angle, 0, 360, tuple(float(c) for c in skin), -1, cv2.LINE_AA)

    base = _apply_lighting(rng, p, kind, base)
    blur = float(rng.uniform(*p.blur_sigma))
    if blur > 0.3:
        base = cv2.GaussianBlur(base, (0, 0), blur)
    noise_sigma = float(rng.uniform(*p.noise_sigma))
    frames_out = []
    for _ in range(frames):
        noisy = base + rng.normal(0.0, noise_sigma, size=base.shape).astype(np.float32)
        frames_out.append(np.clip(np.rint(noisy), 0, 255).astype(np.uint8))

    expect: Optional[bool]
    if kind == KIND_POSITIVE:
        expect = True
    elif kind in NEGATIVE_KINDS or kind == "missing_marker":
        expect = False
    else:
        expect = None
    has_grid = kind not in ("blank", "no_grid")
    truth = SceneTruth(
        seed=seed,
        kind=kind,
        difficulty=difficulty,
        use_aruco=aruco is not None,
        rows=rows,
        cols=cols,
        cubes=tuple(cubes),
        grid_corners_px=tuple((round(float(x), 3), round(float(y), 3)) for x, y in outer_px) if has_grid else None,
        expect_actionable=expect,
        camera={"height_mm": round(camera_height, 2), "nadir_x_mm": round(float(nadir[0]), 2),
                "nadir_y_mm": round(float(nadir[1]), 2), "noise_sigma": round(noise_sigma, 3), "blur_sigma": round(blur, 3)},
    )
    return SyntheticScene(frames=frames_out, truth=truth)


def _draw_cubes(
    canvas: _Canvas, drawn: list, nadir: np.ndarray, k: float, settings: RenderSettings, ks: Optional[Sequence[float]] = None
) -> None:
    """`k` = factorul de paralaxă al fețelor de sus; `ks` (opțional) îl dă per cub (vârfuri de stive mai înalte)."""
    cube = canvas.grid.cube_mm
    # Umbra se randează la rezoluție redusă (e oricum foarte estompată), apoi se mărește.
    full_h, full_w = canvas.image.shape[:2]
    down = 4
    shadow = np.zeros((full_h // down + 1, full_w // down + 1), dtype=np.float32)
    shifts = 1 << _Canvas.SHIFT
    for _, _, yaw, center, *_rest in drawn:
        base = _square((center[0] + 1.5, center[1] + 2.5), cube * 1.08, yaw)
        pts = np.rint(canvas.to_px(base) / down * shifts).astype(np.int32).reshape(-1, 1, 2)
        cv2.fillPoly(shadow, [pts], 1.0, cv2.LINE_AA, _Canvas.SHIFT)
    shadow = cv2.GaussianBlur(shadow, (0, 0), 2.0 * canvas.scale / down)
    shadow = cv2.resize(shadow, (full_w, full_h), interpolation=cv2.INTER_LINEAR)
    canvas.image *= (1.0 - 0.18 * shadow)[..., None]

    tops = []
    for index, (row, col, yaw, center, face, bgr, colored_side, icon_shape) in enumerate(drawn):
        base = _square(center, cube, yaw)
        top = nadir + (base - nadir) * (ks[index] if ks is not None else k)
        hull = cv2.convexHull(np.vstack([base, top]).astype(np.float32)).reshape(-1, 2)
        side_color = tuple(0.75 * c for c in bgr) if colored_side else (172.0, 174.0, 176.0)
        canvas.fill(hull, side_color)
        tops.append((top, yaw, face, bgr, icon_shape))
    for top, yaw, face, bgr, icon_shape in tops:
        canvas.fill(top, bgr)
        if face == "icon":
            center = top.mean(axis=0)
            side = float(np.linalg.norm(top[1] - top[0]))
            band = settings.border_band_mm * side / cube
            canvas.fill(_square(center, side - 2 * band, yaw), (236.0, 236.0, 236.0))
            ink = (45.0, 45.0, 48.0)
            if icon_shape == 0:
                circle = [(center[0] + 0.18 * side * math.cos(t), center[1] + 0.18 * side * math.sin(t))
                          for t in np.linspace(0, 2 * math.pi, 24, endpoint=False)]
                canvas.fill(np.array(circle), ink)
            elif icon_shape == 1:
                canvas.fill(_square(center, 0.3 * side, yaw + 45.0), ink)
            else:
                a = math.radians(yaw)
                tri = [(center[0] + 0.22 * side * math.cos(a + t), center[1] - 0.22 * side * math.sin(a + t))
                       for t in (math.pi / 2, math.pi / 2 + 2.094, math.pi / 2 + 4.189)]
                canvas.fill(np.array(tri), ink)


def _camera_quad(
    rng: np.random.Generator, p: _Params, kind: str, img_w: int, img_h: int, rows: int, cols: int, shrink: float
) -> np.ndarray:
    side = float(rng.uniform(*p.grid_side_px)) * shrink
    width, height = side * cols / max(rows, cols), side * rows / max(rows, cols)
    theta = math.radians(float(rng.uniform(-p.rotation_deg, p.rotation_deg)))
    center = np.array([img_w / 2.0, img_h / 2.0]) + rng.uniform(-p.shift_px, p.shift_px, size=2)
    if kind == "grid_out_of_frame":
        center[0] = img_w - width * float(rng.uniform(0.15, 0.3))
    c, s = math.cos(theta), math.sin(theta)
    quad = []
    for u, v in ((-width / 2, -height / 2), (width / 2, -height / 2), (width / 2, height / 2), (-width / 2, height / 2)):
        quad.append((center[0] + u * c + v * s, center[1] - u * s + v * c))
    quad_arr = np.array(quad) + rng.uniform(-p.perspective, p.perspective, size=(4, 2)) * side
    if kind == "degenerate":
        # Latura dreaptă e strivită spre un punct: perspectivă imposibilă pentru o observare validă.
        mid = (quad_arr[1] + quad_arr[2]) / 2.0
        quad_arr[1] = mid + (quad_arr[1] - mid) * 0.12
        quad_arr[2] = mid + (quad_arr[2] - mid) * 0.12
    return quad_arr


def _apply_lighting(rng: np.random.Generator, p: _Params, kind: str, image: np.ndarray) -> np.ndarray:
    h, w = image.shape[:2]
    ys, xs = np.mgrid[0:h, 0:w].astype(np.float32)
    gx, gy = rng.uniform(-p.gradient, p.gradient, size=2)
    field = 1.0 + gx * (xs - w / 2.0) / w + gy * (ys - h / 2.0) / h
    cast = rng.uniform(1.0 - p.cast, 1.0 + p.cast, size=3).astype(np.float32)
    exposure = float(rng.uniform(*p.exposure))
    if kind == "overexposed":
        exposure *= 3.0
    elif kind == "underexposed":
        exposure *= 0.12
    return image * field[..., None] * cast.reshape(1, 1, 3) * exposure
