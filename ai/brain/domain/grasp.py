"""Geometria prinderii: amprenta cuburilor pe masă, spațiile libere dintre ele și deschiderea ghearei.

Regulile cerute de echipă (2026-10-29):
A. gheara se pre-închide la deschiderea de abordare ÎNAINTE de deplasare (nu călătorește larg deschisă);
B. coborârea: pe centrul cubului dacă spațiul permite; altfel centrul ghearei se mută spre spațiul mai mare,
   iar degetul dinspre spațiul strâmt intră între cub și vecin cu `clearance_mm` față de vecin. Gheara e
   simetrică: la strângere, degetul din partea strâmtă (cel mai aproape de cub) îl atinge primul și îl împinge
   spre spațiul liber, adică departe de vecin;
C. eliberarea: deschidere mică, doar cât să elibereze cubul; ridicarea strict verticală o face serviciul.

Coordonate: mm pe planul mesei, în sistemul caroiajului (u = coloane, spre dreapta; v = rânduri, în jos în
imaginea redresată). Axele unui cub cu yaw ψ (antiorar pozitiv, cu axa v în jos): a = (cos ψ, -sin ψ),
b = (sin ψ, cos ψ). Gheara se închide pe una dintre axele cubului, iar care anume decide serviciul (încheietura
se aliniază cu cubul în ±45° față de braț). De aceea deschiderea aleasă trebuie să încapă pe AMBELE axe, iar
offset-ul se calculează pe fiecare axă: pe axa pe care gheara nu se închide, el doar alunecă degetele de-a
lungul fețelor cubului.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

Point = Tuple[float, float]

AXES = ("a", "b")
# Cheile din `CellObservation.metrics` în care viziunea pune spațiile libere măsurate (mm).
GAP_METRIC_KEYS = (("a-", "gap_a_neg_mm"), ("a+", "gap_a_pos_mm"), ("b-", "gap_b_neg_mm"), ("b+", "gap_b_pos_mm"))


@dataclass(frozen=True)
class GripperGeometry:
    """Gheara (valori date de echipă, nemăsurate de brain). Toate deschiderile sunt INTERIOARE (între degete)."""

    max_open_mm: float  # deschiderea maximă
    finger_mm: float  # grosimea unui deget; lățimea exterioară = deschiderea + 2 · finger_mm
    clearance_mm: float  # spațiul minim păstrat între un deget și un cub vecin
    approach_margin_mm: float  # dorit la abordare: cât rămâne între deget și cub, pe fiecare latură
    min_approach_margin_mm: float  # minim la abordare: sub atât degetul ar atinge cubul la coborâre
    release_margin_mm: float  # la eliberare: deschiderea = cubul + 2 · release_margin_mm
    min_release_margin_mm: float  # minim la eliberare: sub atât cubul nu e eliberat sigur
    # La zone și puncte tampon (fără vecini, nevăzute de cameră) cubul poate sta decalat: abordare mai largă.
    point_approach_margin_mm: float = 10.0

    @property
    def side_space_mm(self) -> float:
        """Spațiul liber minim de care are nevoie un deget lângă cub (deget + distanța față de vecin + cea față de cub)."""
        return self.finger_mm + self.clearance_mm + self.min_approach_margin_mm


@dataclass(frozen=True)
class Footprint:
    """Baza unui cub pe masă: pătrat cu centrul în mm (sistemul caroiajului) și rotit cu yaw."""

    center: Point
    yaw_deg: float
    side_mm: float

    def axes(self) -> Tuple[Point, Point]:
        psi = math.radians(self.yaw_deg)
        return (math.cos(psi), -math.sin(psi)), (math.sin(psi), math.cos(psi))

    def corners(self) -> List[Point]:
        (ax, ay), (bx, by) = self.axes()
        h = self.side_mm / 2.0
        cx, cy = self.center
        return [(cx + sa * h * ax + sb * h * bx, cy + sa * h * ay + sb * h * by)
                for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1))]


def footprint_from_uv(uv: Point, yaw_deg: Optional[float], cell_mm: float, side_mm: float) -> Footprint:
    """`uv` = centrul bazei în celule (0..3); fără yaw măsurat, cubul e presupus aliniat cu caroiajul."""
    return Footprint((uv[0] * cell_mm, uv[1] * cell_mm), float(yaw_deg or 0.0), side_mm)


@dataclass(frozen=True)
class Clearances:
    """Spațiul liber (mm) de la fiecare față a cubului până la cel mai apropiat vecin, pe axele cubului.

    None = nimic pe direcția aceea (liber). 0 = vecinul atinge sau se suprapune cu cubul.
    """

    a_neg: Optional[float] = None
    a_pos: Optional[float] = None
    b_neg: Optional[float] = None
    b_pos: Optional[float] = None

    def side(self, axis: str, sign: int) -> Optional[float]:
        return getattr(self, "{}_{}".format(axis, "pos" if sign > 0 else "neg"))

    def minimum(self) -> Optional[float]:
        values = [v for v in (self.a_neg, self.a_pos, self.b_neg, self.b_pos) if v is not None]
        return min(values) if values else None

    def to_dict(self) -> Dict[str, Optional[float]]:
        return {key: None if value is None else round(value, 1) for key, value in (
            ("a-", self.a_neg), ("a+", self.a_pos), ("b-", self.b_neg), ("b+", self.b_pos))}


def clearances_from_metrics(metrics: Dict[str, float]) -> Clearances:
    """Spațiile măsurate de viziune (`CellObservation.metrics`); cheile lipsă = liber."""
    values = {side: metrics.get(key) for side, key in GAP_METRIC_KEYS}
    return Clearances(values["a-"], values["a+"], values["b-"], values["b+"])


def _clip_band(polygon: List[Point], half_width: float) -> List[Point]:
    """Partea unui poligon convex (coordonate locale x, y) aflată în banda |y| <= half_width."""
    for sign in (1.0, -1.0):  # întâi y <= h, apoi -y <= h
        clipped: List[Point] = []
        count = len(polygon)
        for i in range(count):
            p, q = polygon[i], polygon[(i + 1) % count]
            p_in, q_in = sign * p[1] <= half_width, sign * q[1] <= half_width
            if p_in:
                clipped.append(p)
            if p_in != q_in:
                t = (sign * half_width - p[1]) / (q[1] - p[1])
                clipped.append((p[0] + t * (q[0] - p[0]), sign * half_width))
        polygon = clipped
        if not polygon:
            break
    return polygon


def clearances(target: Footprint, others: Sequence[Footprint], band_mm: Optional[float] = None) -> Clearances:
    """Spațiul liber pe fiecare latură a cubului țintă.

    Un deget coboară lângă o față într-o bandă cât fața (presupunere: degetul nu e mai lat decât cubul,
    `band_mm` o poate lărgi). Pe fiecare direcție contează cel mai apropiat vecin care intră în bandă,
    inclusiv vecinii pe diagonală care se rotesc sau sunt deplasați spre ea.
    """
    half = (band_mm if band_mm is not None else target.side_mm) / 2.0
    s2 = target.side_mm / 2.0
    axes = target.axes()
    result: Dict[str, Optional[float]] = {"a_neg": None, "a_pos": None, "b_neg": None, "b_pos": None}
    for index, axis in enumerate(AXES):
        along, across = axes[index], axes[1 - index]
        for other in others:
            local = [((x - target.center[0]) * along[0] + (y - target.center[1]) * along[1],
                      (x - target.center[0]) * across[0] + (y - target.center[1]) * across[1])
                     for x, y in other.corners()]
            inside = _clip_band(local, half)
            if not inside:
                continue
            xs = [p[0] for p in inside]
            for sign, key in ((1, "{}_pos".format(axis)), (-1, "{}_neg".format(axis))):
                far = max(sign * x for x in xs)
                if far <= s2:  # nu trece de fața cubului pe partea asta: nu e în calea degetului
                    continue
                gap = max(0.0, min(sign * x for x in xs) - s2)
                current = result[key]
                result[key] = gap if current is None else min(current, gap)
    return Clearances(**result)


# ---------------------------------------------------------------------- abordarea (regulile A și B)
@dataclass(frozen=True)
class AxisGrasp:
    """Prinderea pe o axă a cubului: cea mai mare deschidere care încape și offset-ul pentru o deschidere dată."""

    axis: str
    gap_neg: Optional[float]
    gap_pos: Optional[float]
    feasible: bool
    max_open_mm: Optional[float]
    reason: str = ""


@dataclass(frozen=True)
class GraspPlan:
    open_mm: float  # deschiderea de abordare (interioară), cu care gheara călătorește și coboară
    offset_mm: Tuple[float, float]  # deplasarea centrului ghearei față de centrul cubului, pe axele (a, b)
    offset_uv: Tuple[float, float]  # aceeași deplasare în celule (u, v), adunată la uv-ul trimis serviciului
    feasible: bool  # degetele încap pe ambele axe
    centered: bool  # coborâre pe centrul cubului (fără offset)
    feasible_axes: Tuple[str, ...]
    clearances: Clearances
    notes: Tuple[str, ...] = ()

    @property
    def status(self) -> str:
        """`centered` / `offset` (degetele încap, cu sau fără offset), `risky` (încap pe o singură axă), `blocked`."""
        if self.feasible:
            return "centered" if self.centered else "offset"
        return "risky" if self.feasible_axes else "blocked"

    def describe(self) -> str:
        text = "deschidere {:.0f} mm".format(self.open_mm)
        if not self.centered:
            text += ", offset {:+.1f}/{:+.1f} mm".format(*self.offset_mm)
        if not self.feasible:
            text += " (RISC: " + "; ".join(self.notes) + ")"
        return text


def _axis(gripper: GripperGeometry, side: float, axis: str, gap_neg: Optional[float], gap_pos: Optional[float]) -> AxisGrasp:
    need = gripper.side_space_mm
    narrow = [(label, gap) for label, gap in (("-", gap_neg), ("+", gap_pos)) if gap is not None and gap < need]
    if narrow:
        text = ", ".join("{}{}: {:.1f} mm".format(axis, label, gap) for label, gap in narrow)
        return AxisGrasp(axis, gap_neg, gap_pos, False, None,
                         "pe axa {} degetul nu încape ({} < {:.1f} mm)".format(axis, text, need))
    room = 2 * gripper.finger_mm + 2 * gripper.clearance_mm
    total = (gap_neg if gap_neg is not None else math.inf) + (gap_pos if gap_pos is not None else math.inf)
    largest = min(gripper.max_open_mm, side + total - room)
    return AxisGrasp(axis, gap_neg, gap_pos, True, largest)


def _offset(gripper: GripperGeometry, side: float, opening: float, gap_neg: Optional[float], gap_pos: Optional[float]) -> float:
    """Offset-ul cel mai apropiat de 0 pentru care degetele nu ating nici cubul, nici vecinii (sau cel mai bun compromis)."""
    half_open = opening / 2.0
    s2 = side / 2.0
    m, t, c = gripper.min_approach_margin_mm, gripper.finger_mm, gripper.clearance_mm
    inner_low, inner_high = s2 + m - half_open, half_open - s2 - m  # degetele rămân în afara cubului
    low, high = inner_low, inner_high
    if gap_neg is not None:
        low = max(low, -s2 - gap_neg + c + t + half_open)
    if gap_pos is not None:
        high = min(high, s2 + gap_pos - c - t - half_open)
    if low <= high:
        return min(max(0.0, low), high)
    # Nu încape: aceeași lipsă de spațiu pe ambele părți, dar gheara trebuie să înconjoare cubul în continuare.
    return min(max((low + high) / 2.0, inner_low), inner_high)


def plan_grasp(
    target: Footprint,
    gaps: Clearances,
    gripper: GripperGeometry,
    cell_mm: float,
) -> GraspPlan:
    """Deschiderea de abordare și offset-ul coborârii, din spațiile libere măsurate."""
    side = target.side_mm
    per_axis = [_axis(gripper, side, axis, gaps.side(axis, -1), gaps.side(axis, 1)) for axis in AXES]
    minimum = side + 2 * gripper.min_approach_margin_mm
    wanted = min(side + 2 * gripper.approach_margin_mm, gripper.max_open_mm)
    limits = [g.max_open_mm for g in per_axis if g.feasible and g.max_open_mm is not None]
    opening = max(minimum, min([wanted] + limits))
    notes: List[str] = [g.reason for g in per_axis if not g.feasible]
    if opening > gripper.max_open_mm:
        notes.append("cubul ({:.0f} mm) nu încape în gheară ({:.0f} mm)".format(side, gripper.max_open_mm))
        opening = gripper.max_open_mm
    offsets = tuple(round(_offset(gripper, side, opening, g.gap_neg, g.gap_pos), 2) for g in per_axis)
    (ax, ay), (bx, by) = target.axes()
    du = (offsets[0] * ax + offsets[1] * bx) / cell_mm
    dv = (offsets[0] * ay + offsets[1] * by) / cell_mm
    feasible_axes = tuple(g.axis for g in per_axis if g.feasible)
    return GraspPlan(
        open_mm=round(opening, 1),
        offset_mm=(offsets[0], offsets[1]),
        offset_uv=(round(du, 4), round(dv, 4)),
        feasible=len(feasible_axes) == len(AXES) and opening <= gripper.max_open_mm and not notes,
        centered=offsets == (0.0, 0.0),
        feasible_axes=feasible_axes,
        clearances=gaps,
        notes=tuple(notes),
    )


# ---------------------------------------------------------------------- degetele (verificare în simulare)
def finger_polygons(center: Point, yaw_deg: float, axis: str, opening_mm: float, finger_mm: float,
                    width_mm: float) -> List[List[Point]]:
    """Cele două degete (dreptunghiuri) văzute de sus, cu gheara centrată în `center` și închisă pe `axis`."""
    footprint = Footprint(center, yaw_deg, 1.0)
    along, across = footprint.axes() if axis == "a" else footprint.axes()[::-1]
    polygons = []
    for sign in (-1.0, 1.0):
        inner, outer = opening_mm / 2.0, opening_mm / 2.0 + finger_mm
        polygon = []
        for depth, lateral in ((inner, -width_mm / 2.0), (outer, -width_mm / 2.0), (outer, width_mm / 2.0),
                               (inner, width_mm / 2.0)):
            polygon.append((center[0] + sign * depth * along[0] + lateral * across[0],
                            center[1] + sign * depth * along[1] + lateral * across[1]))
        polygons.append(polygon)
    return polygons


def convex_overlap(first: Sequence[Point], second: Sequence[Point], tolerance_mm: float = 0.0) -> bool:
    """Două poligoane convexe se suprapun (teorema axei separatoare); `tolerance_mm` iartă atingerile."""
    for polygon in (first, second):
        count = len(polygon)
        for i in range(count):
            (x0, y0), (x1, y1) = polygon[i], polygon[(i + 1) % count]
            nx, ny = y0 - y1, x1 - x0
            norm = math.hypot(nx, ny)
            if norm == 0.0:
                continue
            nx, ny = nx / norm, ny / norm
            a = [px * nx + py * ny for px, py in first]
            b = [px * nx + py * ny for px, py in second]
            if max(a) <= min(b) + tolerance_mm or max(b) <= min(a) + tolerance_mm:
                return False
    return True


# ---------------------------------------------------------------------- eliberarea (regula C)
@dataclass(frozen=True)
class ReleasePlan:
    release_mm: float  # deschiderea la eliberare (interioară)
    feasible: bool  # degetele (închise pe cub, apoi deschise puțin) nu ating vecinii
    clearances: Clearances
    notes: Tuple[str, ...] = ()


def plan_release(gaps: Clearances, gripper: GripperGeometry, side: float) -> ReleasePlan:
    """Deschiderea mică la eliberare: cât permit vecinii, între `min_release_margin_mm` și `release_margin_mm`.

    Cubul ținut coboară centrat pe slot; degetele (strânse pe cub) ocupă `finger_mm` lângă fiecare față, iar la
    eliberare se depărtează cu marja aleasă. Gheara se închide pe o axă necunoscută, deci contează toate laturile.
    """
    room = gripper.finger_mm + gripper.clearance_mm
    known = {key: gap for key, gap in gaps.to_dict().items() if gap is not None}
    hits = ["{} {:.1f} mm".format(key, gap) for key, gap in known.items() if gap < room]
    spare = min([gap - room for gap in known.values()] + [gripper.release_margin_mm])
    margin = min(gripper.release_margin_mm, max(spare, gripper.min_release_margin_mm))
    opening = min(side + 2 * margin, gripper.max_open_mm)
    notes: List[str] = []
    if hits:
        notes.append("degetul atinge vecinul la coborâre ({} < {:.1f} mm)".format(", ".join(hits), room))
    elif spare < gripper.min_release_margin_mm:
        notes.append("eliberare strâmtă: deschiderea minimă {:.0f} mm atinge vecinul".format(opening))
    return ReleasePlan(round(opening, 1), not notes, gaps, tuple(notes))
