import pytest

from brain.domain.cube import Color, Occupancy, TopFace, normalize_yaw_deg, yaw_difference_deg
from brain.domain.errors import DomainValidationError
from brain.domain.grid import GridCoord, iter_coords
from brain.domain.observation import CellObservation, FrameQualitySummary, GeometrySummary, GridObservation
from brain.domain.scene_state import (
    CellState,
    CubeState,
    DiffKind,
    LocationState,
    SceneState,
    assign_unique_color_ids,
    diff_locations,
    diff_scenes,
    location_from_observation,
)


def test_grid_coord_validation():
    assert GridCoord(2, 1).as_tuple() == (2, 1)
    for bad in [(-1, 0), (0, -2), (1.0, 0), (True, 0)]:
        with pytest.raises(DomainValidationError):
            GridCoord(*bad)


def test_iter_coords_row_major_3x3():
    coords = list(iter_coords(3, 3))
    assert len(coords) == 9
    assert coords[0] == GridCoord(0, 0) and coords[3] == GridCoord(1, 0) and coords[-1] == GridCoord(2, 2)


@pytest.mark.parametrize(
    "angle, expected",
    [(0, 0), (44.9, 44.9), (45, -45), (-45, -45), (90, 0), (100, 10), (-100, -10), (135, -45), (359, -1)],
)
def test_normalize_yaw(angle, expected):
    assert normalize_yaw_deg(angle) == pytest.approx(expected)


def test_yaw_difference_respects_90_degree_symmetry():
    assert yaw_difference_deg(44.0, -44.0) == pytest.approx(2.0)
    assert yaw_difference_deg(10.0, 5.0) == pytest.approx(5.0)
    assert 0.0 <= yaw_difference_deg(30.0, -30.0) <= 45.0


def test_cell_observation_rejects_inconsistent_values():
    with pytest.raises(DomainValidationError):
        CellObservation(GridCoord(0, 0), Occupancy.EMPTY, 0.9, color=Color.RED, color_confidence=0.9)
    with pytest.raises(DomainValidationError):
        CellObservation(GridCoord(0, 0), Occupancy.OCCUPIED, 1.2)
    with pytest.raises(DomainValidationError):
        CellObservation(GridCoord(0, 0), Occupancy.OCCUPIED, 0.9, yaw_deg=45.0, yaw_confidence=0.5)
    with pytest.raises(DomainValidationError):
        CellObservation(GridCoord(0, 0), Occupancy.OCCUPIED, 0.9, yaw_confidence=0.5)


def _observation(cells, actionable=True, reasons=()):
    return GridObservation(
        location="source", rows=3, cols=3, captured_at="2026-10-29T10:00:00Z", actionable=actionable,
        cells=tuple(cells), reasons=tuple(reasons), geometry=GeometrySummary("lines", actionable),
        quality=FrameQualitySummary(5, 5),
    )


def _cells(occupied=None):
    occupied = occupied or {}
    cells = []
    for coord in iter_coords(3, 3):
        if coord in occupied:
            color, yaw = occupied[coord]
            cells.append(CellObservation(coord, Occupancy.OCCUPIED, 0.95, color, 0.9, yaw, 0.8 if yaw is not None else 0.0, TopFace.FULL))
        else:
            cells.append(CellObservation(coord, Occupancy.EMPTY, 0.98))
    return cells


def test_grid_observation_invariants():
    obs = _observation(_cells({GridCoord(1, 1): (Color.RED, 10.0)}))
    assert obs.cell(GridCoord(1, 1)).color is Color.RED
    with pytest.raises(DomainValidationError):
        _observation(_cells()[:8])
    with pytest.raises(DomainValidationError):
        _observation(_cells(), actionable=False, reasons=("grid_not_found",))
    with pytest.raises(DomainValidationError):
        _observation([], actionable=False, reasons=())
    rejected = _observation([], actionable=False, reasons=("grid_not_found",))
    with pytest.raises(DomainValidationError):
        rejected.cell(GridCoord(0, 0))


def test_cell_state_stack_rules():
    cube = CubeState(Color.BLUE, 0.9)
    with pytest.raises(DomainValidationError):
        CellState(GridCoord(0, 0), Occupancy.EMPTY, 1.0, stack=(cube,))
    with pytest.raises(DomainValidationError):
        CellState(GridCoord(0, 0), Occupancy.OCCUPIED, 1.0, stack=())
    with pytest.raises(DomainValidationError):
        CellState(GridCoord(0, 0), Occupancy.OCCUPIED, 1.0, stack=(cube,) * 10)  # serviciul acceptă nivelurile 0..8
    tower = CellState(GridCoord(0, 0), Occupancy.OCCUPIED, 1.0, stack=(cube,) * 5)  # turnul S2 are 5 cuburi
    assert tower.top is cube


def test_location_and_scene_updates():
    location = LocationState.empty("dest", 3, 3)
    cube = CubeState(Color.GREEN, 0.9)
    updated = location.with_cell(CellState(GridCoord(2, 0), Occupancy.OCCUPIED, 0.9, (cube,)))
    assert updated.cell(GridCoord(2, 0)).top is cube
    assert location.cell(GridCoord(2, 0)).occupancy is Occupancy.EMPTY
    scene = SceneState.new("s1").with_location(updated)
    assert scene.location("dest") is updated
    with pytest.raises(DomainValidationError):
        scene.location("missing")
    with pytest.raises(DomainValidationError):
        LocationState("x", 3, 3, cells=location.cells[:8])


def test_location_from_observation_and_unique_ids():
    obs = _observation(_cells({GridCoord(0, 0): (Color.RED, 5.0), GridCoord(0, 1): (Color.BLUE, None),
                               GridCoord(0, 2): (Color.BLUE, -3.0)}))
    scene = assign_unique_color_ids(SceneState.new("s").with_location(location_from_observation(obs)))
    loc = scene.location("source")
    assert loc.cell(GridCoord(0, 0)).top.cube_id == "red"
    assert loc.cell(GridCoord(0, 1)).top.cube_id is None  # două cuburi albastre: interschimbabile
    assert loc.cell(GridCoord(0, 1)).top.yaw_deg is None
    with pytest.raises(DomainValidationError):
        location_from_observation(_observation([], actionable=False, reasons=("grid_not_found",)))


def test_diff_locations_reports_each_kind():
    expected = location_from_observation(_observation(_cells({GridCoord(0, 0): (Color.RED, 0.0),
                                                              GridCoord(1, 1): (Color.BLUE, 10.0),
                                                              GridCoord(2, 2): (Color.GREEN, 0.0)})))
    observed_cells = _cells({GridCoord(0, 0): (Color.YELLOW, 0.0), GridCoord(1, 1): (Color.BLUE, 30.0)})
    observed_cells[0 * 3 + 1] = CellObservation(GridCoord(0, 1), Occupancy.UNKNOWN, 0.0, reasons=("edges_without_color",))
    observed = location_from_observation(_observation(observed_cells))
    diffs = diff_locations(expected, observed, yaw_tolerance_deg=5.0)
    kinds = {(d.coord.as_tuple(), d.kind) for d in diffs}
    assert ((0, 0), DiffKind.COLOR) in kinds
    assert ((0, 1), DiffKind.UNCERTAIN) in kinds
    assert ((1, 1), DiffKind.YAW) in kinds
    assert ((2, 2), DiffKind.OCCUPANCY) in kinds
    assert diff_locations(expected, expected) == []


def test_diff_scenes_missing_location():
    location = LocationState.empty("dest", 3, 3)
    expected = SceneState.new("a").with_location(location)
    diffs = diff_scenes(expected, SceneState.new("b"))
    assert diffs[0].kind is DiffKind.MISSING_LOCATION
