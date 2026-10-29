"""SimWorld, SimulatedProximity, ManualClock, correct_parallax și validarea argumentelor brațului."""

import dataclasses

import pytest

from brain.adapters.arm import validation as v
from brain.adapters.clock import ManualClock
from brain.adapters.proximity.simulated import ProximityFaults, SimulatedProximity
from brain.domain.errors import DomainValidationError, InvalidCommand
from brain.domain.plan import correct_parallax
from brain.domain.proximity import Validity
from brain.simulation.sim_world import (
    COLLISION, DROPPED, GRASPED, MISSED, NOTHING_HELD, PLACED, UNREACHABLE, Hover, SimCube, SimGrid, SimPhysics, SimWorld,
    SimZone,
)


def _world(**physics):
    grid = SimGrid("medium", 3, 3, 43.0)
    world = SimWorld([grid], [SimZone("zone_red")], 30.0, SimPhysics(lost_probability=0.0, **physics), seed=1)
    return world, grid


def _cube(world, color="red", offset=(0.0, 0.0), yaw=0.0):
    return world.new_cube(color, "full", yaw, offset)


# ---------------------------------------------------------------------- SimWorld
def test_grasp_within_and_outside_tolerance():
    world, grid = _world()
    grid.stack((0, 0)).append(_cube(world, offset=(5.0, 0.0)))
    assert world.pick("medium", (0, 0), 0, 0.0, None) == GRASPED  # 5 mm de centru < 7 mm
    world.held = None
    grid.stack((0, 1)).append(_cube(world, offset=(9.0, 0.0)))
    assert world.pick("medium", (0, 1), 0, 0.0, None) == MISSED  # 9 mm > toleranță
    assert world.pick("medium", (0, 1), 0, 0.0, (1.5 + 9.0 / 43.0 + 0.04, 0.5)) == GRASPED  # uv corect compensează


def test_grasp_needs_aligned_yaw():
    world, grid = _world()
    grid.stack((1, 1)).append(_cube(world, yaw=30.0))
    assert world.pick("medium", (1, 1), 0, 0.0, None) == MISSED  # 30° > 20°
    assert world.pick("medium", (1, 1), 0, 28.0, None) == GRASPED


def test_wrong_level_misses_or_collides():
    world, grid = _world()
    grid.stack((0, 0)).extend([_cube(world), _cube(world, "blue")])
    assert world.pick("medium", (0, 0), 2, 0.0, None) == MISSED  # peste vârf: gheara se închide în gol
    assert world.pick("medium", (0, 0), 0, 0.0, None) == COLLISION  # sub vârf: lovește stiva
    assert len(grid.stack((0, 0))) == 1 and any("coliziune" in e for e in world.events)


def test_stacking_and_falling():
    world, grid = _world(place_noise_mm=0.0, place_yaw_noise_deg=0.0)
    grid.stack((0, 0)).append(_cube(world))
    grid.stack((2, 2)).append(_cube(world, "blue"))
    assert world.place("medium", (1, 1), 0, 0.0) == NOTHING_HELD
    assert world.pick("medium", (0, 0), 0, 0.0, None) == GRASPED
    assert world.place("medium", (2, 2), 0, 0.0) == COLLISION  # nivel 0 pe o celulă ocupată
    assert world.pick("medium", (2, 2), 0, 0.0, None) == GRASPED
    assert world.place("medium", (1, 1), 0, 0.0) == PLACED
    # Al doilea cub, prins descentrat cu 6 mm și pus deasupra cu încă 6 mm decalaj → peste suportul de 10 mm.
    grid.stack((0, 2)).append(_cube(world, "red", offset=(6.0, 0.0)))
    assert world.pick("medium", (0, 2), 0, 0.0, None) == GRASPED
    grid.stack((1, 1))[-1].offset_mm = (-6.0, 0.0)
    assert world.place("medium", (1, 1), 1, 0.0) == DROPPED
    assert len(grid.stack((1, 1))) == 1


def test_zone_stack_and_unreachable():
    world, grid = _world()
    grid.stack((0, 0)).append(_cube(world))
    grid.unreachable.add((2, 2))
    assert world.pick("medium", (2, 2), 0, 0.0, None) == UNREACHABLE
    assert world.pick("medium", (0, 0), 0, 0.0, None) == GRASPED
    assert world.place("zone_red", None, 0, 0.0) == PLACED
    assert [c.color for c in world.zones["zone_red"].stack] == ["red"]
    world.zones["zone_red"].unreachable = True
    assert world.place("zone_red", None, 1, 0.0) == UNREACHABLE


def test_drop_in_transit_lands_on_free_cell_or_is_lost():
    world, grid = _world()
    grid.stack((0, 0)).append(_cube(world))
    world.pick("medium", (0, 0), 0, 0.0, None)
    world.drop_held_in_transit()
    assert world.held is None and sum(len(s) for s in grid.stacks.values()) == 1
    lossy = SimWorld([SimGrid("medium", 3, 3, 43.0)], [], 30.0, SimPhysics(lost_probability=1.0))
    lossy.grids["medium"].stack((0, 0)).append(lossy.new_cube("red", "full", 0.0))
    lossy.pick("medium", (0, 0), 0, 0.0, None)
    lossy.drop_held_in_transit()
    assert len(lossy.lost) == 1


# ---------------------------------------------------------------------- SimulatedProximity
def _sensor(app_config, world, faults=None, s=25.0):
    config = dataclasses.replace(app_config.system.proximity, enabled=True)
    return SimulatedProximity(world, config, ManualClock(), sensor_above_jaw_mm=s, noise_mm=0.0, faults=faults, seed=3)


def test_proximity_held_cube_below_and_nothing(app_config):
    world, grid = _world()
    sensor = _sensor(app_config, world)
    assert sensor.read().value_mm == pytest.approx(250.0, abs=3)  # poza de observare: masa e departe
    grid.stack((0, 0)).append(_cube(world))
    world.pick("medium", (0, 0), 0, 0.0, None)
    assert sensor.read().validity is Validity.BELOW_RANGE  # cub ținut chiar sub senzor
    world.place("medium", (0, 0), 0, 0.0)
    assert sensor.read().value_mm == pytest.approx(30.0 + 25.0, abs=3)  # cub sub gheară: ≈ 30 + s
    world.hover = Hover("medium", (1, 1), 0)
    assert sensor.read().value_mm == pytest.approx(60.0 + 25.0, abs=3)  # nimic: ≈ 60 + s


def test_proximity_injected_faults(app_config):
    world, _ = _world()
    sensor = _sensor(app_config, world, ProximityFaults(timeout_probability=1.0))
    reading = sensor.read()
    assert reading.validity is Validity.TIMEOUT and reading.value_mm is None
    disconnected = _sensor(app_config, world, ProximityFaults(disconnect_after=2))
    assert [disconnected.read().validity for _ in range(4)][-1] is Validity.DISCONNECTED
    invalid = _sensor(app_config, world, ProximityFaults(invalid_probability=1.0))
    assert invalid.read().validity is Validity.INVALID_DATA
    assert sensor.spec().confirmed is False


# ---------------------------------------------------------------------- ceas, paralaxă, validare
def test_manual_clock():
    clock = ManualClock(5.0)
    clock.sleep(1.5)
    clock.advance(0.5)
    assert clock.monotonic() == pytest.approx(7.0)
    with pytest.raises(ValueError):
        clock.sleep(-1)


def test_correct_parallax_projects_top_face_to_table():
    # Camera la 250 mm, fața la 30 mm: punctul văzut la 1,0 celule de nadir e de fapt la 1,0·220/250.
    u, v = correct_parallax((2.5, 1.5), 30.0, 250.0, (1.5, 1.5))
    assert (u, v) == pytest.approx((1.5 + 220.0 / 250.0, 1.5))
    assert correct_parallax((1.5, 1.5), 90.0, 250.0, (1.5, 1.5)) == pytest.approx((1.5, 1.5))  # nadirul nu se mișcă
    with pytest.raises(DomainValidationError):
        correct_parallax((1.0, 1.0), 260.0, 250.0, (1.5, 1.5))


@pytest.mark.parametrize("call", [
    lambda: v.check_speed("turbo"),
    lambda: v.check_angles([0, 0, 0, 0, True]),
    lambda: v.check_angles("90"),
    lambda: v.check_time_ms(1.5),
    lambda: v.check_cell((0, 3)),
    lambda: v.check_cell([True, 0]),
    lambda: v.check_level(9),
    lambda: v.check_yaw(float("inf")),
    lambda: v.check_uv((1.0, 1.0), None),
    lambda: v.check_uv((1.0, -0.1), (0, 0)),
    lambda: v.check_location(None),
    lambda: v.check_beep(11, 100),
    lambda: v.check_beep(1, 0),
])
def test_validation_rejects(call):
    with pytest.raises(InvalidCommand):
        call()


def test_validation_accepts_and_normalizes():
    assert v.check_angles((90, 45.5, 0, 180, 90)) == [90.0, 45.5, 0.0, 180.0, 90.0]
    assert v.check_cell([2, 1]) == (2, 1)
    assert v.check_uv([0, 3], (0, 0)) == (0.0, 3.0)
    assert v.check_cell(None) is None and v.check_uv(None, None) is None
    assert v.check_beep(10, 5000) == (10, 5000)
