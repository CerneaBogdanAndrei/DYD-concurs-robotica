"""Contractul ArmInterface: aceleași reguli pentru MockArm și TeamArm (cu un server HTTP fals care imită serviciul)."""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from brain.adapters.arm.mock_arm import FaultSpec, MockArm
from brain.adapters.arm.team_arm import TeamArm, to_service_yaw
from brain.adapters.clock import ManualClock
from brain.domain.errors import (
    ArmCommunicationError,
    ArmTimeout,
    CalibrationMissing,
    CapabilityUnavailable,
    InvalidCommand,
    MotionAborted,
)
from brain.ports.arm import MODE_HARDWARE, MODE_SIMULATION
from brain.simulation.sim_world import SimCube, SimGrid, SimWorld, SimZone


class FakeService:
    """Serviciul de mișcare, imitat: răspunsuri configurabile per rută, cererile primite sunt înregistrate."""

    def __init__(self):
        self.requests = []
        self.responses = {}  # rută -> (status HTTP, corp JSON) sau callable(body) -> (status, corp)
        self.delay_s = 0.0
        self.status = {"sim": False, "poses": ["home", "observe_medium"], "sites": {"medium": "grid", "zone_red": "point"},
                       "joints": [90, 90, 90, 90, 90], "grip": 60, "xyz": None}
        fake = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def _reply(self, status, payload):
                data = json.dumps(payload).encode("utf-8")
                self.send_response(status)
                self.send_header("content-type", "application/json")
                self.send_header("content-length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_POST(self):
                length = int(self.headers.get("content-length") or 0)
                body = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
                route = self.path.strip("/")
                fake.requests.append((route, body))
                if fake.delay_s:
                    time.sleep(fake.delay_s)
                if route == "status" and route not in fake.responses:
                    return self._reply(200, {"result": fake.status})
                response = fake.responses.get(route, (200, {"result": None}))
                if callable(response):
                    response = response(body)
                self._reply(*response)

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True)

    @property
    def url(self):
        return "http://127.0.0.1:{}".format(self.server.server_address[1])

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self.server.shutdown()
        self.server.server_close()

    def bodies(self, route):
        return [body for name, body in self.requests if name == route]


@pytest.fixture
def service():
    with FakeService() as fake:
        yield fake


def _mock_arm(**kwargs):
    grid = SimGrid("medium", 3, 3, 43.0)
    grid.stack((1, 1)).append(SimCube(1, "red", "full", 0.0))
    world = SimWorld([grid], [SimZone("zone_red")], 30.0)
    return MockArm(world, ["observe_medium"], ManualClock(), **kwargs)


@pytest.fixture(params=["mock", "team"])
def arm(request, service):
    """Același contract, rulat pe ambele adaptoare (serviciul fals răspunde ca serviciul real, fără erori)."""
    if request.param == "mock":
        return _mock_arm()
    service.responses["pick"] = (200, {"result": True})
    service.responses["place"] = (200, {"result": True})
    service.responses["read_joints"] = (200, {"result": [90, 90, 90, 90, 90, 60]})
    team = TeamArm(service.url, timeout_s=5)
    team.connect()
    return team


# ---------------------------------------------------------------------- contract comun
@pytest.mark.parametrize("call", [
    lambda a: a.pick("medium", (0, 0), level=9),
    lambda a: a.pick("medium", (0, 0), uv=(3.5, 1.0)),
    lambda a: a.pick("medium", (3, 0)),
    lambda a: a.pick("zone_red", None, uv=(1.0, 1.0)),
    lambda a: a.place("medium", (0, 0), level=-1),
    lambda a: a.move_joints([90, 90, 90, 90, 200], 500),
    lambda a: a.move_joints([90, 90, 90], 500),
    lambda a: a.move_joints([90, 90, 90, 90, 90], 20000),
    lambda a: a.go_pose("home", "turbo"),
    lambda a: a.beep(0),
    lambda a: a.pick("", (0, 0)),
    lambda a: a.pick("medium", (0, 0), yaw_deg=float("nan")),
])
def test_invalid_arguments_rejected_before_sending(arm, service, call):
    before = len(service.requests)
    with pytest.raises(InvalidCommand):
        call(arm)
    assert len(service.requests) == before  # nimic trimis serviciului


def test_basic_calls_and_readback(arm):
    arm.home()
    arm.go_pose("observe_medium")
    arm.open_gripper()
    arm.close_gripper()
    arm.beep(2)
    joints = arm.read_joints()
    assert len(joints) == 6 and all(isinstance(v, float) for v in joints)
    assert arm.position_uncertain is False


def test_pick_and_place_return_true(arm):
    assert arm.pick("medium", (1, 1)) is True
    assert arm.place("medium", (0, 0)) is True
    assert arm.last_reason is None


def test_partial_open_and_beep_duration_not_faked(arm):
    if isinstance(arm, MockArm):
        pytest.skip("MockArm simulează ambele")
    with pytest.raises(CapabilityUnavailable):
        arm.open_gripper(full=False)
    with pytest.raises(CapabilityUnavailable):
        arm.beep(1, 300)


# ---------------------------------------------------------------------- TeamArm față de răspunsurile serviciului
def test_connect_reads_mode_and_sites(service):
    team = TeamArm(service.url, timeout_s=5)
    assert team.connect()["sim"] is False and team.mode == MODE_HARDWARE
    assert team.sites == {"medium": "grid", "zone_red": "point"}
    service.status["sim"] = True
    team.connect()
    assert team.mode == MODE_SIMULATION


def test_false_with_reason_marks_position_uncertain(service):
    service.responses["pick"] = (200, {"result": False, "reason": "punct inaccesibil x=220 y=-140 z=263"})
    team = TeamArm(service.url, timeout_s=5)
    assert team.pick("medium", (2, 2)) is False
    assert "inaccesibil" in team.last_reason
    assert team.position_uncertain  # constatarea 1: brațul s-a putut mișca înainte de refuz
    team.go_pose("observe_medium")
    assert not team.position_uncertain


@pytest.mark.parametrize("status, code, route, expected", [
    (400, "argument", "go_pose", InvalidCommand),
    (400, "limite", "move_joints", InvalidCommand),
    (400, "limite", "pick", MotionAborted),
    (400, "limite", "place", MotionAborted),
    (409, "calibrare", "go_pose", CalibrationMissing),
    (404, "ruta", "beep", CapabilityUnavailable),
    (405, "metoda", "home", ArmCommunicationError),
    (500, "hardware", "home", ArmCommunicationError),
])
def test_service_error_codes_are_mapped(service, status, code, route, expected):
    service.responses[route] = (status, {"error": {"code": code, "message": "test"}})
    team = TeamArm(service.url, timeout_s=5)
    calls = {
        "go_pose": lambda: team.go_pose("observe_medium"),
        "move_joints": lambda: team.move_joints([90, 90, 90, 90, 90], 100),
        "pick": lambda: team.pick("medium", (0, 0)),
        "place": lambda: team.place("medium", (0, 0)),
        "beep": lambda: team.beep(1),
        "home": team.home,
    }
    with pytest.raises(expected) as info:
        calls[route]()
    assert "[{}]".format(code) in str(info.value)
    if expected in (MotionAborted, ArmCommunicationError):
        assert team.position_uncertain


def test_read_joints_null_is_a_failed_read(service):
    service.responses["read_joints"] = (200, {"result": [90, None, 90, 90, 90, 60]})
    with pytest.raises(ArmCommunicationError, match="servo \\[2\\]"):
        TeamArm(service.url, timeout_s=5).read_joints()


def test_malformed_responses(service):
    service.responses["home"] = (200, {"rezultat": 1})
    service.responses["pick"] = (200, {"result": "da"})
    team = TeamArm(service.url, timeout_s=5)
    with pytest.raises(ArmCommunicationError):
        team.home()
    with pytest.raises(ArmCommunicationError):
        team.pick("medium", (0, 0))


def test_timeout_and_dead_service(service):
    service.delay_s = 0.6
    team = TeamArm(service.url, timeout_s=0.2)
    with pytest.raises(ArmTimeout):
        team.home()
    assert team.position_uncertain
    dead = TeamArm("http://127.0.0.1:9", timeout_s=1)
    with pytest.raises(ArmCommunicationError):
        dead.status()


def test_yaw_sign_inverted_only_on_grids(service):
    service.responses["pick"] = (200, {"result": True})
    service.responses["place"] = (200, {"result": True})
    team = TeamArm(service.url, timeout_s=5)
    team.connect()
    team.pick("medium", (1, 1), yaw_deg=12.0, uv=(1.6, 1.4))
    team.pick("zone_red", None, yaw_deg=12.0)
    team.place("zone_red", None, level=2, yaw_deg=-7.0)
    grid_pick, zone_pick = service.bodies("pick")
    assert grid_pick["yaw_deg"] == pytest.approx(to_service_yaw(12.0)) == pytest.approx(-12.0)
    assert grid_pick["uv"] == [1.6, 1.4] and grid_pick["cell"] == [1, 1]
    assert zone_pick["yaw_deg"] == pytest.approx(12.0) and zone_pick["cell"] is None  # zonă: sistemul robotului
    assert service.bodies("place")[0] == {"location": "zone_red", "cell": None, "level": 2, "yaw_deg": -7.0}


def test_speed_medium_is_sent_as_slow(service):
    team = TeamArm(service.url, timeout_s=5)
    team.go_pose("observe_medium", "medium")
    assert service.bodies("go_pose") == [{"name": "observe_medium", "speed": "slow"}]


# ---------------------------------------------------------------------- MockArm: erori injectate
def test_mock_faults_are_deterministic():
    arm = _mock_arm(faults=[FaultSpec("pick", "unreachable", call=1), FaultSpec("go_pose", "comm_error", call=2)])
    assert arm.pick("medium", (1, 1)) is False and arm.position_uncertain
    arm.go_pose("observe_medium")
    assert not arm.position_uncertain
    with pytest.raises(ArmCommunicationError):
        arm.go_pose("observe_medium")
    with pytest.raises(CalibrationMissing):
        arm.go_pose("observe_small")
    with pytest.raises(CalibrationMissing):
        arm.pick("small", (0, 0))
    assert [r.method for r in arm.log][-1] == "go_pose"


def test_mock_timeout_after_motion_leaves_world_changed():
    arm = _mock_arm(faults=[FaultSpec("pick", "timeout", call=1)])
    with pytest.raises(ArmTimeout):
        arm.pick("medium", (1, 1))
    assert arm.world.held is not None  # mișcarea s-a executat; doar răspunsul s-a pierdut
    with pytest.raises(ValueError):
        FaultSpec("pick", "explozie", call=1)
