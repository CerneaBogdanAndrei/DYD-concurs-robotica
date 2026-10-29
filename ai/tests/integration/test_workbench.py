"""Pagina de lucru: ținta din poză, editarea ei, planul și execuția (fără hardware)."""

import dataclasses
import http.client
import json
import math
import re
import threading
import time
import urllib.error
import urllib.request
from contextlib import contextmanager

import cv2
import numpy as np
import pytest

from brain.adapters.arm.mock_arm import MockArm
from brain.adapters.clock import ManualClock
from brain.adapters.preview_server import MAX_UPLOAD_BYTES, PreviewHub, PreviewServer
from brain.cli.preview import PreviewSession
from brain.cli.workbench import Workbench
from brain.domain.cube import Color
from brain.perception.pipeline import VisionPipeline
from brain.planning.goals import CellTarget, ReproduceGoal, load_reproduce_goal
from brain.ports.arm import MODE_SIMULATION
from brain.simulation.render import SimCamera, render_location
from brain.simulation.scenario import check_goal, make_renderer, oracle_observation
from brain.simulation.sim_world import SimCube, SimGrid, SimWorld

STATUS = {"sim": True, "poses": ["home", "observe_medium"], "sites": {"medium": "grid"}, "grip_calibrated": False,
          "grip_range_mm": None}
ROWS = [["yellow", "green", "blue"], ["red", "empty", "green"], ["blue", "red", "empty"]]
TOKEN = "abcdefgh"


def _call(method, url, data=None, headers=None):
    request = urllib.request.Request(url, data=data, method=method, headers=headers or {})
    try:
        with urllib.request.urlopen(request, timeout=120) as response:
            return response.status, json.loads(response.read() or b"null")
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read() or b"null")


def _post_json(url, payload, headers=None):
    return _call("POST", url, json.dumps(payload).encode("utf-8"), dict({"Content-Type": "application/json"}, **(headers or {})))


@contextmanager
def _serving(config, tmp_path, preview=None, hub=None, **kwargs):
    hub = hub or PreviewHub("medium")
    bench = Workbench(config, hub, preview, tmp_path / "preview", status_reader=lambda: dict(STATUS),
                      poll_service=False, **kwargs).start()
    server = PreviewServer(hub, "127.0.0.1", 0, api=bench).start()
    try:
        yield "http://127.0.0.1:{}".format(server.port), bench, hub
    finally:
        bench.stop()
        server.stop()


def _grid(system, rows=ROWS):
    loc = system.location("medium")
    grid = SimGrid("medium", 3, 3, loc.cell_mm)
    uid = 1
    for r, row in enumerate(rows):
        for c, value in enumerate(row):
            if value != "empty":
                grid.stack((r, c)).append(SimCube(uid, value, "full" if uid % 2 else "icon", 6.0 * ((uid % 3) - 1)))
                uid += 1
    return grid


def _phone_photo(system, grid, size=(1600, 1200), rotation_deg=22.0, keystone=0.14, side_px=620.0, seed=4):
    """O poză „de telefon”: rotită, cu perspectivă (latura de sus mai scurtă), la rezoluție mare."""
    loc = system.location("medium")
    width, height = size
    theta = math.radians(rotation_deg)
    c, s = math.cos(theta), math.sin(theta)
    half = side_px / 2.0
    local = [(-half * (1 - keystone), -half), (half * (1 - keystone), -half), (half, half), (-half, half)]
    quad = tuple((width / 2.0 + u * c + v * s, height / 2.0 - u * s + v * c) for u, v in local)
    camera = SimCamera(size, quad, 600.0, (1.5 * loc.cell_mm, 1.5 * loc.cell_mm), (0.03, -0.02), (1.0, 1.0, 1.0),
                       1.0, 0.5, 2.0)
    return render_location(grid, system.grid_geometry("medium"), camera, 1, np.random.default_rng(seed))[0]


def _encode(image, ext):
    ok, data = cv2.imencode(ext, image, [int(cv2.IMWRITE_JPEG_QUALITY), 90] if ext == ".jpg" else [])
    assert ok
    return data.tobytes()


# ---------------------------------------------------------------------- pagina
def test_page_is_self_contained_and_hides_execute_when_disabled(app_config, tmp_path):
    with _serving(app_config, tmp_path) as (base, bench, _):
        with urllib.request.urlopen(base + "/", timeout=10) as response:
            page = response.read().decode("utf-8")
        assert not re.search(r"https?://|(src|href)\s*=\s*[\"']//", page), "pagina nu are voie să încarce resurse externe"
        assert 'id="x-run"' not in page and "Încarcă poza țintă" in page and 'capture="environment"' in page
        assert "prefers-color-scheme: dark" in page and "max-width: 700px" in page
        status = json.loads(urllib.request.urlopen(base + "/status.json", timeout=10).read())
        assert status["workbench"]["service"]["ok"] is True and status["workbench"]["execute_enabled"] is False
    enabled = dataclasses.replace(app_config, system=dataclasses.replace(
        app_config.system, preview=dataclasses.replace(app_config.system.preview, allow_execute=True)))
    with _serving(enabled, tmp_path) as (base, bench, _):
        assert 'id="x-run"' in urllib.request.urlopen(base + "/", timeout=10).read().decode("utf-8")


def test_service_down_is_reported_not_raised(app_config, tmp_path):
    hub = PreviewHub("medium")

    def down():
        raise OSError("connection refused")

    bench = Workbench(app_config, hub, None, tmp_path, status_reader=down, poll_service=False).start()
    snapshot = bench.status()["service"]
    assert snapshot["ok"] is False and "nu răspunde" in snapshot["error"]


# ---------------------------------------------------------------------- ținta din poză
@pytest.mark.parametrize("ext", [".png", ".jpg"])
def test_upload_phone_photo_recognizes_arrangement(app_config, tmp_path, ext):
    system = app_config.system
    photo = _phone_photo(system, _grid(system))
    with _serving(app_config, tmp_path) as (base, bench, _):
        status, data = _call("POST", base + "/target/image", _encode(photo, ext),
                             {"Content-Type": "image/png" if ext == ".png" else "image/jpeg"})
        assert status == 200, data
        assert data["actionable"] is True and data["scaled_from"] == [1600, 1200] and max(data["image_size"]) == 1280
        assert [[c["value"] for c in row] for row in data["cells"]] == ROWS
        assert data["uncertain"] == [] and "găsit" in data["message"]
        with urllib.request.urlopen(base + "/target/annotated.jpg", timeout=10) as response:
            assert response.headers.get("Content-Type") == "image/jpeg"


def test_upload_without_grid_asks_for_manual_edit(app_config, tmp_path):
    blank = np.full((900, 1200, 3), 200, np.uint8)
    cv2.circle(blank, (600, 450), 120, (40, 40, 210), -1)
    with _serving(app_config, tmp_path) as (base, bench, _):
        status, data = _call("POST", base + "/target/image", _encode(blank, ".png"), {"Content-Type": "image/png"})
        assert status == 200
        assert data["actionable"] is False and "NU e găsit" in data["message"]
        assert any("caroiaj" in text for text in data["reasons_text"])
        assert all(c["value"] == "unknown" for row in data["cells"] for c in row)


def test_upload_limits_and_bad_files(app_config, tmp_path):
    with _serving(app_config, tmp_path) as (base, bench, _):
        port = int(base.rsplit(":", 1)[1])
        connection = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
        connection.putrequest("POST", "/target/image")
        connection.putheader("Content-Type", "image/jpeg")
        connection.putheader("Content-Length", str(MAX_UPLOAD_BYTES + 1))
        connection.endheaders()  # corpul nu se trimite: serverul refuză după antet
        response = connection.getresponse()
        assert response.status == 413 and "prea mare" in json.loads(response.read())["error"]
        connection.close()
        assert _call("POST", base + "/target/image", b"nu e o imagine", {"Content-Type": "image/png"})[0] == 400
        status, data = _call("POST", base + "/target/image", b"text", {"Content-Type": "text/plain"})
        assert status == 415 and "imagine" in data["error"]


# ---------------------------------------------------------------------- ținta editată
def test_post_target_is_validated_and_saved_for_reproduce(app_config, tmp_path):
    with _serving(app_config, tmp_path) as (base, bench, _):
        assert _post_json(base + "/target", {"rows": [["red"] * 3] * 2})[0] == 400  # nu e 3x3
        assert _post_json(base + "/target", {"rows": [["purple", "empty", "empty"]] + [["empty"] * 3] * 2})[0] == 400
        assert _post_json(base + "/target", {"rows": ROWS, "extra": 1})[0] == 400
        assert _post_json(base + "/target", {"rows": ROWS, "location": "large"})[0] == 400  # șablon inactiv
        status, data = _post_json(base + "/target", {"location": "medium", "rows": ROWS, "from_photo": False})
        assert status == 200 and data["confirmed"]["cells"] == ROWS
        goal = load_reproduce_goal(data["confirmed"]["file"])  # formatul pentru brain run --target-file
        assert goal.location == "medium" and goal.cells[next(iter(sorted(goal.cells)))].top is Color.YELLOW
        assert _call("GET", base + "/target")[1]["confirmed"]["cells"] == ROWS
    with _serving(app_config, tmp_path) as (base, bench, _):  # după repornire ținta e citită de pe disc
        assert _call("GET", base + "/target")[1]["confirmed"]["cells"] == ROWS


# ---------------------------------------------------------------------- planul
def test_plan_from_known_state_and_target(app_config, tmp_path):
    system = app_config.system
    world = SimWorld([_grid(system, [["yellow", "empty", "empty"], ["empty"] * 3, ["red", "empty", "empty"]])], [], 29.0)
    with _serving(app_config, tmp_path) as (base, bench, hub):
        status, plan = _call("POST", base + "/plan")
        assert status == 200 and plan["ok"] is False and len(plan["problems"]) == 2  # fără țintă, fără observație
        hub.publish_analysis(None, {"updated": "x"}, oracle_observation(world, system, "medium"))
        _post_json(base + "/target", {"rows": [["yellow", "red", "empty"], ["empty"] * 3, ["empty"] * 3]})
        status, plan = _call("POST", base + "/plan")
        assert status == 200 and plan["ok"] and plan["complete"] and not plan["already_done"]
        assert [m["text"] for m in plan["moves"]] == ["roșu din r2c0 → r0c1"]
        assert plan["moves"][0]["grip"].startswith("gheară pre-închisă la 39 mm") and plan["estimated_s"] > 0
        _post_json(base + "/target", {"rows": [["yellow", "empty", "empty"], ["empty"] * 3, ["red", "empty", "empty"]]})
        assert _call("POST", base + "/plan")[1]["already_done"] is True
        _post_json(base + "/target", {"rows": [["blue", "empty", "empty"], ["empty"] * 3, ["empty"] * 3]})
        plan = _call("POST", base + "/plan")[1]
        assert not plan["complete"] and any("albastru" in text for text in plan["unmet"])


# ---------------------------------------------------------------------- execuția
def test_execute_is_forbidden_when_disabled_or_with_wrong_token(app_config, tmp_path):
    with _serving(app_config, tmp_path) as (base, bench, _):
        for path in ("/execute", "/execute/token", "/execute/stop"):
            status, data = _call("POST", base + path, b"", {"X-Brain-Token": "x"})
            assert status == 403 and "dezactivată" in data["error"]
    enabled = dataclasses.replace(app_config, system=dataclasses.replace(
        app_config.system, preview=dataclasses.replace(app_config.system.preview, allow_execute=True)))
    with _serving(enabled, tmp_path, token=TOKEN) as (base, bench, _):
        status, data = _call("POST", base + "/execute", b"", {"X-Brain-Token": "gresit"})
        assert status == 403 and "cod greșit" in data["error"]
        assert _call("POST", base + "/execute/token", b"", {"X-Brain-Token": TOKEN})[0] == 200
        assert _call("POST", base + "/execute", b"", {"X-Brain-Token": TOKEN})[0] == 409  # nicio țintă confirmată
        for _ in range(10):
            _call("POST", base + "/execute/token", b"", {"X-Brain-Token": "gresit"})
        status, data = _call("POST", base + "/execute/token", b"", {"X-Brain-Token": TOKEN})
        assert status == 403 and "blocată" in data["error"]  # blocată chiar și cu codul bun


class _LockedArm:
    """MockArm-ul cu un lacăt comun cu camera sintetică (lumea nu se schimbă în timp ce e randată)."""

    def __init__(self, arm, lock, after_call=None):
        self._arm, self._lock, self._after = arm, lock, after_call
        self.mode = arm.mode

    def connect(self):
        return dict(STATUS)

    def __getattr__(self, name):
        attribute = getattr(self._arm, name)
        if not callable(attribute):
            return attribute

        def locked(*args, **kwargs):
            with self._lock:
                result = attribute(*args, **kwargs)
            if self._after is not None:
                self._after(name)
            return result

        return locked


class _WorldCamera:
    def __init__(self, render, lock):
        self._render, self._lock = render, lock

    def grab(self, count):
        with self._lock:
            return self._render("medium", count)


def test_execute_end_to_end_with_mock_arm_and_synthetic_camera(app_config, tmp_path):
    preview_settings = dataclasses.replace(app_config.system.preview, allow_execute=True, max_fps=12.0,
                                           analyze_interval_s=0.3)
    config = dataclasses.replace(app_config, system=dataclasses.replace(app_config.system, preview=preview_settings))
    system = config.system
    world = SimWorld([_grid(system, [["red", "empty", "empty"], ["empty"] * 3, ["empty", "empty", "blue"]])], [], 29.0, seed=3)
    lock = threading.RLock()
    mock = MockArm(world, ["observe_medium"], ManualClock(), seed=3, gripper=system.gripper)
    hub = PreviewHub("medium")
    session = PreviewSession(_WorldCamera(make_renderer(world, system, 3), lock), hub, preview_settings,
                             pipeline=VisionPipeline(system, config.colors), use_process=False,
                             save_dir=None).start()
    try:
        with _serving(config, tmp_path, preview=session, hub=hub, arm_factory=lambda: _LockedArm(mock, lock),
                      expected_mode=MODE_SIMULATION, clock_factory=ManualClock, token=TOKEN) as (base, bench, _):
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline and not (hub.latest_observation()[0] is not None
                                                       and hub.latest_observation()[0].actionable):
                time.sleep(0.1)
            target = [["blue", "empty", "empty"], ["empty", "red", "empty"], ["empty"] * 3]
            assert _post_json(base + "/target", {"rows": target})[0] == 200
            plan = _call("POST", base + "/plan")[1]
            assert plan["ok"] and len(plan["moves"]) == 2 and plan["complete"], plan
            status, started = _call("POST", base + "/execute", b"", {"X-Brain-Token": TOKEN})
            assert status == 200 and started["started"]
            assert _call("POST", base + "/execute", b"", {"X-Brain-Token": TOKEN})[0] == 409  # una singură odată
            deadline = time.monotonic() + 180
            state = {}
            while time.monotonic() < deadline:
                state = _call("GET", base + "/execute")[1]
                if state["state"] != "running":
                    break
                time.sleep(0.2)
            assert state["state"] == "done", state
            assert state["final_check"]["ok"] and state["goal_reached"], state
            assert all(m["outcome"] == "ok" for m in state["moves"])
            goal = ReproduceGoal("medium", {k: v for k, v in load_reproduce_goal(
                bench.get_target()["confirmed"]["file"]).cells.items()})
            assert check_goal(world, goal, config.sort_map)[0], world.events  # adevărul simulat
            assert mock.collisions == []  # degetele n-au atins niciun vecin
            assert (tmp_path / "preview" / "runs").is_dir()
            assert not session.pause_analysis.is_set()  # analiza periodică a repornit
    finally:
        session.stop()


def test_stop_request_halts_between_commands(app_config, tmp_path):
    preview_settings = dataclasses.replace(app_config.system.preview, allow_execute=True, max_fps=12.0,
                                           analyze_interval_s=0.3)
    config = dataclasses.replace(app_config, system=dataclasses.replace(app_config.system, preview=preview_settings))
    system = config.system
    world = SimWorld([_grid(system, [["red", "yellow", "empty"], ["empty"] * 3, ["empty"] * 3])], [], 29.0, seed=5)
    lock = threading.RLock()
    mock = MockArm(world, ["observe_medium"], ManualClock(), seed=5, gripper=system.gripper)
    hub = PreviewHub("medium")
    session = PreviewSession(_WorldCamera(make_renderer(world, system, 5), lock), hub, preview_settings,
                             pipeline=VisionPipeline(system, config.colors), use_process=False).start()
    holder = {}

    def stop_after_first_move(name):  # „Oprește” apăsat cât timp brațul execută prima comandă
        if name == "go_pose":
            holder["bench"].execution.request_stop()

    try:
        with _serving(config, tmp_path, preview=session, hub=hub,
                      arm_factory=lambda: _LockedArm(mock, lock, stop_after_first_move),
                      expected_mode=MODE_SIMULATION, clock_factory=ManualClock, token=TOKEN) as (base, bench, _):
            holder["bench"] = bench
            _post_json(base + "/target", {"rows": [["empty"] * 3, ["empty"] * 3, ["yellow", "red", "empty"]]})
            assert _call("POST", base + "/execute", b"", {"X-Brain-Token": TOKEN})[0] == 200
            assert bench.execution.wait(60)
            state = bench.execution_status()
            assert state["state"] == "stopped" and "oprire cerută" in (state["error"] or ""), state
            assert [r.method for r in mock.log] == ["go_pose"]  # comanda curentă s-a terminat, apoi nimic (fără home)
            assert "nefăcută" in state["final_check"]["summary"]
    finally:
        session.stop()


def test_compare_with_target_reports_each_cell(app_config):
    from brain.cli.web_execution import compare_with_target
    from brain.domain.grid import GridCoord

    world = SimWorld([SimGrid("medium", 3, 3, 43.0)], [], 29.0)
    world.grids["medium"].stack((0, 0)).append(SimCube(1, "red", "full", 0.0))
    goal = ReproduceGoal("medium", {GridCoord(0, 0): CellTarget(Color.RED), GridCoord(1, 1): CellTarget(Color.BLUE)})
    check = compare_with_target(oracle_observation(world, app_config.system, "medium"), goal)
    assert check["summary"] == "1/2 celule ca în țintă" and not check["ok"]
    assert check["cells"][1][1] == {"want": "blue", "seen": "empty", "ok": False}
