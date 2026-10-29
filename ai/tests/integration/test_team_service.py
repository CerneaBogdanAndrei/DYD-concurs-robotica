"""Brain împotriva serviciului REAL al colegilor, pornit în simulare (`node dist/src/cli/server.js --sim`).

Sărit dacă Node sau `dist/` (npm run build) lipsesc. Folosește `tests/fixtures/calib_brain.json` (șablonul medium,
zonele S3, două puncte tampon; poziții de test, nu măsurători).
"""

import shutil
import socket
import subprocess
import time
import urllib.request
from pathlib import Path

import pytest

from brain.adapters.arm.team_arm import TeamArm
from brain.cli.sim_run import SimOptions, run_scenario
from brain.domain.errors import CalibrationMissing, InvalidCommand
from brain.simulation import scenario as sc

pytestmark = pytest.mark.service

REPO = Path(__file__).resolve().parents[3]
SERVER = REPO / "dist" / "src" / "cli" / "server.js"
CALIB = Path(__file__).resolve().parents[1] / "fixtures" / "calib_brain.json"


def _free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


@pytest.fixture(scope="module")
def service_url():
    node = shutil.which("node")
    if node is None or not SERVER.is_file():
        pytest.skip("serviciul colegilor nu e disponibil (node sau dist/ lipsă: npm install && npm run build)")
    port = _free_port()
    process = subprocess.Popen([node, str(SERVER), "--sim", "--port", str(port), "--calib", str(CALIB)], cwd=str(REPO),
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    url = "http://127.0.0.1:{}".format(port)
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        try:
            urllib.request.urlopen(url + "/status", timeout=1).read()
            break
        except OSError:
            time.sleep(0.2)
    else:
        process.kill()
        pytest.fail("serviciul nu a pornit")
    yield url
    process.terminate()
    process.wait(timeout=10)


def test_contract_against_real_service(service_url):
    arm = TeamArm(service_url, timeout_s=10)
    status = arm.connect()
    assert status["sim"] is True and arm.mode == "simulation"
    assert arm.sites["medium"] == "grid" and arm.sites["zone_red"] == "point"
    arm.go_pose("observe_medium")
    assert arm.pick("medium", (1, 1), yaw_deg=10.0, uv=(1.6, 1.4)) is True
    assert arm.place("zone_red", None, level=2) is True
    assert len(arm.read_joints()) == 6
    with pytest.raises(CalibrationMissing):
        arm.go_pose("observe_large")
    with pytest.raises(CalibrationMissing):
        arm.pick("large", (0, 0))
    with pytest.raises(InvalidCommand):
        arm.place("zone_red", (0, 0))  # zonă cu celulă: refuzat de serviciu
    arm.home()


@pytest.mark.parametrize("task", ["tower", "sort"])
def test_tasks_through_real_service(app_config, service_url, task):
    """TeeArm: fiecare comandă trece prin serviciul real (nume, accesibilitate, erori) și prin lumea simulată."""
    scenario = (sc.example_s2(app_config.system, app_config.sort_map) if task == "tower"
                else sc.random_scenario("sort", app_config.system, app_config.sort_map, 5))
    report = run_scenario(app_config, scenario, SimOptions(service_url=service_url))
    assert report["arm"] == "tee(service+mock)"
    assert report["final_state"] == "DONE" and report["success"], report["moves"]
