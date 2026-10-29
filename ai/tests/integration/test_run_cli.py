"""`brain run` fără --execute (doar observă și planifică), `memorize`, `calib corners` și siguranța execuției."""

import json
import shutil
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from brain.cli.main import EXIT_OK, EXIT_STOPPED, main
from brain.cli.run import CONFIRM_WORD, check_sites, run_task
from brain.configuration import load_app_config
from brain.domain.errors import SafetyRejected
from brain.perception.image_io import write_image
from brain.planning.goals import TowerGoal, parse_order, parse_slot
from brain.simulation.render import make_camera, render_location
from brain.simulation.sim_world import SimCube, SimGrid


@pytest.fixture(scope="module")
def frames_dir(tmp_path_factory, app_config):
    """Cadre sintetice ale șablonului medium: roșu pe r0c0, galben pe r2c2, albastru pe r0c2."""
    system = app_config.system
    loc = system.location("medium")
    grid = SimGrid("medium", 3, 3, loc.cell_mm)
    for uid, (cell, color) in enumerate((((0, 0), "red"), ((2, 2), "yellow"), ((0, 2), "blue")), 1):
        grid.stack(cell).append(SimCube(uid, color, "full", 4.0))
    camera = make_camera(3, system.grid_geometry("medium"), 3, 3, loc.camera_height_mm, loc.nadir_uv)
    folder = tmp_path_factory.mktemp("frames")
    for k, frame in enumerate(render_location(grid, system.grid_geometry("medium"), camera, 5, np.random.default_rng(1))):
        write_image(folder / "medium" / "frame_{}.png".format(k), frame)
    return folder


def test_run_without_execute_only_plans(tmp_path, config_dir, frames_dir):
    out = tmp_path / "run"
    code = main(["--config-dir", str(config_dir), "run", "--task", "tower", "--order", "red,yellow", "--target",
                 "medium:r1c1", "--camera", "files", "--frames-dir", str(frames_dir), "--out", str(out)])
    assert code == EXIT_OK
    report = json.loads((out / "report.json").read_text(encoding="utf-8"))
    assert report["execute"] is False and report["arm_commands"] == 0 and not report["moves"]
    assert len(report["first_plan"]) == 2 and "red medium[0,0]" in report["first_plan"][0]
    assert (out / "scene_state.json").is_file() and (out / "log.jsonl").is_file()


def test_memorize_then_plan_reproduction(tmp_path, config_dir, frames_dir):
    target = tmp_path / "target.yaml"
    common = ["--config-dir", str(config_dir), "run", "--camera", "files", "--frames-dir", str(frames_dir)]
    assert main(common + ["--task", "memorize", "--target-out", str(target), "--out", str(tmp_path / "m")]) == EXIT_OK
    data = yaml.safe_load(target.read_text(encoding="utf-8"))
    assert data["location"] == "medium" and data["cells"]["r0c0"]["top"] == "red" and data["cells"]["r1c1"] == "empty"
    out = tmp_path / "r"
    assert main(common + ["--task", "reproduce", "--target-file", str(target), "--out", str(out)]) == EXIT_OK
    report = json.loads((out / "report.json").read_text(encoding="utf-8"))
    assert report["final_state"] == "DONE" and report["goal_reached"]  # scena e deja aranjamentul memorat


def test_execute_requires_confirmation_and_matching_mode(app_config, tmp_path, frames_dir, monkeypatch):
    calls = []

    class FakeTeamArm:
        def __init__(self, url, timeout_s):
            self.mode = "hardware"
            self.last_reason = None
            self.position_uncertain = False

        def connect(self):
            calls.append("connect")
            return {"sim": False, "poses": ["home", "observe_medium"], "sites": {"medium": "grid"}}

        def go_pose(self, *args, **kwargs):
            calls.append("go_pose")

    monkeypatch.setattr("brain.adapters.arm.team_arm.TeamArm", FakeTeamArm)
    args = SimpleNamespace(out=str(tmp_path / "x"), execute=True, service_sim=False, preview=False, port=None,
                           camera="files", frames_dir=str(frames_dir), camera_index=0, width=640, height=480,
                           state=None, task="tower", location=None, target_out=None)
    goal = TowerGoal(parse_order("red"), parse_slot("medium:r1c1"))
    with pytest.raises(SafetyRejected, match="nu a fost confirmată"):
        run_task(app_config, goal, args, confirm=lambda prompt: "nu")
    assert calls == ["connect"]  # nicio mișcare fără confirmare
    args.service_sim = True  # serviciul raportează brațul real, dar rularea cere simularea
    with pytest.raises(SafetyRejected, match="modul"):
        run_task(app_config, goal, args, confirm=lambda prompt: CONFIRM_WORD)


def test_check_sites(app_config):
    status = {"sites": {"medium": "grid", "buffer_1": "point"}, "poses": ["home", "observe_medium"]}
    import dataclasses

    config = dataclasses.replace(app_config, system=dataclasses.replace(app_config.system, planning=dataclasses.replace(
        app_config.system.planning, buffer_points=("buffer_1", "buffer_2"))))
    assert check_sites(config, status, need_zones=False).system.planning.buffer_points == ("buffer_1",)
    with pytest.raises(SafetyRejected, match="zone_red"):
        check_sites(config, status, need_zones=True)
    with pytest.raises(SafetyRejected, match="observe_medium"):
        check_sites(config, {"sites": {"medium": "grid"}, "poses": ["home"]}, need_zones=False)


def test_calib_corners_writes_config(tmp_path, config_dir, frames_dir):
    copy = tmp_path / "config"
    shutil.copytree(str(config_dir), str(copy))
    frames = str(frames_dir / "medium" / "frame_*.png")
    assert main(["--config-dir", str(copy), "calib", "corners", "--location", "medium", "--frames", frames, "--write"]) == EXIT_OK
    corners = load_app_config(copy).system.location("medium").expected_corners_px
    assert corners is not None and len(corners) == 4
    text = (copy / "system.yaml").read_text(encoding="utf-8")
    assert "MĂSURAT cu rigla" in text and "brain calib corners" in text  # comentariile rămân
    assert load_app_config(copy).system.location("small").expected_corners_px is None
    blank = tmp_path / "blank.png"
    write_image(blank, np.full((480, 640, 3), 200, np.uint8))
    assert main(["--config-dir", str(copy), "calib", "corners", "--location", "medium", "--frames", str(blank)]) == 2


def test_run_rejects_inactive_target(config_dir, frames_dir, tmp_path):
    code = main(["--config-dir", str(config_dir), "run", "--task", "tower", "--order", "red", "--target", "large:r0c0",
                 "--camera", "files", "--frames-dir", str(frames_dir), "--out", str(tmp_path / "y")])
    assert code == 2
    assert EXIT_STOPPED == 4
