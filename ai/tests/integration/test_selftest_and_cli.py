import json

from brain.cli.main import EXIT_INPUT_ERROR, EXIT_NOT_ACTIONABLE, EXIT_OK, main
from brain.cli.selftest import run_stage0_selftest


def _strip_timing(report):
    report = dict(report)
    report.pop("duration_s")
    report["metrics"] = {k: v for k, v in report["metrics"].items() if not k.startswith("processing_ms")}
    return report


def test_selftest_is_deterministic_and_passes(app_config):
    first = run_stage0_selftest(app_config, seed=3, scenes=30, frames=3)
    second = run_stage0_selftest(app_config, seed=3, scenes=30, frames=3)
    assert _strip_timing(first) == _strip_timing(second)
    assert first["metrics"]["grid_wrong_accepted"] == 0
    assert first["metrics"]["high_confidence_error_rate"] == 0.0
    assert first["passed"], first["failures"]


def test_cli_end_to_end(tmp_path, config_dir):
    common = ["--config-dir", str(config_dir)]
    assert main(common + ["config", "validate"]) == EXIT_OK
    synth = tmp_path / "synth"
    assert main(common + ["sim", "generate", "--seed", "900", "--frames", "3", "--out", str(synth)]) == EXIT_OK
    frames = str(synth / "scene_900" / "frame_*.png")
    state = tmp_path / "mem" / "state.json"
    analyze = common + ["vision", "analyze", "--frames", frames, "--location", "medium", "--out", str(tmp_path / "an")]
    assert main(analyze + ["--state-out", str(state)]) == EXIT_OK
    observation = json.loads((tmp_path / "an" / "observation.json").read_text(encoding="utf-8"))
    assert observation["actionable"] and len(observation["cells"]) == 9
    assert (tmp_path / "an" / "annotated.jpg").stat().st_size > 0
    assert main(analyze + ["--state-out", str(state)]) == EXIT_INPUT_ERROR  # fără --overwrite
    assert main(analyze + ["--state-out", str(state), "--overwrite"]) == EXIT_OK
    assert main(common + ["memory", "show", "--path", str(state)]) == EXIT_OK

    assert main(common + ["sim", "generate", "--seed", "5", "--kind", "blank", "--frames", "3", "--out", str(synth)]) == EXIT_OK
    blank = common + ["vision", "analyze", "--frames", str(synth / "scene_5" / "frame_*.png"), "--location", "medium",
                      "--out", str(tmp_path / "blank")]
    assert main(blank) == EXIT_NOT_ACTIONABLE


def test_cli_errors(tmp_path, config_dir):
    assert main(["--config-dir", str(tmp_path), "config", "validate"]) == EXIT_INPUT_ERROR
    assert main(["--config-dir", str(config_dir), "memory", "show", "--path", str(tmp_path / "nu.json")]) == EXIT_INPUT_ERROR
    assert main(["--config-dir", str(config_dir), "vision", "analyze", "--frames", str(tmp_path / "x_*.png"),
                 "--location", "medium"]) == EXIT_INPUT_ERROR
