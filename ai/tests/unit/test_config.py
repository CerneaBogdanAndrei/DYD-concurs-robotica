import shutil
from pathlib import Path

import pytest
import yaml

from brain.configuration import load_app_config, load_colors_config, load_system_config
from brain.domain.cube import Color
from brain.domain.errors import ConfigError


def test_real_config_loads(app_config):
    system, colors = app_config.system, app_config.colors
    assert set(system.locations) == {"small", "medium", "large"}
    assert system.vision.detection_method == "lines"
    assert system.cube.calibrated is True and system.cube.size_mm == 29.0  # măsurat de echipă
    assert colors.calibrated is False
    assert system.px_per_mm("medium") == pytest.approx(100.0 / 43.0)
    assert system.cube_px("medium") == pytest.approx(29.0 * 100.0 / 43.0)
    assert system.gripper is not None and system.gripper.side_space_mm == pytest.approx(7.0)
    assert system.preview.allow_execute is False
    assert system.planning.active_locations == ("medium",)
    assert app_config.sort_map.zone_for(Color.RED).name == "zone_red"
    assert {c.color for c in colors.classes} == {Color.RED, Color.YELLOW, Color.GREEN, Color.BLUE}


def _edited(tmp_path: Path, config_dir: Path, name: str, edit) -> Path:
    for file in ("system.yaml", "colors.yaml"):
        shutil.copy(config_dir / file, tmp_path / file)
    path = tmp_path / name
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    edit(data)
    path.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")
    return path


def _assert_error(exc_info, file_name: str, key: str):
    err = exc_info.value
    assert err.file.endswith(file_name)
    assert err.key == key
    assert file_name in str(err) and key in str(err)


def test_unknown_key_is_rejected(tmp_path, config_dir):
    path = _edited(tmp_path, config_dir, "system.yaml", lambda d: d["vision"]["cells"].update(roi_margn_px=3))
    with pytest.raises(ConfigError) as exc_info:
        load_system_config(path)
    _assert_error(exc_info, "system.yaml", "vision.cells.roi_margn_px")


def test_out_of_range_value(tmp_path, config_dir):
    path = _edited(tmp_path, config_dir, "system.yaml", lambda d: d["locations"]["small"].update(cell_mm=25.0))
    with pytest.raises(ConfigError) as exc_info:
        load_system_config(path)
    _assert_error(exc_info, "system.yaml", "locations.small.cell_mm")
    assert exc_info.value.value == 25.0


def test_missing_key(tmp_path, config_dir):
    path = _edited(tmp_path, config_dir, "system.yaml", lambda d: d["vision"]["geometry"].pop("min_line_support"))
    with pytest.raises(ConfigError) as exc_info:
        load_system_config(path)
    _assert_error(exc_info, "system.yaml", "vision.geometry.min_line_support")


def test_wrong_type(tmp_path, config_dir):
    path = _edited(tmp_path, config_dir, "system.yaml", lambda d: d["vision"].update(frames_to_aggregate="cinci"))
    with pytest.raises(ConfigError) as exc_info:
        load_system_config(path)
    _assert_error(exc_info, "system.yaml", "vision.frames_to_aggregate")


def test_aruco_method_requires_section(tmp_path, config_dir):
    def edit(d):
        d["vision"]["detection_method"] = "aruco"
        d["vision"]["aruco"] = None

    path = _edited(tmp_path, config_dir, "system.yaml", edit)
    with pytest.raises(ConfigError) as exc_info:
        load_system_config(path)
    _assert_error(exc_info, "system.yaml", "vision.aruco")


def test_odd_rotation_requires_square_grid(tmp_path, config_dir):
    path = _edited(tmp_path, config_dir, "system.yaml",
                   lambda d: d["locations"]["large"].update(rows=1, cols=4, orientation_quarter_turns=1))
    with pytest.raises(ConfigError) as exc_info:
        load_system_config(path)
    _assert_error(exc_info, "system.yaml", "locations.large.orientation_quarter_turns")


def test_overlapping_hue_ranges(tmp_path, config_dir):
    path = _edited(tmp_path, config_dir, "colors.yaml", lambda d: d["colors"]["yellow"].update(hue_ranges=[[8, 30]]))
    with pytest.raises(ConfigError) as exc_info:
        load_colors_config(path)
    _assert_error(exc_info, "colors.yaml", "colors.yellow.hue_ranges[0]")
    assert "red" in str(exc_info.value)


def test_invalid_hue_range(tmp_path, config_dir):
    path = _edited(tmp_path, config_dir, "colors.yaml", lambda d: d["colors"]["blue"].update(hue_ranges=[[130, 200]]))
    with pytest.raises(ConfigError) as exc_info:
        load_colors_config(path)
    _assert_error(exc_info, "colors.yaml", "colors.blue.hue_ranges[0]")


def test_unknown_color(tmp_path, config_dir):
    path = _edited(tmp_path, config_dir, "colors.yaml", lambda d: d["colors"].update(purple={"hue_ranges": [[140, 150]]}))
    with pytest.raises(ConfigError) as exc_info:
        load_colors_config(path)
    _assert_error(exc_info, "colors.yaml", "colors.purple")


def test_schema_version_mismatch(tmp_path, config_dir):
    path = _edited(tmp_path, config_dir, "colors.yaml", lambda d: d.update(schema_version=2))
    with pytest.raises(ConfigError) as exc_info:
        load_colors_config(path)
    _assert_error(exc_info, "colors.yaml", "schema_version")


def test_yaml_syntax_error_and_missing_file(tmp_path):
    bad = tmp_path / "system.yaml"
    bad.write_text("grid: [unclosed\n", encoding="utf-8")
    with pytest.raises(ConfigError) as exc_info:
        load_system_config(bad)
    assert exc_info.value.key == "<yaml>"
    with pytest.raises(ConfigError) as exc_info:
        load_app_config(tmp_path / "nu-exista")
    assert exc_info.value.key == "<fișier>"
