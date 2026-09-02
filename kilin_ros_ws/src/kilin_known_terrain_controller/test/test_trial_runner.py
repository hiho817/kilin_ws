from pathlib import Path

import pytest
import yaml

from kilin_known_terrain_controller.trial_runner import load_trial_config


def _write_trial(tmp_path: Path, recording: dict, mapper: dict | None = None) -> None:
    (tmp_path / "terrain_profile.yaml").write_text("/**: {ros__parameters: {}}\n")
    (tmp_path / "hip_pid.yaml").write_text("/**: {ros__parameters: {}}\n")
    (tmp_path / "fastlio_config.yaml").write_text("common: {}\n")
    (tmp_path / "terrain_mapper_profile.yaml").write_text("/**: {ros__parameters: {}}\n")
    config = {
        "recording": recording,
        "fastlio": {"enabled": True, "config": "fastlio_config.yaml"},
        "terrain_mapper": mapper or {"enabled": False},
        "controller": {
            "startup_wait_s": 8.0,
            "armed": False,
            "mode": "known_ramp",
            "use_speed_command": False,
            "speed_m_s": 0.1,
            "run_duration_s": 30.0,
            "hard_motion_limit_s": 35.0,
            "use_odometry": True,
            "odometry_relative_origin": False,
            "use_terrain_window": False,
            "terrain_window_timeout_s": 1.0,
            "vicon_trigger": True,
            "debug_publish": True,
            "terrain_profile": "terrain_profile.yaml",
            "hip_pid_profile": "hip_pid.yaml",
        },
    }
    (tmp_path / "trial_config.yaml").write_text(yaml.safe_dump(config))


def test_trial_config_accepts_record_all(tmp_path):
    _write_trial(tmp_path, {"enabled": True, "mode": "all"})
    assert load_trial_config(tmp_path)["recording"]["mode"] == "all"


def test_trial_config_accepts_selected_absolute_topics(tmp_path):
    _write_trial(
        tmp_path,
        {"enabled": True, "mode": "selected", "topics": ["/motor/state", "/Odometry"]},
    )
    assert load_trial_config(tmp_path)["recording"]["topics"] == ["/motor/state", "/Odometry"]


def test_trial_config_rejects_selected_without_topics(tmp_path):
    _write_trial(tmp_path, {"enabled": True, "mode": "selected", "topics": []})
    with pytest.raises(ValueError, match="recording.topics"):
        load_trial_config(tmp_path)


def test_trial_config_requires_mapper_profile_when_live_mapping_is_enabled(tmp_path):
    _write_trial(
        tmp_path,
        {"enabled": True, "mode": "all"},
        {"enabled": True, "profile": "terrain_mapper_profile.yaml"},
    )
    assert load_trial_config(tmp_path)["terrain_mapper"]["enabled"] is True


def test_trial_config_rejects_console_directory_outside_trial(tmp_path):
    _write_trial(tmp_path, {"enabled": True, "mode": "all"})
    config_path = tmp_path / "trial_config.yaml"
    config = yaml.safe_load(config_path.read_text())
    config["console_logging"] = {"enabled": True, "directory": "../outside"}
    config_path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match="console_logging.directory"):
        load_trial_config(tmp_path)
