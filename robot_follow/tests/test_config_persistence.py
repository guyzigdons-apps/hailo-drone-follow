"""Tests for ControllerConfig save / live-load round-trip.

Covers:
- save_json → from_json preserves every serialisable field.
- load_from_file mutates the target ControllerConfig in place (callers hold
  the reference) and reports which fields changed.
- load_from_file raises on validation failure and rolls back cleanly.
- Missing file raises FileNotFoundError.
- DEFAULT_CONFIG_PATH points inside the repo root (not /usr, not the schema).
"""

import json
import os
import tempfile

import pytest

from robot_follow.follow_api.config import ControllerConfig, DEFAULT_CONFIG_PATH


def test_default_config_path_in_repo_root():
    """Default save path lives at <repo_root>/df_config.json, not under /usr."""
    assert DEFAULT_CONFIG_PATH.endswith("/df_config.json")
    assert "/usr/" not in DEFAULT_CONFIG_PATH
    # Parent must be the repo root — where df_params.json also lives.
    parent = os.path.dirname(DEFAULT_CONFIG_PATH)
    assert os.path.isfile(os.path.join(parent, "df_params.json")), \
        f"df_params.json not found next to df_config.json at {parent}"


def test_save_roundtrip_preserves_all_fields(tmp_path):
    """save_json + from_json restores every field identically."""
    cfg = ControllerConfig(
        kp_yaw=7.5, kp_distance=2.3, max_forward=1.8,
        target_bbox_height=0.42, auto_select=False, max_forward_accel=0.9,
        yaw_only=False,
    )
    path = str(tmp_path / "df_config.json")
    cfg.save_json(path)

    loaded = ControllerConfig.from_json(path)
    assert loaded.kp_yaw == 7.5
    assert loaded.kp_distance == 2.3
    assert loaded.max_forward == 1.8
    assert loaded.target_bbox_height == 0.42
    assert loaded.auto_select is False
    assert loaded.max_forward_accel == 0.9
    assert loaded.yaw_only is False


def test_load_from_file_mutates_in_place(tmp_path):
    """load_from_file updates the same object; callers keep their reference valid."""
    cfg = ControllerConfig(kp_yaw=1.0, max_forward_accel=1.5)
    original_id = id(cfg)

    path = str(tmp_path / "df_config.json")
    snapshot = ControllerConfig(kp_yaw=9.9, max_forward_accel=2.7)
    snapshot.save_json(path)

    changed = cfg.load_from_file(path)
    assert id(cfg) == original_id, "reference must not change"
    assert cfg.kp_yaw == 9.9
    assert cfg.max_forward_accel == 2.7
    assert set(changed) >= {"kp_yaw", "max_forward_accel"}


def test_load_from_file_reports_only_changed(tmp_path):
    """Fields identical between memory and file are not in the returned list."""
    cfg = ControllerConfig(kp_yaw=5.0, kp_distance=1.5)
    path = str(tmp_path / "df_config.json")
    # Only kp_yaw differs on disk
    disk = ControllerConfig(kp_yaw=7.0, kp_distance=1.5)
    disk.save_json(path)

    changed = cfg.load_from_file(path)
    assert "kp_yaw" in changed
    assert "kp_distance" not in changed


def test_load_from_file_rolls_back_on_validation_error(tmp_path):
    """If the loaded values fail validate(), previous values are restored."""
    cfg = ControllerConfig(min_altitude=2.0, max_altitude=20.0)
    path = str(tmp_path / "df_config.json")
    # Write an invalid combo directly — min >= max trips validate()
    with open(path, "w") as f:
        json.dump({"min_altitude": 15.0, "max_altitude": 5.0}, f)

    with pytest.raises(ValueError):
        cfg.load_from_file(path)
    # Roll-back: values unchanged
    assert cfg.min_altitude == 2.0
    assert cfg.max_altitude == 20.0


def test_load_from_file_missing():
    cfg = ControllerConfig()
    with pytest.raises(FileNotFoundError):
        cfg.load_from_file("/nonexistent/path/df_config.json")


def test_load_from_file_ignores_unknown_keys(tmp_path):
    """Unknown keys in the file are silently ignored (no AttributeError)."""
    cfg = ControllerConfig(kp_yaw=1.0)
    path = str(tmp_path / "df_config.json")
    with open(path, "w") as f:
        json.dump({"kp_yaw": 4.0, "not_a_real_field": 99}, f)
    changed = cfg.load_from_file(path)
    assert cfg.kp_yaw == 4.0
    assert "not_a_real_field" not in changed
    assert not hasattr(cfg, "not_a_real_field")


def test_kp_alt_hold_round_trip(tmp_path):
    """kp_alt_hold survives save/load — it's a tunable controller field."""
    cfg = ControllerConfig(kp_alt_hold=0.7)
    p = str(tmp_path / "df_config.json")
    cfg.save_json(p)
    loaded = ControllerConfig.from_json(p)
    assert loaded.kp_alt_hold == pytest.approx(0.7)


def test_kp_alt_hold_in_df_params():
    """Slider for kp_alt_hold exists so QOpenHD/web-UI can tune it."""
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    with open(os.path.join(repo_root, "df_params.json")) as f:
        params = json.load(f)["params"]
    ids = {p["id"] for p in params}
    assert "kp_alt_hold" in ids


def test_kp_alt_hold_in_openhd_bridge_params():
    """OpenHD MAVLink bridge exposes kp_alt_hold so QOpenHD can set it.

    CLEAN-14: openhd_bridge no longer owns its own param dict; it iterates
    ControllerConfig.tunable_fields() filtered to entries with a mavlink_id.
    """
    from robot_follow.follow_api.config import ControllerConfig
    tf = ControllerConfig.tunable_fields()
    assert "kp_alt_hold" in tf
    assert tf["kp_alt_hold"].mavlink_id is not None


def test_kp_alt_hold_in_web_server_fields():
    """Web UI /config endpoint exposes kp_alt_hold.

    CLEAN-14: web_server no longer owns its own field dict; it iterates
    ControllerConfig.tunable_fields() (every entry, including web-UI-only).
    """
    from robot_follow.follow_api.config import ControllerConfig
    assert "kp_alt_hold" in ControllerConfig.tunable_fields()


def test_forward_velocity_deadband_round_trip(tmp_path):
    """forward_velocity_deadband survives save/load — it's a tunable controller field."""
    cfg = ControllerConfig(forward_velocity_deadband=0.12)
    p = str(tmp_path / "df_config.json")
    cfg.save_json(p)
    loaded = ControllerConfig.from_json(p)
    assert loaded.forward_velocity_deadband == pytest.approx(0.12)


def test_forward_velocity_deadband_in_df_params():
    """Slider for forward_velocity_deadband exists so QOpenHD/web-UI can tune it."""
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    with open(os.path.join(repo_root, "df_params.json")) as f:
        params = json.load(f)["params"]
    ids = {p["id"] for p in params}
    assert "forward_velocity_deadband" in ids


def test_forward_velocity_deadband_in_openhd_bridge_params():
    """OpenHD MAVLink bridge exposes forward_velocity_deadband so QOpenHD can set it.

    CLEAN-14: see test_kp_alt_hold_in_openhd_bridge_params for the schema source.
    """
    from robot_follow.follow_api.config import ControllerConfig
    tf = ControllerConfig.tunable_fields()
    assert "forward_velocity_deadband" in tf
    assert tf["forward_velocity_deadband"].mavlink_id is not None


def test_forward_velocity_deadband_in_web_server_fields():
    """Web UI /config endpoint exposes forward_velocity_deadband.

    CLEAN-14: see test_kp_alt_hold_in_web_server_fields for the schema source.
    """
    from robot_follow.follow_api.config import ControllerConfig
    assert "forward_velocity_deadband" in ControllerConfig.tunable_fields()


def test_vfov_field_removed():
    """CLEAN-03: `vfov` is gone from ControllerConfig (no readers anywhere)."""
    from dataclasses import fields
    from robot_follow.follow_api.config import ControllerConfig
    assert "vfov" not in {f.name for f in fields(ControllerConfig)}


def test_tunable_fields_source_of_truth():
    """CLEAN-14: ControllerConfig.tunable_fields() is the single schema source.

    Replaces the historic web_server._CONFIG_FIELDS and
    openhd_bridge._CONFIG_PARAMS dicts. Every entry must be a TunableField
    with a Python type; mavlink_id may be None (web-UI-only) or a
    <=16 char string.
    """
    from robot_follow.follow_api.config import ControllerConfig, TunableField
    tf = ControllerConfig.tunable_fields()
    assert isinstance(tf, dict)
    assert len(tf) >= 24, f"expected at least 24 tunable fields, got {len(tf)}"
    for k, v in tf.items():
        assert isinstance(v, TunableField), f"{k} schema is not TunableField: {v!r}"
        assert isinstance(v.py_type, type), f"{k} py_type is not a type"
        assert v.mavlink_id is None or (
            isinstance(v.mavlink_id, str) and len(v.mavlink_id) <= 16
        ), f"{k} mavlink_id invalid: {v.mavlink_id!r}"
        # Every tunable field must be an actual attribute on the dataclass.
        assert hasattr(ControllerConfig(), k), f"{k} not a real ControllerConfig field"
