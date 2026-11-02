import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "main"))
from control_jobs import EVENT_PREFIX, create_job, parse_event
from control_model import Field, apply_fields, parse_value

from uavsim.config.loader import load_config
from uavsim.config.validator import Config


def test_navigation_editor_changes_preserve_unexposed_parameters():
    cfg = Config()
    field = Field("disturbances.navigation_attitude_error.bias_std_body_deg", "姿态 σ", "vector")
    updated = apply_fields(cfg, [field], {field.path: ["0.1", "0.2", "0.3"]})
    assert updated.disturbances.navigation_attitude_error.bias_std_body_deg == (0.1, 0.2, 0.3)
    assert cfg.disturbances.navigation_attitude_error.bias_std_body_deg == (0.05, 0.05, 0.1)
    assert updated.vehicle == cfg.vehicle
    assert updated.controller == cfg.controller
    assert updated.pass_seeds == cfg.pass_seeds


@pytest.mark.parametrize(
    "field,value",
    [
        (Field("base_seed", "随机种子", "int"), "3.5"),
        (Field("trajectory.speed_mps", "速度"), "nan"),
        (Field("nominal_baseline_sar_m", "基线", "vector"), "1,2"),
        (Field("trajectory.speed_mps", "速度"), ""),
    ],
)
def test_editor_rejects_invalid_numeric_values(field, value):
    with pytest.raises(ValueError):
        parse_value(field, value)


def test_editor_uses_core_constraints_and_supports_zero_axis_std():
    cfg = Config()
    field = Field("disturbances.navigation_velocity_error.bias_std_sar_mps", "速度 σ", "vector")
    result = apply_fields(cfg, [field], {field.path: "0,0,.05"})
    assert result.disturbances.navigation_velocity_error.bias_std_sar_mps == (0.0, 0.0, 0.05)
    with pytest.raises(ValueError):
        apply_fields(cfg, [field], {field.path: [0.0, -0.1, 0.05]})
    duration = Field("simulation.acquisition_duration_s", "采集时长")
    with pytest.raises(ValueError, match="采样网格"):
        apply_fields(cfg, [duration], {duration.path: "0.12345"})


def test_job_snapshots_are_independent_and_never_overwrite(tmp_path):
    cfg = Config()
    first = create_job(tmp_path, cfg, "simulate")
    second = create_job(tmp_path, cfg, "simulate")
    assert first != second
    assert load_config(first / "input_config.yaml") == cfg
    (first / "retained.csv").write_text("keep", encoding="utf-8")
    assert (first / "retained.csv").read_text() == "keep"
    assert json.loads((second / "session.json").read_text())["status"] == "created"


def test_worker_protocol_rejects_unknown_event_types():
    assert parse_event("normal log line") is None
    assert parse_event(EVENT_PREFIX + '{"type":"result","status":"simulated"}')["status"] == "simulated"
    with pytest.raises(ValueError):
        parse_event(EVENT_PREFIX + '{"type":"run_shell"}')
