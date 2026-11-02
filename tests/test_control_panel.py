import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "main" / "internal"))
import control_panel
from control_jobs import EVENT_PREFIX, create_job, parse_event
from control_model import Field, apply_fields, parse_value
from control_panel import find_report_path

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


def test_progress_protocol_and_parallel_pass_aggregation():
    for payload in ['{"type":"progress","percent":101}', '{"type":"progress","fraction":0.2,"pass_index":-1}']:
        with pytest.raises(ValueError):
            parse_event(EVENT_PREFIX + payload)
    panel = SimpleNamespace(
        progress_values=[0.0, 0.0], progress_percent=0.0,
        progress=SimpleNamespace(configure=lambda **kwargs: None),
        progress_text=SimpleNamespace(set=lambda value: None),
    )
    for index, fraction, expected in [(1, 1.0, 40), (0, 0.5, 60), (0, 0.2, 60), (0, 1.0, 80)]:
        event = parse_event(EVENT_PREFIX + json.dumps({"type": "progress", "pass_index": index, "fraction": fraction}))
        control_panel.ControlPanel._update_progress(panel, event)
        assert panel.progress_percent == expected


@pytest.mark.parametrize("mode", ["simulate", "acceptance", "direct"])
def test_find_report_from_navigation_data_directory(tmp_path, mode):
    if mode == "simulate":
        job = tmp_path / "simulate_1"
        data = job / "artifacts" / "run_1"
        report = job / "index.html"
    elif mode == "acceptance":
        job = tmp_path / "acceptance_1" / "artifacts" / "acceptance_2"
        data = job / "validate_1"
        report = job / "index.html"
    else:
        job = tmp_path / "outputs" / "acceptance_1"
        data = job / "validate_1"
        report = job / "index.html"
    data.mkdir(parents=True)
    report.write_text("report", encoding="utf-8")
    assert find_report_path(data) == report


def test_open_report_uses_system_file_handler_and_shows_failure(tmp_path, monkeypatch):
    report = tmp_path / "index.html"
    report.write_text("report", encoding="utf-8")
    errors = []
    monkeypatch.setattr(control_panel.messagebox, "showerror", lambda *args, **kwargs: errors.append(args))
    opened = []
    if os.name == "nt":
        monkeypatch.setattr(control_panel.os, "startfile", lambda path: opened.append(path))
    else:
        monkeypatch.setattr(control_panel.webbrowser, "open", lambda url: opened.append(url) or True)
    panel = SimpleNamespace(report_path=report, root=None)
    control_panel.ControlPanel.open_report(panel)
    assert len(opened) == 1
    assert not errors

    if os.name == "nt":
        monkeypatch.setattr(control_panel.os, "startfile", lambda path: (_ for _ in ()).throw(OSError("no browser")))
    else:
        monkeypatch.setattr(control_panel.webbrowser, "open", lambda url: False)
    control_panel.ControlPanel.open_report(panel)
    assert errors and "index.html" in errors[0][1]
