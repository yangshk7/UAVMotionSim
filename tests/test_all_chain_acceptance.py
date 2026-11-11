"""All-chain orchestration and independent analytic comparison checks."""

import io
import json
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "main" / "internal"))
import all_chain_acceptance as all_chains
from control_jobs import create_job

from uavsim.config.validator import Config
from uavsim.motion.export import export_motion_segment
from uavsim.motion.segment import MotionSegment, MotionSeries


def test_all_acceptance_keeps_distinct_mode_and_snapshot(tmp_path):
    root = create_job(tmp_path, Config(), "all_acceptance")
    assert root.name.startswith("all_acceptance_")
    assert json.loads((root/"session.json").read_text())["mode"] == "all_acceptance"
    assert len(all_chains.CHAINS) == len(set(all_chains.CHAINS)) == 5
    assert "cpu" in all_chains.CHAINS and "auto" not in all_chains.CHAINS


def test_progress_does_not_jump_to_next_chains_100_percent():
    progress = all_chains.ChildProgress(True)
    assert progress.update({"pass_index": 0, "fraction": 1}) == 20
    assert progress.update({"pass_index": 1, "fraction": 1}) == 40
    assert progress.update({"percent": 60}) == 60
    assert progress.update({"phase": "convergence", "rate": 6400, "fraction": 1}) < 90
    assert progress.update({"percent": 100}) == 100
    assert progress.update({"percent": 20}) == 100


def test_only_explicitly_out_of_scope_external_bundle_is_excluded():
    stages = [*({"name": name, "status": "passed"} for name in all_chains.required_check_names("sitl")),
              {"name": "跨平台对照、OU统计、积分收敛及SAR端到端", "status": "skipped"},
              {"name": "unknown missing check", "status": "skipped"}]
    required = all_chains.chain_required_stages("sitl", {"stages": stages})
    assert len(required) == len(all_chains.required_check_names("sitl"))+1
    assert all_chains.acceptance_status(required) == "partial"
    assert all_chains.chain_required_stages("cpu", {"stages": stages})[-1]["name"] == "必需检查缺失"
    assert all_chains.acceptance_status(all_chains.chain_required_stages("sitl", {})) == "failed"


def test_orchestrator_continues_after_failure_without_promoting_it(tmp_path, monkeypatch):
    calls = []
    class Process:
        stdout = None
        def wait(self):
            return 0
        def poll(self):
            return 0
    def launch(project, child, mode, passes, matlab, executable):
        calls.append((child.name, mode, passes))
        if child.name == "matlab":
            raise RuntimeError("MATLAB unavailable")
        stages = [{"name": name, "status": "passed"} for name in all_chains.required_check_names(child.name)]
        if child.name in ("sitl", "px4"):
            stages.append({"name": "跨平台对照、OU统计、积分收敛及SAR端到端", "status": "skipped"})
        (child/"acceptance.json").write_text(json.dumps({"stages": stages}), encoding="utf-8")
        process = Process()
        process.stdout = io.StringIO("")
        return process
    monkeypatch.setattr(all_chains, "launch_job", launch)
    monkeypatch.setattr(all_chains, "compare_segments", lambda *_: {"status": "passed"})
    monkeypatch.setattr(all_chains, "compare_external", lambda *_: {"status": "passed"})
    monkeypatch.setattr(all_chains, "create_report", lambda root, *_: (root/"index.html").write_text("report"))
    result = all_chains.run_all_chains(tmp_path, Config())
    assert [row[0] for row in calls] == list(all_chains.CHAINS)
    assert all(row[2] == 2 for row in calls)
    assert result["status"] == "failed" and result["returncode"] == 1
    summary = json.loads((tmp_path/"acceptance.json").read_text(encoding="utf-8"))
    assert len(summary["chain_acceptance"]) == 5
    assert next(row for row in summary["chain_acceptance"] if row["backend"] == "matlab")["status"] == "failed"
    assert all(row["status"] == "passed" for row in summary["chain_acceptance"] if row["backend"] != "matlab")


def test_pair_metrics_match_known_translation_and_rotation(tmp_path):
    t = np.arange(201)/100
    directories = {}
    for index, name in enumerate(all_chains.CHAINS):
        directory = tmp_path/name
        directory.mkdir()
        directories[name] = directory
        p = np.column_stack((10*t+index, np.zeros_like(t), np.full_like(t, 300)))
        v = np.tile([10., 0, 0], (len(t), 1))
        q = np.tile(Rotation.from_euler("z", index, degrees=True).as_quat()[[3, 0, 1, 2]], (len(t), 1))
        series = MotionSeries(t, p, v, q, p)
        segment = MotionSegment(series, series, name, 100, 100, "exact_common_grid", "analytic test")
        for pass_name in ("pass1", "pass2"):
            export_motion_segment(segment, directory, pass_name)
    record = all_chains.compare_segments(directories, tmp_path)
    assert len(record["pairs"]) == 20
    first = record["pairs"][0]
    np.testing.assert_allclose(first["position_xyz_rms_m"], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(first["velocity_xyz_rms_mps"], [0, 0, 0], atol=1e-12)
    assert first["attitude_angle_rms_deg"] == pytest.approx(1)
