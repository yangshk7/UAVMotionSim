import importlib.util
import json
from pathlib import Path

import numpy as np
from numpy.testing import assert_array_equal


def test_failed_route_details_include_actual_coverage(tmp_path, monkeypatch):
    import sys

    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/"main"/"internal"))
    sys.modules.pop("run", None)
    from run import collect_checks

    validation = {"passed": False,
                  "pass1": {"passed": True, "sar_samples": 88001, "navigation_samples": 11001},
                  "pass2": {"passed": False, "kinematics_passed": True,
                            "route_quality": {"passed": False, "target_route_length_m": 1000,
                                              "full_forward_distance_m": 797.7029, "speed_band_enforced": False}}}
    (tmp_path/"validation.json").write_text(json.dumps(validation), encoding="utf-8")
    stages, _ = collect_checks(tmp_path)
    assert stages[0]["status"] == "failed"
    assert "797.703" in stages[0]["detail"] and "1000" in stages[0]["detail"]
    assert "硬门槛未启用" in stages[0]["detail"]


def load_review_module():
    path = Path(__file__).resolve().parents[1] / "main" / "internal" / "review_report.py"
    spec = importlib.util.spec_from_file_location("manual_review_report", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_acceptance_status_never_promotes_missing_checks():
    module = load_review_module()
    assert module.acceptance_status([]) == "partial"
    assert module.acceptance_status([{"status": "passed"}, {"status": "skipped"}]) == "partial"
    assert module.acceptance_status([{"status": "passed"}, {"status": "failed"}]) == "failed"
    assert module.acceptance_status([{"status": "passed"}]) == "passed"


def test_offline_report_links_and_failed_status(tmp_path):
    module = load_review_module()
    data = tmp_path / "validate_test"
    data.mkdir()
    (data / "navigation_pass1.csv").write_text("time_s,x_sar_m\n0,0\n", encoding="utf-8")
    summary = {
        "stages": [{"name": "<unsafe>", "status": "failed", "detail": "test failure"}],
        "data_directory": str(data),
        "message": "<script>not executed</script>",
    }
    path = module.create_report(tmp_path, summary)
    html = path.read_text(encoding="utf-8")
    assert "本次流程存在失败项" in html
    assert "&lt;unsafe&gt;" in html and "<unsafe>" not in html
    assert "validate_test/navigation_pass1.csv" in html
    assert "<script>not executed</script>" not in html
    assert "本次未生成该图" in html
    assert "__PAYLOAD__" not in html
    figures = html.split('<section id="figures">', 1)[1]
    assert figures.index("双航次 APC 三维航迹") < figures.index("导航位置：真值与估计")


def test_cursor_samples_come_from_navigation_without_modifying_data():
    module = load_review_module()
    truth = {
        "time_s": np.array([0.0, 0.01]),
        "position_sar_m": np.array([[0.0, 1.0, 300.0], [0.1, 1.0, 300.0]]),
        "velocity_sar_mps": np.array([[10.0, 0.0, 0.0], [10.0, 0.0, 0.0]]),
        "euler_NB_rpy_deg": np.zeros((2, 3)),
        "q_NB_wxyz": np.array([[1.0, 0.0, 0.0, 0.0]] * 2),
        "apc_position_sar_m": np.array([[0.0, 1.1, 299.8], [0.1, 1.1, 299.8]]),
    }
    nav = {key: value.copy() for key, value in truth.items()}
    nav["position_sar_m"][:, 0] += 0.0123456789123
    original = nav["position_sar_m"].copy()
    payload = module.cursor_payload({"pass1": {"navigation": nav, "navigation_truth": truth}})
    assert payload["pass1"]["time_s"] == [0.0, 0.01]
    assert (
        payload["pass1"]["navigation"]["position_sar_m"][0][0]
        != payload["pass1"]["navigation_truth"]["position_sar_m"][0][0]
    )
    json.dumps(payload, allow_nan=False)
    assert_array_equal(original, nav["position_sar_m"])


def test_cross_comparison_failure_is_visible_and_skip_links_are_not_fabricated(tmp_path):
    module = load_review_module()
    summary = {
        'stages': [{'name': 'external', 'status': 'failed', 'detail': 'missing native log'}],
        'external_comparison': {'status': 'failed', 'detail': '<missing native log>'},
        'matlab_motion_comparison': {'status': 'skipped', 'detail': 'MATLAB skipped', 'cases': []},
    }
    page = module.create_report(tmp_path, summary).read_text(encoding='utf-8')
    assert '本次流程存在失败项' in page
    assert 'SITL / PX4 交叉对照' in page and '&lt;missing native log&gt;' in page
    assert 'MATLAB UAV / RotorPy 同工况对照' in page
    assert 'href="external_comparison/result.json"' not in page
    assert 'href="matlab_motion_comparison/result.json"' not in page
