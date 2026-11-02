import importlib.util
import json
from pathlib import Path


def module():
    path = Path(__file__).resolve().parents[1] / "deployment/create_report.py"
    spec = importlib.util.spec_from_file_location("external_report_test", path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def save_result(base, name, stamp="20260101", passed=True):
    root = base / "results" / name / (name + "_" + stamp)
    root.mkdir(parents=True)
    (base / "main").mkdir(exist_ok=True)
    (root / "validation.json").write_text(
        json.dumps({"passed": passed, "extra": "</script><script>bad</script>"})
    )
    (root / "navigation_position_sar.csv").write_text(
        "time_boot_s,x_sar_m,y_sar_m,z_sar_m,vx_sar_mps,vy_sar_mps,vz_sar_mps\n"
        "0,0,0,0,1,0,0\n0.02,1,0,2,1,0,0\n"
    )
    if name.endswith("_route"):
        (root / "route_navigation_sar.csv").write_text(
            "time_boot_s,x_sar_m,y_sar_m,z_sar_m\n0,0,0,300\n0.02,1,0,300\n",
            encoding="utf-8",
        )
    return root


def test_report_retains_real_samples_and_offline_links(tmp_path):
    m = module()
    directory = save_result(tmp_path, "sitl")
    save_result(tmp_path, "xtdrone")
    original = (directory / "navigation_position_sar.csv").read_bytes()
    root = m.begin(tmp_path)
    page = m.create_report(root, tmp_path)
    text = page.read_text(encoding="utf-8")
    assert "可拖动三维航迹" in text
    assert "../../sitl/sitl_20260101/validation.json" in text
    assert "</script><script>bad</script>" not in text
    assert "__PAYLOAD__" not in text and "__SCRIPT__" not in text
    summary = json.loads((root / "report_summary.json").read_text(encoding="utf-8"))
    assert summary["status"] == "passed"
    assert m.packet(directory, "sitl")["pose"][1] == [0.02, 1, 0, 2, 1, 0, 0]
    assert (directory / "navigation_position_sar.csv").read_bytes() == original


def test_current_run_never_reuses_old_passed_results(tmp_path):
    m = module()
    save_result(tmp_path, "sitl")
    save_result(tmp_path, "xtdrone")
    root = m.begin(tmp_path)
    m.create_report(root, tmp_path, {"sitl": "1", "xtdrone": "not_run"})
    summary = json.loads((root / "report_summary.json").read_text(encoding="utf-8"))
    assert summary["status"] == "failed"
    assert all(stage["directory"] is None for stage in summary["stages"])
    text = (root / "index.html").read_text(encoding="utf-8")
    assert "../../sitl/" not in text


def test_entry_failure_not_promoted_by_saved_true_flag(tmp_path):
    m = module()
    (tmp_path / "main").mkdir()
    root = m.begin(tmp_path)
    save_result(tmp_path, "sitl", "20260102")
    m.create_report(root, tmp_path, {"sitl": "1", "xtdrone": "not_run"})
    summary = json.loads((root / "report_summary.json").read_text(encoding="utf-8"))
    assert summary["stages"][0]["status"] == "failed"
    assert summary["stages"][1]["status"] == "partial"


def test_route_report_uses_only_new_route_results(tmp_path):
    m = module()
    save_result(tmp_path, "sitl")
    save_result(tmp_path, "xtdrone")
    save_result(tmp_path, "sitl_route")
    save_result(tmp_path, "xtdrone_route")
    root = m.begin(tmp_path, route=True)
    assert root.parent.name == "external_route_review"
    save_result(tmp_path, "sitl_route", "20260102")
    save_result(tmp_path, "xtdrone_route", "20260102")
    page = m.create_report(root, tmp_path, {"sitl": "0", "xtdrone": "0"}, route=True)
    text = page.read_text(encoding="utf-8")
    summary = json.loads((root / "report_summary.json").read_text(encoding="utf-8"))
    assert summary["status"] == "passed"
    assert "完整采集航段" in text
    assert "仅水平采集段" in text
    assert "route-track" in text
    assert "route-detail-track" in text
    assert "显示比例放大" in text
    assert m.packet(tmp_path / "results/sitl_route/sitl_route_20260102", "sitl", route=True)["route_pose"] == [
        [0, 0, 0, 300], [0.02, 1, 0, 300]
    ]
    assert "../../sitl_route/sitl_route_20260102/validation.json" in text
    assert "../../sitl/sitl_20260101/validation.json" not in text


def test_long_log_display_uses_only_original_samples():
    m = module()
    source = [[index, index * 2] for index in range(10000)]
    shown = m.display_rows(source)
    assert len(shown) <= 4000
    assert shown[0] is source[0] and shown[-1] is source[-1]
    source_ids = {id(item) for item in source}
    assert all(id(row) in source_ids for row in shown)
