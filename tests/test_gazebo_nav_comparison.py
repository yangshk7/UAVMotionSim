import csv
import importlib.util
import math
from pathlib import Path

import pytest


def load_module():
    path = Path(__file__).resolve().parents[1] / "deployment/compare_gazebo_nav.py"
    spec = importlib.util.spec_from_file_location("gazebo_nav_comparison", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_csv(path, fields, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(rows)


def test_matching_preserves_origin_offset_and_enu_axes(tmp_path):
    write_csv(
        tmp_path / "gazebo_truth_enu.csv",
        ["sim_time_s", "receipt_monotonic_s", "x_enu_m", "y_enu_m", "z_up_m"],
        [[i * 0.01, i * 0.01, 2, i * 0.1, 300] for i in range(110)],
    )
    write_csv(
        tmp_path / "navigation_position_sar.csv",
        ["timestamp_s", "receipt_monotonic_s", "x_sar_m", "y_sar_m", "z_sar_m", "xy_valid", "z_valid"],
        [[i * 0.01, i * 0.01 + 0.002, i * 0.1 + 0.5, 2 - 0.25, 299, "True", "True"] for i in range(110)],
    )
    write_csv(tmp_path / "route_navigation_sar.csv", ["timestamp_s"], [[0], [1.09]])
    output = tmp_path / "aligned.csv"
    result = load_module().compare(tmp_path, output)
    assert result["samples"] == 110
    assert result["navigation_minus_gazebo_mean_sar_m"] == pytest.approx([0.5, -0.25, -1.0])
    assert result["receipt_time_gap_max_s"] == pytest.approx(0.002)
    with output.open(newline="", encoding="utf-8") as stream:
        aligned = list(csv.DictReader(stream))
    assert len(aligned) == 110
    assert float(aligned[10]["truth_x_sar_m"]) == pytest.approx(1.0)
    assert float(aligned[10]["nav_minus_truth_z_m"]) == pytest.approx(-1.0)


def test_matching_rejects_unsorted_truth_timestamps(tmp_path):
    write_csv(tmp_path / "gazebo_truth_enu.csv", ["receipt_monotonic_s"], [[1], [0]])
    write_csv(tmp_path / "navigation_position_sar.csv", ["timestamp_s"], [[0]])
    write_csv(tmp_path / "route_navigation_sar.csv", ["timestamp_s"], [[0], [1]])
    with pytest.raises(ValueError, match="not strictly increasing"):
        load_module().compare(tmp_path)


def test_matching_excludes_px4_clock_resync_outlier(tmp_path):
    write_csv(
        tmp_path / "gazebo_truth_enu.csv",
        ["sim_time_s", "receipt_monotonic_s", "x_enu_m", "y_enu_m", "z_up_m",
         "qw", "qx", "qy", "qz"],
        [[i * 0.01, i * 0.01, 0, i * 0.1, 300, 1, 0, 0, 0] for i in range(110)],
    )
    write_csv(
        tmp_path / "navigation_position_sar.csv",
        ["timestamp_s", "receipt_monotonic_s", "x_sar_m", "y_sar_m", "z_sar_m", "xy_valid", "z_valid"],
        [[i * 0.01, i * 0.01, i * 0.1, 0, 300, "True", "True"] for i in range(110)]
        + [[-1e9, 1.1, 0, 0, 0, "True", "True"]],
    )
    write_csv(
        tmp_path / "navigation_attitude.csv",
        ["timestamp_s", "qw_nb", "qx_nb", "qy_nb", "qz_nb"],
        [[i * 0.01, 1, 0, 0, 0] for i in range(55)]
        + [[-1e9, 1, 0, 0, 0]]
        + [[i * 0.01, 1, 0, 0, 0] for i in range(55, 110)],
    )
    write_csv(tmp_path / "route_navigation_sar.csv", ["timestamp_s"], [[0], [1.09]])
    result = load_module().compare(tmp_path)
    assert result["samples"] == 110
    assert result["navigation_timestamp_outlier_samples"] == 1
    assert result["attitude_timestamp_outlier_samples"] == 1


def test_identity_gazebo_pose_maps_to_ned_yaw_ninety_degrees():
    module = load_module()
    expected = (math.sqrt(0.5), 0, 0, math.sqrt(0.5))
    converted = module.gazebo_to_ned_frd((1, 0, 0, 0))
    assert module.attitude_distance_deg(converted, expected) == pytest.approx(0)
    assert module.attitude_distance_deg(converted, tuple(-value for value in expected)) == pytest.approx(0)
