"""Independent checks for WGS84 conversion and receipt-time truth pairing."""

import csv
import importlib.util
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / "deployment" / "compare_sitl_truth.py"
SPEC = importlib.util.spec_from_file_location("compare_sitl_truth", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_csv(path, fieldnames, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(fieldnames)
        writer.writerows(rows)


def test_wgs84_local_tangent_at_equator():
    north, east, up = MODULE.local_neu(0.001, 0.001, 1, 0, 0, 0)
    assert north == pytest.approx(110.574, abs=0.002)
    assert east == pytest.approx(111.319, abs=0.002)
    assert up == 1


def test_relative_route_alignment_and_attitude(tmp_path):
    # Independent WGS84 meridian scale at the equator, rounded to 0.1 mm.
    metres_per_degree = 110574.2758
    n = 121
    write_csv(tmp_path / "navigation_position_sar.csv",
              ["time_boot_s", "x_sar_m", "y_sar_m", "z_sar_m", "vx_sar_mps",
               "vy_sar_mps", "vz_sar_mps", "receipt_monotonic_s"],
              [[i, 2 + i * 1e-6 * metres_per_degree, -1, 0, 0, 0, 0, 10 + i]
               for i in range(n)])
    write_csv(tmp_path / "navigation_attitude_ned.csv",
              ["time_boot_s", "roll_rad", "pitch_rad", "yaw_rad"],
              [[i, 0, 0, 0] for i in range(n)])
    write_csv(tmp_path / "simulation_truth_precise_wgs84.csv",
              ["receipt_monotonic_s", "lat_degE7", "lon_degE7", "roll_rad", "pitch_rad", "yaw_rad"],
              [[10 + i + 0.001, i * 10, 0, 0, 0, 0] for i in range(n)])
    write_csv(tmp_path / "simulation_truth_wgs84.csv",
              ["receipt_monotonic_s", "lat_raw_float", "lon_raw_float", "alt_m", "vn_mps",
               "ve_mps", "vd_mps", "qw", "qx", "qy", "qz"],
              [[10 + i - 0.001, 0, 0, 100, 0, 0, 0, 1, 0, 0, 0] for i in range(n)])
    write_csv(tmp_path / "route_navigation_sar.csv",
              ["time_boot_s", "x_sar_m", "y_sar_m", "z_sar_m"],
              [[i, 2 + i * 1e-6 * metres_per_degree, -1, 0] for i in range(1, n)])
    output = tmp_path / "aligned.csv"
    result = MODULE.compare(tmp_path, output)
    assert result["samples"] == n - 1
    assert result["receipt_time_gap_max_s"] == pytest.approx(0.001)
    assert result["relative_navigation_minus_truth_position_rms_sar_m"][0] < 0.001
    assert result["navigation_minus_truth_attitude_rms_deg"] == pytest.approx(0)
    with output.open(encoding="utf-8", newline="") as stream:
        assert len(list(csv.DictReader(stream))) == n - 1


def test_invalid_or_empty_truth_is_rejected(tmp_path):
    for filename in ("navigation_position_sar.csv", "navigation_attitude_ned.csv",
                     "simulation_truth_precise_wgs84.csv", "simulation_truth_wgs84.csv",
                     "route_navigation_sar.csv"):
        write_csv(tmp_path / filename, ["empty"], [])
    with pytest.raises(ValueError, match="nonempty"):
        MODULE.compare(tmp_path)
