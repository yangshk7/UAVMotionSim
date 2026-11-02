"""公共仿真时钟上的真值/导航配对边界检查。"""

import importlib.util
from pathlib import Path

import numpy as np
import pytest


def _load():
    script = Path(__file__).resolve().parents[1] / "deployment/validate_clock_aligned.py"
    spec = importlib.util.spec_from_file_location("validate_clock_aligned", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write(path: Path, time: np.ndarray):
    values = np.column_stack((time, 10 * time, np.zeros((len(time), 2)),
                              np.full(len(time), 10), np.zeros((len(time), 2)),
                              np.ones(len(time)), np.zeros((len(time), 3))))
    header = ("time_s,cg_x_sar_m,cg_y_sar_m,cg_z_sar_m,cg_vx_sar_mps,cg_vy_sar_mps,"
              "cg_vz_sar_mps,qw_nb,qx_nb,qy_nb,qz_nb")
    np.savetxt(path, values, delimiter=",", header=header, comments="")


@pytest.mark.parametrize("truth_rate,exact_count", [(1000, 101), (400, 26)])
def test_shared_clock_pairs_native_truth_and_100hz_navigation(tmp_path, truth_rate, exact_count):
    module = _load()
    truth_time = np.arange(truth_rate + 1) / truth_rate
    nav_time = np.arange(101) / 100
    truth_name = "truth_1000hz.csv" if truth_rate == 1000 else "truth_native.csv"
    _write(tmp_path / truth_name, truth_time)
    _write(tmp_path / "navigation_100hz.csv", nav_time)
    result = module.validate(tmp_path, truth_name)
    assert result["paired_samples"] == 101
    assert result["exact_truth_time_matches"] >= exact_count
    assert result["maximum_truth_interpolation_gap_s"] <= 1 / truth_rate + 1e-12
    np.testing.assert_allclose(result["position_nav_minus_truth_rms_sar_m"], [0, 0, 0], atol=1e-10)
    with pytest.raises(FileExistsError):
        module.validate(tmp_path, truth_name)
