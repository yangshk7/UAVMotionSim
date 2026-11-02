import numpy as np
import pytest
from numpy.testing import assert_allclose
from test_navigation import short_config

from uavsim.experiments.two_pass import run_experiment
from uavsim.sar_adapter.matlab_export import export_experiment, load_exported_pass


def test_mat_csv_round_trip_and_no_overwrite(tmp_path):
    cfg = short_config()
    result = run_experiment(cfg, 1, {"workers": 1})
    export_experiment(result, tmp_path / "export")
    restored = load_exported_pass(tmp_path / "export/uav_motion_pass1.mat")
    assert_allclose(restored["truth_sar"]["q_NB_wxyz"], result["pass1"]["truth_sar"]["q_NB_wxyz"])
    assert_allclose(
        restored["truth_sar"]["apc_position_sar_m"], result["pass1"]["truth_sar"]["apc_position_sar_m"]
    )
    csv = np.genfromtxt(tmp_path / "export/uav_motion_pass1.csv", delimiter=",", names=True)
    assert_allclose(csv["time_s"], result["pass1"]["truth_sar"]["time_s"])
    with pytest.raises(FileExistsError):
        export_experiment(result, tmp_path / "export")
