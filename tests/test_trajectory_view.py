"""三维视图只使用已导出导航位置，坐标和单位不能混淆。"""

import importlib.util
from pathlib import Path

import numpy as np
import pytest
from matplotlib.figure import Figure
from numpy.testing import assert_array_equal


def load_view():
    path = Path(__file__).resolve().parents[1] / "main" / "trajectory_view.py"
    spec = importlib.util.spec_from_file_location("trajectory_view_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_navigation_3d_tracks_use_exported_cg_and_apc_positions():
    dtype = [
        (name, "f8")
        for name in ("x_sar_m", "y_sar_m", "z_sar_m", "apc_x_sar_m", "apc_y_sar_m", "apc_z_sar_m")
    ]
    truth = np.zeros(3, dtype=dtype)
    navigation = np.zeros(3, dtype=dtype)
    for array, offset in ((truth, 0.0), (navigation, 0.2)):
        array["x_sar_m"] = [0, 1, 2]
        array["y_sar_m"] = offset
        array["z_sar_m"] = 300
        array["apc_x_sar_m"] = [0.1, 1.1, 2.1]
        array["apc_y_sar_m"] = offset + 0.1
        array["apc_z_sar_m"] = 299.8
    original = navigation.copy()
    axes = Figure().add_subplot(111, projection="3d")
    view = load_view()
    view.draw_trajectory(axes, navigation, truth, "cg", False)
    assert_array_equal(axes.lines[0].get_data_3d()[0], truth["x_sar_m"])
    assert_array_equal(axes.lines[1].get_data_3d()[1], navigation["y_sar_m"])
    assert "导航估计" in axes.get_title()
    view.draw_trajectory(axes, navigation, truth, "apc", True)
    assert_array_equal(axes.lines[1].get_data_3d()[2], navigation["apc_z_sar_m"])
    assert_array_equal(navigation, original)
    with pytest.raises(ValueError, match="cg 或 apc"):
        view.draw_trajectory(axes, navigation, truth, "invalid", False)
