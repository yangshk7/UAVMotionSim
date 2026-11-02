import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from uavsim.config.validator import Config
from uavsim.dynamics.simulator import simulate_pass
from uavsim.sar_adapter.pulse_sync import sample_indices


def short_config():
    return Config.model_validate({"simulation": {"pre_stabilization_s": 0.1, "acquisition_duration_s": 0.2}})


def test_default_sample_counts():
    ids = sample_indices(Config())
    assert len(ids["sar"]) == 16385
    assert len(ids["navigation"]) == 2049
    assert len(ids["acquisition"]) == 65537
    assert ids["sar"][0] == 16000
    assert ids["sar"][-1] + 1 == 81537
    assert_array_equal(np.diff(ids["sar"]), 4)


def test_navigation_never_changes_truth():
    config = short_config()
    a = simulate_pass(config, 123)
    data = config.model_dump()
    data["disturbances"]["navigation_position_error"]["enabled"] = True
    b = simulate_pass(Config.model_validate(data), 123)
    for key in ["position_N_m", "velocity_N_mps", "q_NB_wxyz", "omega_B_radps", "acceleration_N_mps2"]:
        assert_array_equal(a["truth_full"][key], b["truth_full"][key])
    assert np.linalg.norm(b["navigation"]["error_sar_m"]) > 0
    assert_allclose(b["navigation_sar"]["position_sar_m"][::8], b["navigation"]["position_sar_m"], atol=1e-12)
    assert_array_equal(a["truth_sar"]["time_s"], np.arange(161) / 800)


def test_pass_reproducibility_and_decoupling():
    config = short_config()
    a = simulate_pass(config, 123)
    b = simulate_pass(config, 123)
    assert_array_equal(a["truth_full"]["position_N_m"], b["truth_full"]["position_N_m"])
    assert a["sar_geometry"]["physical_truth"] is False
    assert_allclose(a["sar_geometry"]["apc_position_sar_m"], a["reference"]["apc_position_sar_m"], atol=1e-10)
