"""高频外力下检查积分一致性，同时保留瞬时有限差分作为诊断量。"""

from uavsim.analysis.metrics import state_metrics
from uavsim.config.loader import load_config
from uavsim.config.validator import Config
from uavsim.dynamics.simulator import simulate_pass


def test_force_run_passes_integral_check_but_biased_acceleration_fails():
    values = load_config("configs/three_axis_force_6n.yaml").model_dump()
    values["simulation"].update(
        dynamics_rate_hz=800, pre_stabilization_s=0.1, acquisition_duration_s=8.0, duration_mode="fixed"
    )
    config = Config.model_validate(values)
    result = simulate_pass(config, config.pass_seeds[0])
    metrics = state_metrics(result, config)
    assert metrics["velocity_derivative_rms_mps2"] > 1e-3
    assert metrics["velocity_integral_rms_mps2"] < 1e-3
    assert metrics["kinematics_passed"]

    truth = result["truth_full"]
    altered = {
        **result,
        "truth_full": {**truth, "acceleration_N_mps2": truth["acceleration_N_mps2"] + 0.1},
    }
    bad = state_metrics(altered, config)
    assert bad["velocity_integral_rms_mps2"] > 0.01
    assert not bad["kinematics_passed"]
