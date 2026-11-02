import numpy as np
from numpy.testing import assert_allclose, assert_array_equal
from scipy.interpolate import CubicSpline
from scipy.signal import welch

from uavsim.config.loader import load_config
from uavsim.config.validator import Config
from uavsim.disturbances.injection_manager import DisturbanceInputs
from uavsim.disturbances.ou_process import dual_ou, ou_process
from uavsim.disturbances.presets import factor_scale
from uavsim.disturbances.translation import translation_command


def test_ou_stationary_statistics():
    # 20,000 s，400,000 样本；验证长序列，不用单次孔径声称标定。
    dt = 0.05
    tau = 2.5
    x = ou_process(400000, dt, tau, np.random.default_rng(123))
    assert abs(x.mean()) < 0.04
    assert abs(x.var() - 1) < 0.06
    lag = round(1 / dt)
    corr = np.corrcoef(x[:-lag], x[lag:])[0, 1]
    assert abs(corr - np.exp(-1 / tau)) < 0.04
    f, psd = welch(x, fs=1 / dt, nperseg=8192)
    assert abs(np.trapezoid(psd, f) - 1) < 0.08
    assert psd[(f > 0.02) & (f < 0.05)].mean() > psd[(f > 3) & (f < 4)].mean() * 30


def test_dual_ou_reproducibility_independence():
    def generate(seed):
        return dual_ou(100000, 0.05, 2.5, 0.35, 0.7, np.random.SeedSequence(seed))

    x = generate(17)
    assert_array_equal(x, generate(17))
    assert abs(np.corrcoef(x, generate(18))[0, 1]) < 0.08
    lag = 20
    target = 0.7 * np.exp(-1 / 2.5) + 0.3 * np.exp(-1 / 0.35)
    assert abs(np.corrcoef(x[:-lag], x[lag:])[0, 1] - target) < 0.05


def test_channel_independence_and_units():
    cfg = Config.model_validate({"disturbances": {"translation": {"x": {"enabled": True}, "y": {}, "z": {}}}})
    a = DisturbanceInputs(cfg, 11)
    second = cfg.model_dump()
    second["disturbances"]["attitude"]["roll"]["enabled"] = True
    b = DisturbanceInputs(Config.model_validate(second), 11)
    assert_array_equal(a.channels["x"]["values"], b.channels["x"]["values"])
    assert abs(np.corrcoef(b.channels["x"]["values"], b.channels["roll"]["values"])[0, 1]) < 0.8
    t = np.array([-0.1, 0.0, 0.1, 1.0])
    force, torque = a.dynamics_arrays(t)
    assert_allclose(force, 0)
    assert_allclose(torque, 0)
    assert_allclose(a.command_arrays(t)["position_offset_N_m"][:2], 0, atol=1e-12)


def test_three_position_target_disturbances_use_the_same_units_and_no_integration():
    spline = CubicSpline([0.0, 1.0, 2.0], [0.15, 0.15, 0.15])
    position, velocity, acceleration = translation_command([spline] * 3, np.array([0.0, 1.0, 2.0]))
    assert_allclose(position, 0.15)
    assert_allclose(velocity, 0.0, atol=1e-12)
    assert_allclose(acceleration, 0.0, atol=1e-12)


def test_default_position_and_optional_force_configurations():
    for path, strength in [
        ("configs/disturbance_small.yaml", 0.05),
        ("main/motion_config.yaml", 0.15),
        ("configs/disturbance_large.yaml", 0.30),
    ]:
        factors = load_config(path).disturbances.translation
        assert all(factors[axis].enabled and factors[axis].injection_mode == "command" for axis in "xyz")
        assert [factor_scale(factors[axis], "translation", index) for index, axis in enumerate("xyz")] == [
            (strength, "m")
        ] * 3
    forces = load_config("configs/three_axis_force_6n.yaml").disturbances.translation
    assert all(
        forces[axis].enabled
        and forces[axis].injection_mode == "dynamics"
        and forces[axis].amplitude == 6.0
        and forces[axis].unit == "N"
        for axis in "xyz"
    )


def test_dynamics_input_is_newton_not_position():
    cfg = Config.model_validate(
        {
            "disturbances": {
                "translation": {
                    "x": {
                        "enabled": True,
                        "injection_mode": "dynamics",
                        "unit": "N",
                        "amplitude": 2.0,
                        "profile": "constant",
                    },
                    "y": {},
                    "z": {},
                }
            }
        }
    )
    inputs = DisturbanceInputs(cfg, 1)
    force, _ = inputs.dynamics_arrays(np.array([1.0, 2.0]))
    assert_allclose(force, [[2, 0, 0], [2, 0, 0]])
    assert_allclose(inputs.command_arrays(np.array([1.0, 2.0]))["position_offset_N_m"], 0)
