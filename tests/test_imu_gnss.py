import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from uavsim.config.loader import load_config
from uavsim.navigation.imu_gnss import simulate_errors, transition


def scenario(seconds=10):
    cfg = load_config('configs/navigation_x1_5_rtk.yaml')
    t = np.arange(0, seconds + .05, .1)
    truth = {'time_s': t, 'acceleration_N_mps2': np.zeros((len(t), 3))}
    rotations = np.repeat(np.eye(3)[None], len(t), axis=0)
    return cfg, truth, rotations


def test_constant_acceleration_bias_analytic_propagation():
    f = np.zeros((15, 15))
    f[:3, 3:6] = np.eye(3)
    f[3:6, 9:12] = np.eye(3)
    error = np.zeros(15)
    error[9:12] = [.01, -.02, .03]
    final = transition(f, 10) @ error
    assert_allclose(final[:3], [.5, -1., 1.5])
    assert_allclose(final[3:6], [.1, -.2, .3])


def test_zero_error_and_reproducibility():
    cfg, truth, rotations = scenario()
    a = simulate_errors(cfg, truth, rotations, 17)
    b = simulate_errors(cfg, truth, rotations, 17)
    for first, second in zip(a, b):
        assert_array_equal(first, second)
    c = cfg.disturbances.imu_gnss
    c.gnss_enabled = False
    c.initial_position_std_m = c.initial_velocity_std_mps = 0
    c.initial_tilt_std_deg = c.initial_heading_std_deg = 0
    c.accelerometer_bias_std_mps2 = c.accelerometer_vrw_mps_sqrt_hour = (0.,)*3
    c.gyro_bias_std_degph = c.gyro_arw_deg_sqrt_hour = (0.,)*3
    result = simulate_errors(cfg, truth, rotations, 17)
    assert_array_equal(result[0], 0)
    assert_array_equal(result[3], 0)


def test_gnss_reduces_ensemble_position_drift_and_tracks_update_rate():
    cfg, truth, rotations = scenario(20)
    aided, unaided = [], []
    for seed in range(8):
        cfg.disturbances.imu_gnss.gnss_enabled = True
        errors, _, _, sigmas, updates = simulate_errors(cfg, truth, rotations, seed)
        aided.append(np.linalg.norm(errors[-1, :3]))
        assert np.count_nonzero(updates) == 101
        assert np.isfinite(sigmas).all() and (sigmas >= 0).all()
        cfg.disturbances.imu_gnss.gnss_enabled = False
        unaided.append(np.linalg.norm(simulate_errors(cfg, truth, rotations, seed)[0][-1, :3]))
    assert np.mean(np.square(aided)) < np.mean(np.square(unaided)) / 10
