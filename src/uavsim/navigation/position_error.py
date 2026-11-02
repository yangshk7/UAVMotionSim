import numpy as np

from uavsim.disturbances.ou_process import ou_process
from uavsim.disturbances.presets import NAVIGATION_M


def navigation_estimate(config, time_s: np.ndarray, truth_position_sar_m: np.ndarray, pass_seed: int) -> dict:
    """仅输出位置估计；独立随机流，无任何动力学/飞控写入通路。"""
    cfg = config.disturbances.navigation_position_error
    error = np.zeros_like(truth_position_sar_m)
    bias = np.zeros_like(error)
    noise = np.zeros_like(error)
    if cfg.enabled or config.disturbances.all_enabled:
        sigma = cfg.bias_std_sar_m if cfg.bias_std_sar_m is not None else NAVIGATION_M[cfg.level]
        for axis in range(3):
            seed = np.random.SeedSequence([config.base_seed, pass_seed, 3, axis, cfg.random_seed or 0])
            bias_seed, noise_seed = seed.spawn(2)
            bias[:, axis] = sigma[axis] * ou_process(
                len(time_s),
                1 / config.simulation.navigation_rate_hz,
                cfg.correlation_time_s,
                np.random.default_rng(bias_seed),
            )
            noise[:, axis] = cfg.noise_std_sar_m[axis] * np.random.default_rng(noise_seed).normal(
                size=len(time_s)
            )
        error = bias + noise
    return {
        "time_s": time_s,
        "position_sar_m": truth_position_sar_m + error,
        "error_sar_m": error,
        "bias_sar_m": bias,
        "noise_sar_m": noise,
        "frame": "SAR_X_forward_Y_right_Z_up",
    }
