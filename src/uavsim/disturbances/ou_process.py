import numpy as np
from scipy.signal import lfilter


def ou_process(count: int, dt_s: float, tau_s: float, rng: np.random.Generator) -> np.ndarray:
    """精确离散 OU，均值 0、稳态方差 1；初值从稳态分布抽样。"""
    if count < 1 or dt_s <= 0 or tau_s <= 0:
        raise ValueError("Positive count, dt and tau required")
    a = np.exp(-dt_s / tau_s)
    innovation = rng.normal(size=count) * np.sqrt(-np.expm1(-2 * dt_s / tau_s))
    innovation[0] = rng.normal()
    return lfilter([1.0], [1.0, -a], innovation)


def dual_ou(
    count: int,
    dt_s: float,
    tau_slow_s: float,
    tau_fast_s: float,
    slow_fraction: float,
    seed: np.random.SeedSequence,
) -> np.ndarray:
    slow_seed, fast_seed = seed.spawn(2)
    slow = ou_process(count, dt_s, tau_slow_s, np.random.default_rng(slow_seed))
    fast = ou_process(count, dt_s, tau_fast_s, np.random.default_rng(fast_seed))
    return np.sqrt(slow_fraction) * slow + np.sqrt(1 - slow_fraction) * fast
