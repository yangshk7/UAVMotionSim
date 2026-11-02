import numpy as np


def sample_indices(config) -> dict:
    """通过整数步索引生成精确公共时轴，包含采集首末点。"""
    s = config.simulation
    pre = round(s.pre_stabilization_s * s.dynamics_rate_hz)
    active = round(s.acquisition_duration_s * s.dynamics_rate_hz)
    return {
        "sar": pre + np.arange(0, active + 1, s.dynamics_rate_hz // s.sar_rate_hz, dtype=np.int64),
        "navigation": pre
        + np.arange(0, active + 1, s.dynamics_rate_hz // s.navigation_rate_hz, dtype=np.int64),
        "acquisition": pre + np.arange(active + 1, dtype=np.int64),
    }
