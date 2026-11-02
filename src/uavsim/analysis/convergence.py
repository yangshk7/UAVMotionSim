from pathlib import Path

import numpy as np

from uavsim.config.validator import Config
from uavsim.dynamics.simulator import simulate_pass
from uavsim.runtime.progress import convergence_progress
from uavsim.visualization.chinese_style import use_chinese_font

from .metrics import rms


def convergence_study(
    config: Config, root: str | Path, duration_s: float | None = None, wavelength_m: float = 299792458 / 1.2e9
) -> dict:
    """1600/3200/6400 Hz 公共 6400 Hz OU 样条输入，比较同一 SAR 脉冲网格。

    使用独立、显式的 2 N 外力和 0.1 N m 外力矩作为数值试验激励，非实机标定。
    wavelength_m 只用于报告相位误差，不修改现有雷达配置。
    """
    import matplotlib.pyplot as plt

    use_chinese_font()

    base = config.model_dump()
    base["disturbances"]["all_enabled"] = False
    base["disturbances"]["input_rate_hz"] = 6400
    for group in ["translation", "attitude"]:
        for key in base["disturbances"][group]:
            base["disturbances"][group][key]["enabled"] = False
    base["disturbances"]["translation"]["y"].update(
        enabled=True, injection_mode="dynamics", unit="N", amplitude=2.0
    )
    base["disturbances"]["attitude"]["roll"].update(
        enabled=True, injection_mode="dynamics", unit="N*m", amplitude=0.1
    )
    if duration_s is not None:
        base["simulation"]["acquisition_duration_s"] = duration_s
    solutions = {}
    for rate in [1600, 3200, 6400]:
        base["simulation"]["dynamics_rate_hz"] = rate
        with convergence_progress(rate):
            run = simulate_pass(Config.model_validate(base), 98231)
        solutions[rate] = run["truth_sar"]
    ref = solutions[6400]
    h = config.trajectory.apc_height_m
    target = np.array([0.0, h / np.tan(np.deg2rad(config.antenna.look_angle_deg)), 0.0])
    ref_range = np.linalg.norm(ref["apc_position_sar_m"] - target, axis=1)
    comparisons = {}
    fig, ax = plt.subplots(figsize=(10, 4), layout="constrained")
    for rate in [1600, 3200]:
        state = solutions[rate]
        delta = np.linalg.norm(state["apc_position_sar_m"] - target, axis=1) - ref_range
        comparison = {
            field + "_rms": rms(state[field] - ref[field])
            for field in [
                "position_N_m",
                "velocity_N_mps",
                "acceleration_N_mps2",
                "omega_B_radps",
                "apc_position_sar_m",
            ]
        }
        # 姿态误差用旋转夹角，不用四元数分量相减。
        relative = np.swapaxes(state["rotation_NB"], -1, -2) @ ref["rotation_NB"]
        from scipy.spatial.transform import Rotation

        comparison["attitude_geodesic_rms_rad"] = rms(Rotation.from_matrix(relative).magnitude())
        comparison["slant_range_rms_m"] = rms(delta)
        comparison["slant_range_max_m"] = float(np.max(np.abs(delta)))
        comparison["two_way_phase_rms_rad"] = rms(4 * np.pi * delta / wavelength_m)
        comparisons[str(rate)] = comparison
        ax.plot(state["time_s"], delta * 1000, label=f"{rate} 对比 6400 Hz")
    ax.set(
        xlabel="采集时间 (s)",
        ylabel="斜距差 (mm)",
        title="各积分频率使用同一连续扰动输入",
    )
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.savefig(Path(root) / "numerical_convergence.png", dpi=160)
    plt.close(fig)
    return {
        "rates_hz": [1600, 3200, 6400],
        "input_grid_hz": 6400,
        "seed": 98231,
        "target_sar_m": target.tolist(),
        "duration_s": float(ref["time_s"][-1]),
        "wavelength_m": wavelength_m,
        "stimulus": "FRD y force std 2 N; roll torque std 0.1 N*m; same continuous OU splines",
        "comparisons_to_6400_hz": comparisons,
        "limit_slant_range_rms_m": 0.001,
        "passed": comparisons["3200"]["slant_range_rms_m"] <= 0.001,
    }
