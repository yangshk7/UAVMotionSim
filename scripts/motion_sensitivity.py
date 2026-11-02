"""同随机种子配对的单因素横向指令扰动试验；不调用 SAR 成像。"""

import argparse
import json
import math
import time
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
from matplotlib import pyplot as plt

from uavsim.config.loader import load_config
from uavsim.config.validator import Config
from uavsim.dynamics.simulator import simulate_pass
from uavsim.motion.segment import from_simulated_pass
from uavsim.visualization.chinese_style import use_chinese_font


def run_study(config_path: Path, output: Path, duration_s: float, amplitudes_m: tuple[float, ...],
              seeds: tuple[int, ...]) -> dict:
    """仅改变 SAR Y 向指令 OU 标准差；统计采样点管道直径，单位 m。"""
    if (not np.isfinite(duration_s) or duration_s <= 0 or not amplitudes_m or not seeds
            or any(not np.isfinite(x) or x < 0 for x in amplitudes_m)
            or len(set(amplitudes_m)) != len(amplitudes_m) or len(set(seeds)) != len(seeds)):
        raise ValueError("Positive duration, unique nonnegative amplitudes and unique seeds required")
    base = load_config(config_path).model_dump()
    base["simulation"]["duration_mode"] = "fixed"
    base["simulation"]["acquisition_duration_s"] = duration_s
    base["output"].update(save_mat=False, save_csv=False, save_plots=False)
    base["disturbances"]["all_enabled"] = False
    for group in ("translation", "attitude"):
        for factor in base["disturbances"][group].values():
            factor["enabled"] = False
    for name in ("navigation_position_error", "navigation_velocity_error", "navigation_attitude_error"):
        base["disturbances"][name]["enabled"] = False
    rows = []
    for seed in seeds:
        for amplitude in amplitudes_m:
            data = json.loads(json.dumps(base))
            factor = data["disturbances"]["translation"]["y"]
            factor.update(enabled=amplitude > 0, injection_mode="command", profile="ou",
                          amplitude=amplitude, unit="m", random_seed=seed)
            config = Config.model_validate(data)
            started = time.perf_counter()
            run = simulate_pass(config, seed)
            segment = from_simulated_pass(run, config)
            position = segment.truth.apc_position_sar_m
            velocity = segment.truth.velocity_sar_mps
            radial = np.hypot(position[:, 1], position[:, 2] - config.trajectory.apc_height_m)
            p95_index = math.ceil(0.95 * len(radial)) - 1
            speed = np.linalg.norm(velocity, axis=1)
            rows.append({
                "seed": seed,
                "command_y_ou_std_m": amplitude,
                "tube_d95_m": float(2 * np.partition(radial, p95_index)[p95_index]),
                "tube_dall_m": float(2 * np.max(radial)),
                "y_rms_m": float(np.sqrt(np.mean(position[:, 1] ** 2))),
                "speed_min_mps": float(np.min(speed)),
                "speed_max_mps": float(np.max(speed)),
                "speed_within_10pct_fraction": float(np.mean(
                    (speed >= 0.9 * config.trajectory.speed_mps)
                    & (speed <= 1.1 * config.trajectory.speed_mps))),
                "samples": len(segment.truth.time_s),
                "elapsed_s": time.perf_counter() - started,
            })
    summary = []
    for amplitude in amplitudes_m:
        selected = [row for row in rows if row["command_y_ou_std_m"] == amplitude]
        values = np.array([row["tube_d95_m"] for row in selected])
        summary.append({"command_y_ou_std_m": amplitude, "tube_d95_mean_m": float(np.mean(values)),
                        "tube_d95_std_m": float(np.std(values, ddof=1)) if len(values) > 1 else None,
                        "repetitions": len(values)})
    report = {
        "design": "one_factor_paired_seed",
        "source_config": str(config_path.resolve()),
        "duration_s": duration_s,
        "pre_stabilization_s": base["simulation"]["pre_stabilization_s"],
        "backend": base["runtime"]["backend"],
        "amplitudes_m": amplitudes_m,
        "seeds": seeds,
        "baseline_config": base,
        "tube_definition": "D95=2*nearest-rank-95th-percentile(sqrt(y_APC^2+(z_APC-z_target)^2)); native-rate sampled truth only",
        "interpretation": "Engineering simulation response, not a calibrated real-flight mapping or a monotonicity guarantee",
        "runs": rows,
        "summary": summary,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir(exist_ok=False)
    (output / "sensitivity.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    use_chinese_font()
    fig, ax = plt.subplots(figsize=(8, 5))
    x = np.array(amplitudes_m)
    y = np.array([item["tube_d95_mean_m"] for item in summary])
    spread = np.array([item["tube_d95_std_m"] or 0 for item in summary])
    ax.errorbar(x, y, yerr=spread, marker="o", capsize=4, label=f"{len(seeds)}次重复：均值 ± 标准差")
    for row in rows:
        ax.scatter(row["command_y_ou_std_m"], row["tube_d95_m"], alpha=0.45, color="#e67e22")
    ax.set(xlabel="横向指令 OU 标准差 / m", ylabel="真值航迹管道 D95 / m",
           title="单因素扰动与航迹管道（工程仿真）")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / "sensitivity.png", dpi=160)
    plt.close(fig)
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/rotorpy_hex_x12.yaml"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=8.0)
    parser.add_argument("--amplitudes-m", type=float, nargs="+", default=(0.0, 0.15, 0.30))
    parser.add_argument("--seeds", type=int, nargs="+", default=(101, 102, 103))
    args = parser.parse_args()
    report = run_study(args.config, args.output, args.duration_s, tuple(args.amplitudes_m), tuple(args.seeds))
    print(json.dumps(report["summary"], ensure_ascii=False))


if __name__ == "__main__":
    main()
