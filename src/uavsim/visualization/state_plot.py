from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from uavsim.visualization.chinese_style import use_chinese_font


def plot_states(result: dict, root: str | Path) -> list[Path]:
    use_chinese_font()
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    first = result["pass1"]
    truth = first["truth_sar"]
    t = truth["time_s"]
    fields = [
        ("velocity_sar_mps", "SAR 速度 (m/s)"),
        ("acceleration_sar_mps2", "SAR 加速度 (m/s²)"),
        ("euler_NB_rpy_rad", "NED 滚转/俯仰/偏航 (rad)"),
        ("omega_B_radps", "FRD 角速度 p/q/r (rad/s)"),
    ]
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True, layout="constrained")
    for ax, (field, label) in zip(axes, fields):
        ax.plot(t, truth[field])
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
        ax.legend(("X / 滚转 / p", "Y / 俯仰 / q", "Z / 偏航 / r"), loc="upper right", ncol=3)
    axes[-1].set_xlabel("采集时间 (s)")
    path = root / "motion_states.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True, layout="constrained")
    for name, data in result.items():
        if name.startswith("pass"):
            axes[0].plot(
                t,
                data["truth_sar"]["apc_position_sar_m"] - data["reference"]["apc_position_sar_m"],
                label=[name.replace("pass", "航次 ") + " " + axis for axis in "XYZ"],
            )
            nav = data["navigation"]
            axes[1].plot(
                nav["time_s"],
                nav["error_sar_m"],
                label=[name.replace("pass", "航次 ") + " " + axis for axis in "XYZ"],
            )
    for ax, label in zip(axes, ["APC 相对名义航迹偏差 (m)", "导航位置估计误差 (m)"]):
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
        ax.legend(ncol=3)
    axes[-1].set_xlabel("采集时间 (s)")
    navpath = root / "position_errors.png"
    fig.savefig(navpath, dpi=160)
    plt.close(fig)
    return [path, navpath]
