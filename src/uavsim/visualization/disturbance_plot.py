from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from uavsim.visualization.chinese_style import use_chinese_font


def plot_disturbances(result: dict, root: str | Path) -> Path:
    use_chinese_font()
    inputs = result["pass1"]["disturbance_inputs"]
    channels = [key for key in ("x", "y", "z", "roll", "pitch", "yaw") if key in inputs]
    fig, axes = plt.subplots(
        max(1, len(channels)),
        1,
        figsize=(11, max(3, 2 * len(channels))),
        sharex=True,
        layout="constrained",
        squeeze=False,
    )
    if not channels:
        axes[0, 0].text(
            0.5,
            0.5,
            "A/B 扰动全部关闭",
            ha="center",
            va="center",
            transform=axes[0, 0].transAxes,
        )
    for ax, name in zip(axes[:, 0], channels):
        data = inputs[name]
        ax.plot(inputs["time_s"], data["values"])
        axis = {"x": "X 平移", "y": "Y 平移", "z": "Z 平移", "roll": "滚转", "pitch": "俯仰", "yaw": "偏航"}[
            name
        ]
        ax.set_ylabel(f"{axis} ({data['unit']})")
        ax.grid(True, alpha=0.3)
    axes[-1, 0].set_xlabel("采集时间 (s)")
    path = Path(root) / "disturbance_inputs.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path
