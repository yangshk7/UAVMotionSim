from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from uavsim.visualization.chinese_style import use_chinese_font


def plot_trajectories(result: dict, root: str | Path) -> Path:
    use_chinese_font()
    fig = plt.figure(figsize=(11, 7), layout="constrained")
    ax = fig.add_subplot(111, projection="3d")
    for name, data in result.items():
        if name.startswith("pass"):
            p = data["truth_sar"]["apc_position_sar_m"]
            ax.plot(*p.T, label=name.replace("pass", "航次 ") + " APC 真值", lw=1)
            nav = data["navigation"]["apc_position_sar_m"]
            ax.plot(*nav.T, "--", label=name.replace("pass", "航次 ") + " APC 导航估计", lw=1)
    p = result["pass1"]["reference"]["apc_position_sar_m"]
    ax.plot(*p.T, "k--", label="名义 APC 航迹")
    ax.set(
        xlabel="SAR 前向 X (m)",
        ylabel="SAR 右向 Y (m)",
        zlabel="SAR 向上 Z (m)",
        title="六旋翼天线相位中心三维航迹",
    )
    ax.legend()
    fig.text(0.02, 0.02, "各坐标轴独立缩放，用于显示小幅横向与高度偏差；非真实比例。", fontsize=9)
    path = Path(root) / "trajectory_3d.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path
