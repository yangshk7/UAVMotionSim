from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from uavsim.visualization.chinese_style import use_chinese_font


def plot_navigation(result: dict, root: str | Path) -> Path:
    """导航输出的三类误差；数据来自实际生成的 100 Hz 记录。"""
    use_chinese_font()
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, layout="constrained")
    fields = [
        ("position_error_sar_m", 1.0, "SAR 位置估计误差 (m)"),
        ("velocity_error_sar_mps", 1.0, "SAR 速度估计误差 (m/s)"),
        ("attitude_error_rotvec_B_rad", 180 / np.pi, "机体系旋转向量误差 (度)"),
    ]
    for ax, (field, scale, label) in zip(axes, fields):
        for name, data in result.items():
            if name.startswith("pass"):
                nav = data["navigation"]
                ax.plot(
                    nav["time_s"],
                    nav[field] * scale,
                    label=[name.replace("pass", "航次 ") + " " + axis for axis in "XYZ"],
                )
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
        ax.legend(ncol=3)
    axes[-1].set_xlabel("采集时间 (s)")
    path = Path(root) / "navigation_pose_errors.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    active = [(name, data['navigation']) for name, data in result.items()
              if name.startswith('pass') and 'ins_error_state' in data['navigation']]
    if active:
        extra, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=True, layout='constrained')
        for name, nav in active:
            state = nav['ins_error_state']
            label = [name.replace('pass', '航次 ') + ' ' + a for a in 'XYZ']
            axs[0].plot(nav['time_s'], state[:, 9:12], label=label)
            axs[1].plot(nav['time_s'], np.rad2deg(state[:, 12:15])*3600, label=label)
            axs[2].plot(nav['time_s'], nav['ins_error_std'][:, :3], label=label)
        for ax, label in zip(axs, ['加计残余偏置 (m/s²)', '陀螺残余偏置 (度/小时)', '位置预测标准差 (m)']):
            ax.set_ylabel(label)
            ax.grid(True, alpha=.3)
            ax.legend(ncol=3)
        axs[-1].set_xlabel('采集时间 (s)')
        extra.savefig(Path(root) / 'imu_gnss_diagnostics.png', dpi=160)
        plt.close(extra)
    return path
