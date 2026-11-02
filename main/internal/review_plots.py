"""人工验收补充图；只可视化已有结果，不修改仿真或导航数据。"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from uavsim.visualization.chinese_style import use_chinese_font


def plot_pose_comparisons(result: dict, root: Path) -> list[Path]:
    """各轴分开展示 100 Hz 真值与导航估计，位置 m、速度 m/s、姿态 deg。"""
    use_chinese_font()
    paths = []
    passes = [name for name in ("pass1", "pass2") if name in result]
    for name, label, axes_names, unit, filename in [
        (
            "position_sar_m",
            "SAR 位置",
            ("前向 X", "右向 Y", "向上 Z"),
            "m",
            "navigation_position_comparison.png",
        ),
        (
            "velocity_sar_mps",
            "SAR 速度",
            ("前向 X", "右向 Y", "向上 Z"),
            "m/s",
            "navigation_velocity_comparison.png",
        ),
        (
            "euler_NB_rpy_deg",
            "NED/FRD 姿态",
            ("滚转", "俯仰", "偏航"),
            "度",
            "navigation_attitude_comparison.png",
        ),
    ]:
        fig, axes = plt.subplots(
            3, len(passes), figsize=(7 * len(passes), 9), sharex=True, layout="constrained", squeeze=False
        )
        for column, pass_name in enumerate(passes):
            data = result[pass_name]
            truth, nav = data["navigation_truth"], data["navigation"]
            for axis in range(3):
                ax = axes[axis, column]
                ax.plot(truth["time_s"], truth[name][:, axis], color="#122f52", lw=1.5, label="真值")
                ax.plot(
                    nav["time_s"],
                    nav[name][:, axis],
                    color="#df7624",
                    lw=0.85,
                    alpha=0.85,
                    label="导航估计",
                )
                ax.set_ylabel(f"{axes_names[axis]} ({unit})")
                ax.grid(True, alpha=0.25)
                if axis == 0:
                    ax.set_title(f"{pass_name.replace('pass', '航次 ')} — {label}")
                    ax.legend()
                if axis == 2:
                    ax.set_xlabel("采集时间 (s)")
        path = root / filename
        fig.savefig(path, dpi=150)
        plt.close(fig)
        paths.append(path)
    return paths


def plot_geometry_and_actuators(result: dict, root: Path, config) -> list[Path]:
    """APC/CG 杆臂、姿态波束、旋翼边界和四元数可视化。"""
    use_chinese_font()
    passes = [name for name in ("pass1", "pass2") if name in result]
    fig, axes = plt.subplots(
        3, len(passes), figsize=(7 * len(passes), 10), sharex=True, layout="constrained", squeeze=False
    )
    labels = ["X", "Y", "Z"]
    for column, name in enumerate(passes):
        s = result[name]["truth_sar"]
        t = s["time_s"]
        axes[0, column].plot(t, s["apc_position_sar_m"] - s["position_sar_m"], label=labels)
        axes[0, column].set_ylabel("APC 相对质心 (m)")
        axes[0, column].set_title(name.replace("pass", "航次 ") + " — 天线刚体几何")
        axes[1, column].plot(t, s["beam_sar_unit"], label=labels)
        axes[1, column].set_ylabel("SAR 波束单位向量")
        beam = s["beam_sar_unit"]
        depression = np.rad2deg(np.arctan2(-beam[:, 2], np.linalg.norm(beam[:, :2], axis=1)))
        squint = np.rad2deg(np.arctan2(beam[:, 0], beam[:, 1]))
        axes[2, column].plot(t, depression, label="下视角")
        axes[2, column].plot(t, squint, label="斜视角")
        axes[2, column].set_ylabel("波束指向 (度)")
        for ax in axes[:, column]:
            ax.grid(True, alpha=0.25)
            ax.legend(ncol=3)
        axes[2, column].set_xlabel("采集时间 (s)")
    geometry = root / "antenna_geometry.png"
    fig.savefig(geometry, dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(
        3, len(passes), figsize=(7 * len(passes), 10), sharex=True, layout="constrained", squeeze=False
    )
    for column, name in enumerate(passes):
        s = result[name]["truth_sar"]
        t = s["time_s"]
        thrust = config.vehicle.kf * s["rotor_speed_radps"] ** 2
        axes[0, column].plot(t, thrust, lw=0.7)
        axes[0, column].axhline(config.vehicle.max_rotor_thrust_N, color="red", ls="--", label="旋翼推力上限")
        axes[0, column].axhline(0, color="black", ls=":")
        axes[0, column].set_ylabel("旋翼推力 (N)")
        axes[0, column].set_title(name.replace("pass", "航次 ") + " — 执行器约束")
        axes[0, column].legend()
        axes[1, column].plot(t, np.linalg.norm(s["q_NB_wxyz"], axis=1) - 1, label="真值")
        nav = result[name]["navigation"]
        axes[1, column].plot(
            nav["time_s"], np.linalg.norm(nav["q_NB_wxyz"], axis=1) - 1, label="导航估计", alpha=0.7
        )
        axes[1, column].set_ylabel("四元数模长减 1")
        axes[1, column].legend()
        axes[2, column].plot(t, np.abs(s["allocation_residual"]))
        axes[2, column].set_ylabel("控制分配残差 (N / N·m)")
        axes[2, column].set_xlabel("采集时间 (s)")
        for ax in axes[:, column]:
            ax.grid(True, alpha=0.25)
    actuators = root / "actuator_and_quaternion_checks.png"
    fig.savefig(actuators, dpi=150)
    plt.close(fig)
    return [geometry, actuators]
