"""上位机可旋转三维航迹；只读取 100 Hz 导航 CSV，不修改仿真数据。"""

import numpy as np

from uavsim.visualization.chinese_style import use_chinese_font


def draw_trajectory(axes, navigation, truth, point: str, equal_scale: bool) -> None:
    """绘制 SAR 坐标航迹，单位 m；导航与真值均来自同一航次的 CSV。

    point 只能为 cg 或 apc。独立比例模式用于观察小偏差，不表示真实几何比例。
    """
    if point not in {"cg", "apc"}:
        raise ValueError("航迹位置必须选择 cg 或 apc")
    use_chinese_font()
    fields = (
        ("x_sar_m", "y_sar_m", "z_sar_m") if point == "cg" else ("apc_x_sar_m", "apc_y_sar_m", "apc_z_sar_m")
    )
    paths = []
    for source in (truth, navigation):
        points = np.column_stack([source[field] for field in fields])
        if points.ndim != 2 or points.shape[0] == 0 or not np.all(np.isfinite(points)):
            raise ValueError("三维航迹数据为空或包含非有限数值")
        paths.append(points)
    axes.clear()
    label = "质心" if point == "cg" else "APC"
    for points, name, color, linestyle in zip(
        paths, ("真实航迹", "导航估计航迹"), ("#173f68", "#e37724"), ("-", "--")
    ):
        axes.plot(*points.T, color=color, lw=1.6, ls=linestyle, label=name)
    axes.scatter(*paths[1][0], color="#16834a", s=45, label="起点", depthshade=False)
    axes.scatter(*paths[1][-1], color="#b42f39", s=45, label="终点", depthshade=False)
    axes.set(
        xlabel="SAR 前向 X (m)",
        ylabel="SAR 右向 Y (m)",
        zlabel="SAR 向上 Z (m)",
        title=f"{label} 三维航迹：导航估计与真值（{'真实比例' if equal_scale else '各轴独立比例'}）",
    )
    axes.legend(loc="upper left")
    if equal_scale:
        all_points = np.vstack(paths)
        spans = np.maximum(np.ptp(all_points, axis=0), 0.01)
        axes.set_box_aspect(spans, zoom=0.82)
    else:
        axes.set_box_aspect((1.8, 1.0, 1.0), zoom=0.82)
