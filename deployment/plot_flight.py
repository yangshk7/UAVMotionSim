"""绘制实际 SITL / XTDrone2 遥测，不插值生成虚构的 SAR 真值。"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from uavsim.visualization.chinese_style import use_chinese_font


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    args = parser.parse_args()
    data = np.atleast_1d(np.genfromtxt(args.input / "navigation_position_sar.csv", delimiter=",", names=True))
    if not len(data):
        raise ValueError("没有实际位置遥测")
    use_chinese_font()
    figure = plt.figure(figsize=(12, 5), layout="constrained")
    axis = figure.add_subplot(121, projection="3d")
    axis.plot(data["x_sar_m"], data["y_sar_m"], data["z_sar_m"], label="导航估计航迹")
    axis.set(
        xlabel="局部前向 X (m)",
        ylabel="局部右向 Y (m)",
        zlabel="局部向上 Z (m)",
        title="实际仿真遥测三维航迹（各轴独立比例）",
    )
    axis.legend()
    altitude = figure.add_subplot(122)
    time_field = "timestamp_s" if "timestamp_s" in data.dtype.names else "time_boot_s"
    altitude.plot(data[time_field] - data[time_field][0], data["z_sar_m"])
    altitude.set(xlabel="本次记录相对时间 (s)", ylabel="局部原点 Z (m)", title="起降期间导航 Z（非离地真值）")
    altitude.grid(alpha=0.3)
    output = args.input / "flight_summary.png"
    figure.savefig(output, dpi=150)
    plt.close(figure)
    try:
        display_path = output.resolve().relative_to(Path.cwd())
    except ValueError:
        display_path = output.resolve()
    print(display_path)


if __name__ == "__main__":
    main()
