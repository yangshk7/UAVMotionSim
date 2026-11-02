"""从 PX4 ULog 导出未经过 ROS DDS 时间变换的导航与内部真值。"""

import argparse
import csv
from pathlib import Path

from pyulog import ULog

TOPICS = {
    "vehicle_local_position": ("timestamp", "timestamp_sample", "x", "y", "z", "vx", "vy", "vz", "xy_valid", "z_valid"),
    "vehicle_attitude": ("timestamp", "timestamp_sample", "q[0]", "q[1]", "q[2]", "q[3]"),
    "vehicle_local_position_groundtruth": ("timestamp", "timestamp_sample", "x", "y", "z", "vx", "vy", "vz"),
    "vehicle_attitude_groundtruth": ("timestamp", "timestamp_sample", "q[0]", "q[1]", "q[2]", "q[3]"),
}


def extract(ulog_file: Path, output: Path) -> list[Path]:
    """时间字段保留原始微秒整数；输出目录须已存在且目标文件均不存在。"""
    if not output.is_dir():
        raise ValueError("Existing output directory required")
    log = ULog(str(ulog_file), message_name_filter_list=list(TOPICS))
    datasets = {dataset.name: dataset for dataset in log.data_list if dataset.multi_id == 0}
    if set(datasets) != set(TOPICS):
        raise ValueError(f"Missing ULog topics: {set(TOPICS) - set(datasets)}")
    targets = {name: output / f"ulog_{name}.csv" for name in TOPICS}
    if any(path.exists() for path in targets.values()):
        raise FileExistsError("Refusing to overwrite extracted ULog data")
    for name, fields in TOPICS.items():
        data = datasets[name].data
        if any(field not in data for field in fields):
            raise ValueError(f"Missing {name} fields")
        with targets[name].open("x", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(fields)
            writer.writerows(zip(*(data[field] for field in fields), strict=True))
    return list(targets.values())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ulog", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    for path in extract(args.ulog, args.output):
        print(path)


if __name__ == "__main__":
    main()
