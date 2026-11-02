"""从 ArduPilot DataFlash 导出共用 TimeUS 的 SIM2 真值与 EKF 导航。"""

import argparse
import csv
from pathlib import Path

from pymavlink import mavutil

TOPICS = {
    "SIM2": ("TimeUS", "PN", "PE", "PD", "VN", "VE", "VD"),
    "SIM": ("TimeUS", "Q1", "Q2", "Q3", "Q4"),
    "XKF1": ("TimeUS", "C", "PN", "PE", "PD", "VN", "VE", "VD"),
    "XKQ": ("TimeUS", "C", "Q1", "Q2", "Q3", "Q4"),
}


def extract(log_path: Path, output: Path) -> dict:
    """写原始日志字段，不合并接收时间；所有目标文件须不存在。"""
    if not output.is_dir():
        raise ValueError("Existing output directory required")
    targets = {name: output / f"dataflash_{name.lower()}.csv" for name in TOPICS}
    if any(path.exists() for path in targets.values()):
        raise FileExistsError("Refusing to overwrite DataFlash extraction")
    counts = {name: 0 for name in TOPICS}
    streams = {}
    try:
        for name, path in targets.items():
            stream = path.open("x", encoding="utf-8", newline="")
            streams[name] = stream
            csv.writer(stream).writerow(TOPICS[name])
        log = mavutil.mavlink_connection(str(log_path))
        while (message := log.recv_match()) is not None:
            name = message.get_type()
            if name not in TOPICS or (name in ("XKF1", "XKQ") and message.C != 0):
                continue
            csv.writer(streams[name]).writerow([getattr(message, field) for field in TOPICS[name]])
            counts[name] += 1
    finally:
        for stream in streams.values():
            stream.close()
    if min(counts.values()) < 2:
        raise ValueError(f"Insufficient DataFlash streams: {counts}")
    return counts


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(extract(args.log, args.output))


if __name__ == "__main__":
    main()
