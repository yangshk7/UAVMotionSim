"""记录 Linux 实际安装版本和文件哈希，不把安装等同于飞行验收通过。"""

import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def output(command):
    return subprocess.check_output(command, text=True, timeout=20).strip()


def main():
    base = Path("/opt/uav_motion")
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "os_release": Path("/etc/os-release").read_text(),
        "repositories": {},
        "binaries": {},
    }
    for name, directory in {
        "PX4": base / "PX4-Autopilot",
        "XTDrone2": base / "xtdrone_ws/src/XTDrone2",
        "px4_msgs": base / "xtdrone_ws/src/XTDrone2/xtd2_third_party_pkgs/px4_msgs",
        "Micro-XRCE-DDS-Agent": base / "Micro-XRCE-DDS-Agent",
        "Fast-DDS": base / "Micro-XRCE-DDS-Agent/build/fastdds/src/fastdds",
    }.items():
        report["repositories"][name] = {
            "path": str(directory),
            "commit": output(["git", "-C", str(directory), "rev-parse", "HEAD"]),
            "local_changes": output(["git", "-C", str(directory), "status", "--short"]),
        }
    for name, binary in {
        "arducopter": base / "sitl/4.7.2/bin/arducopter",
        "px4": base / "PX4-Autopilot/build/px4_sitl_default/bin/px4",
        "MicroXRCEAgent": Path("/usr/local/bin/MicroXRCEAgent"),
    }.items():
        report["binaries"][name] = {
            "path": str(binary),
            "sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
        }
    report["packages"] = output(
        [
            "dpkg-query",
            "-W",
            "-f",
            "${Package}=${Version}\n",
            "ros-jazzy-ros-base",
            "ros-jazzy-ros-gz",
            "ros-jazzy-gz-sim-vendor",
            "ros-jazzy-tf-transformations",
            "python3-colcon-common-extensions",
        ]
    )
    report["gazebo_version"] = output(["gz", "sim", "--versions"])
    target = Path(__file__).resolve().parent / "environment_snapshot.json"
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(target)


if __name__ == "__main__":
    main()
