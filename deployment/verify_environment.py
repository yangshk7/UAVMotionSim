"""Read-only verification of external source pins, patches and runtime prerequisites."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path


def verify():
    root = Path(__file__).resolve().parent
    lock = json.loads((root/"environment.lock.json").read_text())
    checks = []
    def check(name, action):
        try:
            detail = action()
            checks.append({"name": name, "passed": True, "detail": detail})
        except Exception as error:
            checks.append({"name": name, "passed": False, "detail": str(error)})
    def command(args):
        return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT, timeout=30).strip()
    def equal(actual, expected):
        if actual != expected:
            raise ValueError(f"Expected {expected!r}, found {actual!r}")
        return actual
    for name, spec in lock["repositories"].items():
        check(name, lambda spec=spec: equal(command(["git", "-C", spec["path"], "rev-parse", "HEAD"]), spec["commit"]))
    for repository, patch in (("Micro-XRCE-DDS-Agent", "agent-fastdds-pin.patch"),
                              ("Fast-DDS", "fastdds-cstdint.patch"), ("Fast-DDS", "fastdds-build-compat.patch"),
                              ("XTDrone2", "xtdrone-stop-offboard-on-land.patch")):
        check(patch, lambda repository=repository, patch=patch: command([
            "git", "-C", lock["repositories"][repository]["path"], "apply", "--reverse", "--check", str(root/"patches"/patch)]))
    spec = lock["sitl"]
    check("SITL binary", lambda: equal(hashlib.sha256(Path(spec["path"]).read_bytes()).hexdigest(), spec["sha256"]))
    check("Gazebo", lambda: equal(command(["bash", "-c", "source /opt/ros/jazzy/setup.bash && gz sim --versions"]), lock["gazebo_sim"]))
    for line in (root/"apt_packages.lock").read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        package, version = line.split("=", 1)
        check(package, lambda package=package, version=version: equal(command(["dpkg-query", "-W", "-f", "${Version}", package]), version))
    for file in ("/opt/uav_motion/px4_venv/bin/python", "/opt/uav_motion/sitl/4.7.2/venv/bin/python",
                 "/opt/uav_motion/PX4-Autopilot/build/px4_sitl_default/bin/px4", "/usr/local/bin/MicroXRCEAgent",
                 "/opt/uav_motion/xtdrone_ws/install/setup.bash"):
        check(file, lambda file=file: command(["test", "-f", file]))
    return {"release": lock["release"], "passed": all(c["passed"] for c in checks), "checks": checks,
            "scope": "Environment pins and prerequisites only; actual flight acceptance must run separately"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = verify()
    value = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(value, encoding="utf-8")
    print(value)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
