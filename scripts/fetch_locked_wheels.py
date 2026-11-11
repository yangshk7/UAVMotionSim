"""Optional Python 3.11 x86_64 wheel fetcher; mirrors must match uv.lock SHA256."""

import argparse
import hashlib
import json
import re
import subprocess
import sys
import tomllib
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("wheelhouse"))
    parser.add_argument("--mirror", default="https://files.pythonhosted.org")
    parser.add_argument("--platform", choices=("linux", "windows"), default="windows" if sys.platform == "win32" else "linux")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    versions = {}
    for line in (root/"requirements.lock").read_text().splitlines():
        match = re.match(r"([\w.-]+)==([^ #]+)", line)
        if match:
            versions[match[1].lower().replace("_", "-")] = match[2]
    wheels = []
    for package in tomllib.loads((root/"uv.lock").read_text())["package"]:
        if versions.get(package["name"]) != package["version"]:
            continue
        candidates = []
        for wheel in package.get("wheels", []):
            file = wheel["url"].rsplit("/", 1)[1]
            platform_ok = ("win_amd64" in file if args.platform == "windows"
                           else "manylinux" in file and "x86_64" in file)
            if file.endswith("any.whl") or platform_ok and ("cp311" in file or "-none-" in file):
                candidates.append(wheel)
        if not candidates:
            raise RuntimeError("No locked compatible wheel: "+package["name"])
        wheels.append(candidates[0])
    build = json.loads((root/"deployment/build_backend.lock.json").read_text())
    wheels.append({"url": build["url"], "hash": "sha256:"+build["sha256"]})
    args.output.mkdir(parents=True, exist_ok=True)
    def fetch(wheel):
        file = args.output/wheel["url"].rsplit("/", 1)[1]
        digest = wheel["hash"].removeprefix("sha256:")
        if not file.exists() or hashlib.sha256(file.read_bytes()).hexdigest() != digest:
            url = wheel["url"].replace("https://files.pythonhosted.org", args.mirror.rstrip("/"), 1)
            subprocess.run(["curl", "-fL", "--retry", "2", "--connect-timeout", "15", "--max-time", "180",
                            "-sS", url, "-o", str(file)], check=True)
        if hashlib.sha256(file.read_bytes()).hexdigest() != digest:
            raise RuntimeError("Locked SHA256 mismatch: "+file.name)
        print("VERIFIED "+file.name, flush=True)
    with ThreadPoolExecutor(max_workers=3) as pool:
        list(pool.map(fetch, wheels))
    print(f"Verified {len(wheels)} locked wheels")


if __name__ == "__main__":
    main()
