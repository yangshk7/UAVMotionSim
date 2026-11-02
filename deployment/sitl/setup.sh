#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
target=/opt/uav_motion/sitl/4.7.2
mkdir -p "$target/bin"
expected=caed35e4cb796ec06cac40b09566a18251771486b1cbac5e2bf16b7a5ed5045d
if [[ ! -f "$target/bin/arducopter" ]]; then
    download="$(mktemp)"
    trap 'rm -f "$download"' EXIT
    curl -fL --max-time 180 https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/arducopter -o "$download"
    printf '%s  %s\n' "$expected" "$download" | sha256sum -c -
    install -m 755 "$download" "$target/bin/arducopter"
fi
printf '%s  %s\n' "$expected" "$target/bin/arducopter" | sha256sum -c -
curl -fL --max-time 60 https://raw.githubusercontent.com/ArduPilot/ardupilot/fb773567896014da007bca58358704d4a0d8516f/Tools/autotest/default_params/copter.parm -o "$target/copter.parm"
if [[ ! -x "$target/venv/bin/python" ]]; then
    uv venv --python /usr/bin/python3 "$target/venv"
fi
uv pip sync --python "$target/venv/bin/python" "$script_dir/requirements.lock"
printf 'SITL environment ready: %s\n' "$target"
