#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
project_dir="$(cd "$script_dir/../.." && pwd)"
px4=/opt/uav_motion/PX4-Autopilot
xtdrone=/opt/uav_motion/xtdrone_ws/src/XTDrone2
patch_file="$project_dir/deployment/patches/xtdrone-stop-offboard-on-land.patch"
scenario_file=""
truth_rate_hz=1000
route_mode=false
while (($#)); do
    case "$1" in
        --route) route_mode=true; shift ;;
        --scenario) scenario_file="$2"; shift 2 ;;
        --truth-rate-hz) truth_rate_hz="$2"; shift 2 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done
if ! git -C "$xtdrone" apply --reverse --check "$patch_file" >/dev/null 2>&1; then
    git -C "$xtdrone" apply --check "$patch_file"
    git -C "$xtdrone" apply "$patch_file"
fi
model_args=()
if [[ -n "$scenario_file" ]]; then model_args=(--scenario "$scenario_file"); fi
python3 "$script_dir/generate_hex_model.py" --px4 "$px4" --output /opt/uav_motion/models --truth-rate-hz "$truth_rate_hz" "${model_args[@]}"
install -m 644 /opt/uav_motion/models/6001_gz_hex_x12 "$px4/ROMFS/px4fmu_common/init.d-posix/airframes/6001_gz_hex_x12"
install -m 644 /opt/uav_motion/models/6001_gz_hex_x12 "$px4/build/px4_sitl_default/etc/init.d-posix/airframes/6001_gz_hex_x12"
source /opt/ros/jazzy/setup.bash
source /opt/uav_motion/xtdrone_ws/install/setup.bash
if $route_mode; then
    output_dir="$project_dir/results/xtdrone_route"
else
    output_dir="$project_dir/results/xtdrone"
fi
validation_args=(--truth-rate-hz "$truth_rate_hz")
if $route_mode; then validation_args+=(--route); fi
if [[ -n "$scenario_file" ]]; then validation_args+=(--scenario "$scenario_file"); fi
python3 "$script_dir/validate_stack.py" --output "$output_dir" "${validation_args[@]}"
