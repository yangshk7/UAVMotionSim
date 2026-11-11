#!/usr/bin/env bash
# Ubuntu 24.04 x86_64 only. Install in a dedicated WSL distribution as root.
set -eo pipefail
project=$(cd "$(dirname "$0")/.." && pwd)
if [[ ${1:-} == --check ]]; then exec python3 "$project/deployment/verify_environment.py"; fi
[[ $(id -u) == 0 ]] || { echo 'Run as root in a dedicated WSL distribution' >&2; exit 2; }
source /etc/os-release
[[ $VERSION_ID == 24.04 && $(uname -m) == x86_64 ]] || { echo 'Requires Ubuntu 24.04 x86_64' >&2; exit 2; }
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y curl ca-certificates gnupg git locales software-properties-common
locale-gen en_US.UTF-8
export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
add-apt-repository -y universe
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main' > /etc/apt/sources.list.d/ros2.list
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main' > /etc/apt/sources.list.d/gazebo-stable.list
apt-get update
apt-get install -y build-essential cmake ninja-build pkg-config python3-venv python3-dev python3-pip \
    libeigen3-dev libxml2-dev libtinyxml2-dev libssl-dev libasio-dev libgtest-dev \
    libopencv-dev libprotobuf-dev protobuf-compiler libgz-sim8-dev gz-harmonic
mapfile -t packages < <(grep -v '^#' "$project/deployment/apt_packages.lock" | sed '/^$/d')
apt-get install -y "${packages[@]}"
if ! command -v uv >/dev/null; then
    curl -fsSL https://github.com/astral-sh/uv/releases/download/0.12.1/uv-x86_64-unknown-linux-gnu.tar.gz -o /tmp/uav_uv.tar.gz
    tar -xzf /tmp/uav_uv.tar.gz -C /tmp
    install -m755 /tmp/uv-x86_64-unknown-linux-gnu/uv /usr/local/bin/uv
fi
bash "$project/deployment/sitl/setup.sh"
read_pin() { python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["repositories"][sys.argv[2]][sys.argv[3]])' "$project/deployment/environment.lock.json" "$1" "$2"; }
clone_pin() {
    local name=$1 target url commit
    target=$(read_pin "$name" path); url=$(read_pin "$name" url); commit=$(read_pin "$name" commit)
    if [[ ! -d $target/.git && ! -f $target/.git ]]; then
        [[ ! -e $target ]] || { echo "Refusing to replace existing $target" >&2; exit 3; }
        git clone --no-checkout "$url" "$target"
        git -C "$target" checkout --detach "$commit"
    fi
    [[ $(git -C "$target" rev-parse HEAD) == "$commit" ]] || { echo "Wrong revision in $target; not overwriting" >&2; exit 3; }
}
patch_once() {
    if ! git -C "$1" apply --reverse --check "$2" >/dev/null 2>&1; then
        git -C "$1" apply --check "$2"
        git -C "$1" apply "$2"
    fi
}
clone_pin PX4
px4=$(read_pin PX4 path)
git -C "$px4" submodule update --init --recursive
uv venv --allow-existing --python /usr/bin/python3 /opt/uav_motion/px4_venv
uv pip sync --python /opt/uav_motion/px4_venv/bin/python "$project/deployment/px4/requirements.lock"
export PATH=/opt/uav_motion/px4_venv/bin:$PATH
jobs=${UAV_BUILD_JOBS:-2}
make -C "$px4" -j"$jobs" px4_sitl_default
clone_pin Micro-XRCE-DDS-Agent
agent=$(read_pin Micro-XRCE-DDS-Agent path)
patch_once "$agent" "$project/deployment/patches/agent-fastdds-pin.patch"
cmake -S "$agent" -B "$agent/build" -DUAGENT_SUPERBUILD=ON -DCMAKE_BUILD_TYPE=Release
# Run the generated download step before patching the exact pinned dependency.
cmake -P "$agent/build/fastdds/tmp/fastdds-gitclone.cmake"
fastdds=$(read_pin Fast-DDS path)
[[ $(git -C "$fastdds" rev-parse HEAD) == $(read_pin Fast-DDS commit) ]]
patch_once "$fastdds" "$project/deployment/patches/fastdds-cstdint.patch"
patch_once "$fastdds" "$project/deployment/patches/fastdds-build-compat.patch"
cmake --build "$agent/build" -j"$jobs"
cmake --install "$agent/build"
ldconfig
clone_pin XTDrone2
xtdrone=$(read_pin XTDrone2 path)
# Checkout only the message submodule; the documentation wiki is not a build dependency.
git -C "$xtdrone" submodule update --init xtd2_third_party_pkgs/px4_msgs
[[ $(git -C "$(read_pin px4_msgs path)" rev-parse HEAD) == $(read_pin px4_msgs commit) ]] || {
    git -C "$(read_pin px4_msgs path)" fetch origin "$(read_pin px4_msgs commit)"
    git -C "$(read_pin px4_msgs path)" checkout --detach "$(read_pin px4_msgs commit)"
}
patch_once "$xtdrone" "$project/deployment/patches/xtdrone-stop-offboard-on-land.patch"
# ROS Python packages use the system interpreter, independently of the PX4 build venv.
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/jazzy/setup.bash
cd /opt/uav_motion/xtdrone_ws
colcon build --symlink-install --parallel-workers "$jobs" --cmake-args -DCMAKE_BUILD_TYPE=Release
python3 "$project/deployment/xtdrone/generate_hex_model.py" --px4 "$px4" --output /opt/uav_motion/models --truth-rate-hz 1000
python3 "$project/deployment/verify_environment.py"
