#!/usr/bin/env bash
# check_ros2.sh — type-check / test the `ros2` feature without a full image build.
#
# WHY THIS EXISTS
#   `src/ros2/` is behind `#[cfg(feature = "ros2")]`, so a host `cargo check`
#   never looks at it — and r2r's build script hard-panics without a sourced
#   ROS 2 ("Source your ROS!") plus a colcon-built `rm_interfaces`. The only
#   in-repo way to compile that feature used to be the 13-minute
#   `docker build --target runtime`, which turns a one-line edit into a coffee
#   break.
#
#   This reuses the Dockerfile's own `chef` stage — ROS 2 Jazzy + the Rust
#   toolchain + a warmed dependency cache — mounts the working tree into it, and
#   keeps the cargo target dir and the colcon workspace in named volumes so the
#   second run is incremental.
#
# USAGE
#   docker/check_ros2.sh check          # cargo check --features ros2  (default)
#   docker/check_ros2.sh test           # cargo test  --features ros2
#   docker/check_ros2.sh check --release
#
# The first run compiles the whole dependency graph in the dev profile (several
# minutes); later runs only rebuild the crates this workspace owns, so they take
# seconds. Nothing here writes to the host tree.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-daedalus-chef}"
TARGET_VOLUME="${TARGET_VOLUME:-daedalus-check-target}"
ROS_WS_VOLUME="${ROS_WS_VOLUME:-daedalus-check-ros2ws}"

# ROS 2 / r2r environment, mirroring the `builder` stage of the Dockerfile. The
# two IDL variables are the hard allowlist that keeps bindgen from walking every
# message package in the base image (see docker/ros_idl_packages.py).
read -r -d '' INNER <<'EOS' || true
set -euo pipefail

# ros_env.sh is *sourced*, and a sourced script given no arguments sees the
# caller's positional parameters — so our cargo arguments would be interpreted
# as setup files. Park them first.
cargo_args=("$@")
set --

. /usr/local/bin/ros_env.sh

if [ ! -d /ros2_ws/install/share/rm_interfaces ]; then
    echo "=== colcon build rm_interfaces (first run only) ==="
    mkdir -p /ros2_ws/src
    cp -r /app/rm_interfaces /ros2_ws/src/rm_interfaces
    cd /ros2_ws
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_cohesion+
fi

. /usr/local/bin/ros_env.sh /opt/ros/"${ROS_DISTRO}"/setup.bash /ros2_ws/install/setup.bash
export IDL_PACKAGE_FILTER="geometry_msgs;sensor_msgs;std_msgs;visualization_msgs;tf2_msgs;rm_interfaces;rcl_interfaces;builtin_interfaces;action_msgs;unique_identifier_msgs"
# NOTE: /usr/local/bin/ros_idl_packages.py is COPYed in the *builder* stage, so
# it does not exist in the chef image. The mounted working tree has it, and
# `export X="$(...)"` does not fail loudly under `set -e` -- it silently leaves
# the allowlist empty, which makes r2r_msg_gen emit a broken action_bindings.rs.
if [ ! -f /usr/local/bin/ros_idl_packages.py ]; then
    cp /app/docker/ros_idl_packages.py /usr/local/bin/ros_idl_packages.py
fi
export CMAKE_IDL_PACKAGES="$(python3 /usr/local/bin/ros_idl_packages.py)"
echo "CMAKE_IDL_PACKAGES=$(echo "${CMAKE_IDL_PACKAGES}" | tr ' ' '\n' | wc -l) package dirs"

cd /app
echo "=== cargo ${cargo_args[*]} --locked --features ros2 ==="
CARGO_TARGET_DIR=/target cargo "${cargo_args[@]}" --locked --features ros2
EOS

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "=== building the $IMAGE image (Dockerfile target: chef) ==="
    docker build --target chef -t "$IMAGE" "$REPO"
fi

CARGO_ARGS=("$@")
if [ "${#CARGO_ARGS[@]}" -eq 0 ]; then
    CARGO_ARGS=(check)
fi

exec docker run --rm \
    -v "$REPO":/app \
    -v "$TARGET_VOLUME":/target \
    -v "$ROS_WS_VOLUME":/ros2_ws \
    "$IMAGE" \
    bash -c "$INNER" -- "${CARGO_ARGS[@]}"
