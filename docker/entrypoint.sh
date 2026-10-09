#!/usr/bin/env bash
# daedalus-entrypoint — thin launcher for the Daedalus container.
#
# Behaviour:
#   * first arg is an absolute path or a `sanity`/`shell` keyword -> dispatched
#   * no args -> runs the app binary in "$PWD" (i.e. /app/daedalus)
#
# Examples
#   docker run --rm daedalus:latest sanity           # linkage + ROS env check
#   docker run --rm -it daedalus:latest shell        # interactive bash
#   docker run --rm daedalus:latest bash -lc 'ldd /app/daedalus'
#   docker run --rm daedalus:latest                  # start the simulator (needs GPU/X11)
set -euo pipefail

# Make sure the colcon-installed interfaces and the ROS 2 runtime are on the
# search path even when the container is started with a clean environment.
: "${LD_LIBRARY_PATH:=/app/lib:/ros2_ws/install/lib:/opt/ros/${ROS_DISTRO:-jazzy}/lib}"
export LD_LIBRARY_PATH

# --- ROS 2 environment --------------------------------------------------------
# Setting LD_LIBRARY_PATH/AMENT_PREFIX_PATH as image ENV is NOT enough to expose
# the `ros2` CLI: only ament's setup.bash puts /opt/ros/<distro>/bin on PATH. So
# without this block `docker run --rm daedalus:latest ros2 topic list` fails with
# "executable file not found", which is a poor experience for an image whose whole
# purpose is a ROS 2 simulator. Sourcing here makes the documented ROS 2 topic
# workflow (ros2 topic list / ros2 topic echo /rm_gimbal/cmd) work directly.
#
# Failure to source is non-fatal on purpose: the simulator itself does not need
# the CLI, so we must not break `docker run ... /app/daedalus` if ament's scripts
# ever change. `set +u` is needed because ament's setup files reference
# AMENT_TRACE_SETUP_FILES unset, which aborts under `set -u`.
_ros_source() {
    local f="$1"
    [[ -f "$f" ]] || return 0
    set +u
    # shellcheck disable=SC1090
    . "$f" || true
    set -u
}
_ros_source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
_ros_source /ros2_ws/install/setup.bash

log() { printf '[daedalus] %s\n' "$*" >&2; }

case "${1:-}" in
    sanity)
        exec daedalus-sanity
        ;;
    shell)
        exec bash -l
        ;;
    # NOTE: `bash`/`sh` must only be treated as "give me a shell" when they are
    # the ONLY argument. The previous version matched `bash|sh` unconditionally
    # and ran `exec bash -l`, silently DISCARDING the rest of the command line —
    # so the documented `docker run ... bash -lc 'ldd /app/daedalus'` produced no
    # output at all (and looked like a passing empty result). Caught in review.
    bash|sh)
        if [[ $# -gt 1 ]]; then
            exec "$@"
        fi
        exec bash -l
        ;;
    "")
        log "starting /app/daedalus (requires a working GPU/Vulkan + a display)"
        exec /app/daedalus
        ;;
    --help|-h)
        cat >&2 <<'EOF'
daedalus container entrypoint

  sanity            run the headless sanity check (ldd + ROS env), exit 0/1
  shell | bash | sh interactive shell
  <cmd> [args...]   run an arbitrary command (e.g. `bash -lc 'ldd /app/daedalus'`)
  (no arguments)    start the simulator; needs /dev/dri (or lavapipe) and a display

The ROS 2 environment (/opt/ros/jazzy + the colcon-built rm_interfaces) is
sourced by this entrypoint, so the CLI works without a manual source:
  docker run --rm daedalus:latest ros2 topic list
  docker run --rm daedalus:latest ros2 interface show rm_interfaces/msg/GimbalCmd
(For the simulator to see a robot on the LAN, use --network host and match
ROS_DOMAIN_ID, as docker-compose.yml does.)

Useful environment variables:
  ROS_DOMAIN_ID                    DDS domain for discovery
  RMW_IMPLEMENTATION               default rmw_fastrtps_cpp
  DAEDALUS_FORCE_TALOS_CAPTURE=1   force the Talos capture path (see src/main.rs)
  VK_ICD_FILENAMES                 override the Vulkan ICD; leave UNSET on a GPU
                                   host, or set to
                                   /usr/share/vulkan/icd.d/lvp_icd.json for the
                                   mesa software rasteriser (no-GPU/CI)
  LIBGL_ALWAYS_SOFTWARE=1          force software GL if llvmpipe is used
EOF
        exit 0
        ;;
    *)
        if command -v "$1" >/dev/null 2>&1 || [[ "$1" == /* ]]; then
            exec "$@"
        fi
        log "unknown command '$1'; falling back to /app/daedalus $*"
        exec /app/daedalus "$@"
        ;;
esac
