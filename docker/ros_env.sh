#!/usr/bin/env bash
# ros_env.sh — source ROS 2 without tripping `set -u`.
#
# WHY THIS EXISTS
# /opt/ros/<distro>/setup.bash (and every ament-generated setup file) references
# AMENT_TRACE_SETUP_FILES without a default:
#
#     $ set -u; . /opt/ros/jazzy/setup.bash
#     /opt/ros/jazzy/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
#
# Our RUN steps use `set -eux`, so sourcing ROS directly aborts the build. The
# ament scripts are not ours to fix, so nounset is suspended for the duration of
# the source (only), and `set -e`/`set -x` behaviour is otherwise preserved by
# the caller.
#
# Usage from a Dockerfile RUN:
#     . /usr/local/bin/ros_env.sh
#       -> sources /opt/ros/$ROS_DISTRO/setup.bash
#     . /usr/local/bin/ros_env.sh /opt/ros/jazzy/setup.bash /ros2_ws/install/setup.bash
#       -> sources each given file, in order
#
# With no arguments it sources "$ROS_DISTRO" (default jazzy).

_ros_env_source_one() {
    local f="$1"
    if [ ! -f "$f" ]; then
        echo "ros_env.sh: no such setup file: $f" >&2
        return 1
    fi
    # Suspend nounset only for the source itself.
    set +u
    # shellcheck disable=SC1090
    . "$f"
    local rc=$?
    set -u
    return $rc
}

if [ "$#" -eq 0 ]; then
    _ros_env_source_one "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
else
    for _f in "$@"; do
        _ros_env_source_one "$_f"
    done
fi
