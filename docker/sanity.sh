#!/usr/bin/env bash
# daedalus-sanity — headless verification that the image is actually usable.
#
# Rendering/GPU is intentionally NOT exercised here (the CI/build host has no
# /dev/dri). What we *can* prove:
#   1. both dynamic-linking payloads (libbevy_dylib-<hash>.so, libstd-<hash>.so)
#      are present and resolve — this is what `ldd` proves;
#   2. the binary is a real ELF executable of the expected architecture;
#   3. the ROS 2 environment (rmw + the project's rm_interfaces) is available.
#
# Exit code 0 = healthy.
set -uo pipefail

APP_BIN="${APP_BIN:-/app/daedalus}"
FAIL=0

section() { printf '\n=== %s ===\n' "$*"; }
ok()   { printf '  [ok]   %s\n' "$*"; }
bad()  { printf '  [FAIL] %s\n' "$*"; FAIL=1; }

section "payload"
if [[ -x "$APP_BIN" ]]; then ok "$APP_BIN executable"; else bad "$APP_BIN missing/not executable"; fi
for so in /app/lib/libbevy_dylib-*.so /app/lib/libstd-*.so; do
    if [[ -f "$so" ]]; then ok "found $(basename "$so")"; else bad "missing $so"; fi
done

section "binary identity"
if command -v file >/dev/null 2>&1; then
    file "$APP_BIN"
else
    # `file` is intentionally absent from the lean runtime image.
    magic="$(head -c 5 "$APP_BIN" | od -An -tx1 | tr -d ' \n')"
    size="$(stat -c %s "$APP_BIN")"
    echo "  magic=$magic (7f454c4602 = ELF 64-bit LE)  size=${size} bytes"
    if [[ "$magic" == "7f454c4602" ]]; then ok "valid ELF 64-bit executable"; else bad "bad ELF magic"; fi
fi

section "ldd ($APP_BIN)"
if ldd "$APP_BIN" > /tmp/.daedalus_ldd 2>&1; then
    cat /tmp/.daedalus_ldd
    if grep -q 'not found' /tmp/.daedalus_ldd; then
        bad "unresolved shared library (see 'not found' above)"
    else
        ok "all shared libraries resolve"
    fi
else
    bad "ldd failed"
    cat /tmp/.daedalus_ldd
fi

section "ROS 2 environment"
# --- SOURCE ROS FIRST ---------------------------------------------------------
# Without this, `command -v ros2` below tests a PATH that does not include
# /opt/ros/<distro>/bin, so it reports "[warn] ros2 CLI not installed" and SKIPS
# the interface check — even though the CLI is installed and working. That made
# this gate silently weaker than it looked: it exited 0 without ever verifying
# that the project's own message type is discoverable.
#
# NOTE: setting AMENT_PREFIX_PATH/LD_LIBRARY_PATH as image ENV (as the Dockerfile
# does) is NOT sufficient — it is the ament `setup.bash` that puts
# /opt/ros/<distro>/bin on PATH. This is exactly the gotcha that caused the
# original misdiagnosis ("CLI was removed by autoremove"); it never was.
# Use docker/ros_env.sh when present (it suspends `set -u` around the ament
# scripts, which reference unset vars and would otherwise abort under `set -e`).
if [[ -f /usr/local/bin/ros_env.sh ]]; then
    # shellcheck disable=SC1091
    . /usr/local/bin/ros_env.sh /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash 2>/dev/null || true
else
    set +u
    # shellcheck disable=SC1090
    . "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" 2>/dev/null || true
    set -u
fi
# The colcon workspace ships the project's own rm_interfaces package.
if [[ -f /ros2_ws/install/setup.bash ]]; then
    set +u
    # shellcheck disable=SC1091
    . /ros2_ws/install/setup.bash 2>/dev/null || true
    set -u
fi

echo "  ROS_DISTRO=${ROS_DISTRO:-<unset>}"
echo "  AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-<unset>}"
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo "  ros2 CLI: $(command -v ros2 || echo '<not on PATH>')"
if command -v ros2 >/dev/null 2>&1; then
    ok "ros2 CLI available ($(command -v ros2))"
    # NOTE: `ros2` has no --version flag (verified: it errors with
    # "unrecognized arguments: --version"), and `ros2 --help` does not list the
    # verbs. Exercise the subcommands the README's topic workflow needs instead —
    # that is a real check rather than a banner.
    for v in topic node interface; do
        if ros2 "$v" --help >/dev/null 2>&1; then
            ok "ros2 $v available"
        else
            bad "ros2 $v broken"
        fi
    done
    # The project's own message type must be visible; this is the part of the
    # ROS integration that most easily breaks (rm_interfaces is colcon-built).
    if ros2 interface show rm_interfaces/msg/GimbalCmd >/dev/null 2>&1; then
        ok "rm_interfaces/msg/GimbalCmd visible"
    else
        bad "rm_interfaces/msg/GimbalCmd NOT visible (colcon install not sourced?)"
    fi
else
    # Reaching here means the CLI really is absent, not merely unsourced.
    bad "ros2 CLI not found on PATH even after sourcing ROS (checked /opt/ros/${ROS_DISTRO:-jazzy}/bin)"
fi

section "vulkan"
if command -v vulkaninfo >/dev/null 2>&1; then
    vulkaninfo --summary 2>/dev/null | head -30 || true
else
    echo "  (vulkaninfo not installed; ICD files:)"
    ls -1 /usr/share/vulkan/icd.d/ 2>/dev/null || echo "  none"
fi

section "result"
if [[ "$FAIL" -eq 0 ]]; then
    echo "SANITY: PASS"
else
    echo "SANITY: FAIL"
fi
exit "$FAIL"
