#!/usr/bin/env python3
"""Collect the cmake package dirs of the ROS 2 packages r2r must introspect.

r2r / r2r_msg_gen 0.9.5 resolves what to generate in `r2r_common::get_wanted_messages()`:

  1. If `CMAKE_IDL_PACKAGES` is set, it uses exactly those packages (each entry is
     a `<prefix>/share/<pkg>/cmake` directory; r2r takes `.parent()` of each, i.e.
     `<prefix>/share/<pkg>`, and scans msg/ srv/ action/ there).
  2. Otherwise it walks `AMENT_PREFIX_PATH` + `CMAKE_PREFIX_PATH` and picks up
     *every* rosidl interface package it finds — that includes all of
     ros-jazzy-*-msgs and is why bindgen takes forever.

`IDL_PACKAGE_FILTER` is applied *after* step 2 as a package-NAME filter, and only
when `CMAKE_IDL_PACKAGES` is NOT set. Setting `CMAKE_IDL_PACKAGES` is therefore the
strictly better control: it is a hard allowlist instead of a post-hoc filter, and
it avoids enumerating the whole ROS 2 installation at all.

The package list is the transitive closure of what the crate actually names:

    $ grep -rho 'r2r::[a-z_0-9]*' src/ | sort -u
    r2r::builtin_interfaces  r2r::geometry_msgs     r2r::rm_interfaces
    r2r::sensor_msgs         r2r::std_msgs          r2r::tf2_msgs
    r2r::visualization_msgs

plus `rcl_interfaces`: no rcl_interfaces type appears by path in src/, but
`r2r::Node` / `r2r::Context` (src/ros2/plugin.rs) internally use
rcl_interfaces (Parameter*/SetParameters) messages, so the bindings will not
compile without it.
"""

from __future__ import annotations

import os
import sys

PACKAGES = (
    # --- named directly in src/ ---
    "geometry_msgs",
    "sensor_msgs",
    "std_msgs",
    "visualization_msgs",
    "tf2_msgs",
    "builtin_interfaces",
    "rm_interfaces",  # the project's own interfaces, colcon-built into /ros2_ws
    # --- required by the r2r crate's own internals, NOT by src/ ---
    # `r2r::Node` / `r2r::Context` (src/ros2/plugin.rs) use rcl_interfaces
    # (Parameter*/SetParameters) messages.
    "rcl_interfaces",
    # r2r_actions' build script links rcl_action_* to action_msgs srv types:
    #     error[E0425]: cannot find type `action_msgs__srv__CancelGoal_Response`
    # so the action_msgs *services* must be present too. Note this is exactly the
    # set r2r_common.get_wanted_messages() force-adds to IDL_PACKAGE_FILTER
    # (`rcl_interfaces`, `builtin_interfaces`, `unique_identifier_msgs`,
    #  `action_msgs`) — because CMAKE_IDL_PACKAGES short-circuits that code path,
    # the same four must be listed here explicitly. Verified by build failure.
    "action_msgs",
    "unique_identifier_msgs",
)

# Order matters: put the colcon workspace first so that a locally built
# rm_interfaces wins over any system copy.
PREFIXES = ["/ros2_ws/install"]


def main() -> int:
    prefixes = list(PREFIXES)
    for var in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"):
        for p in os.environ.get(var, "").split(":"):
            if p and p not in prefixes:
                prefixes.append(p)
    for p in os.environ.get("EXTRA_ROS_PREFIXES", "").split(":"):
        if p and p not in prefixes:
            prefixes.append(p)

    dirs: list[str] = []
    missing: list[str] = []
    for pkg in PACKAGES:
        for prefix in prefixes:
            cand = os.path.join(prefix, "share", pkg, "cmake")
            if os.path.isdir(cand) and cand not in dirs:
                dirs.append(cand)
                break
        else:
            missing.append(pkg)

    print(":".join(dirs))
    if missing:
        print(
            "ERROR: no cmake dir found for ROS packages: " + ", ".join(missing),
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
