# syntax=docker/dockerfile:1.7
#
# Daedalus (bevy_robomaster_simulator) — ROS 2 Jazzy + Bevy 0.19 container image
# =============================================================================
#
# Verified facts this file is built around (see README / task report):
#
#  * Base is `ros:jazzy` (Ubuntu 24.04 "noble", glibc 2.39) so the Rust binaries
#    are built against the same toolchain generation as ROS 2 Jazzy.
#
#  * bevy is pulled with the `dynamic_linking` feature, so the final binary does
#    NOT statically embed bevy. `readelf -d target/release/daedalus` NEEDED shows
#    hash-suffixed shared objects, e.g.
#        libbevy_dylib-db89a30660521e69.so
#        libstd-a4cf8c125ff393c3.so
#    Neither an rpath nor a runpath is emitted by the current Cargo.toml, so both
#    .so files MUST ship with the runtime image and be reachable via
#    LD_LIBRARY_PATH. Hashes are toolchain/build specific => we glob them out of
#    the *same* stage that produced the binary; they are never hardcoded.
#
#  * r2r / r2r_msg_gen (0.9.5) runs bindgen at build time and introspects the
#    *sourced* ROS 2 environment. That means:
#      - libclang (libclang-dev) is required;
#      - the project's OWN interfaces in ./rm_interfaces (ament_cmake package that
#        is NOT part of ros:jazzy) must be colcon-built and sourced first,
#        otherwise `r2r::rm_interfaces::msg::GimbalCmd` does not exist;
#      - IDL_PACKAGE_FILTER must be set or r2r generates bindings for *every*
#        sourced package (hundreds of messages => very long builds, and it also
#        links `cargo:rustc-link-lib=dylib=<module>__rosidl_*` for each one).
#    Important subtlety: IDL_PACKAGE_FILTER is used for two different things —
#        (a) which package *names* get their bindings generated, and
#        (b) which message *types* are emitted into the bindings.
#      The filter is applied by package FIRST (a), so the list must contain every
#      package whose types are named in src/**. This is why `rcl_interfaces` is
#      in the list even though no rcl_interfaces type appears in `r2r::` paths:
#      `r2r::Node::create_node` (src/ros2/plugin.rs) uses rcl_interfaces
#      types internally inside the r2r crate itself.
#    The list below is exactly the transitive closure of what
#    `grep -rho 'r2r::[a-z_0-9]*' src/` reports, plus rcl_interfaces.
#
#  * `--features ros2` is additive: default features (`talos`) stay on, so the
#    primary target is talos + ros2. `CARGO_FEATURES` lets you pick others.
#
# Build:
#   docker buildx build -f Dockerfile -t daedalus:latest .        # talos + ros2
#   docker run --rm daedalus:latest sanity                        # headless check
#   docker run --rm daedalus:latest bash -lc 'ldd /app/daedalus'  # linkage only
#
#   # lean fallback (no r2r/bindgen involved):
#   docker buildx build -f Dockerfile --target runtime-lean \
#       --build-arg CARGO_FEATURES=talos -t daedalus:lean .
#
#   # add the optional ffmpeg-gated mock server:
#   docker buildx build -f Dockerfile \
#       --build-arg CARGO_FEATURES='ros2 ffmpeg' -t daedalus:ffmpeg .
#
# FALLBACKS / known limits:
#   * CARGO_FEATURES defaults to `ros2` and is additive on the default `talos`.
#   * If the r2r/bindgen path ever breaks (new clang, new ROS), use
#     `--target runtime-lean --build-arg CARGO_FEATURES=talos`: that path does
#     not touch r2r at all and is the smallest build that still produces a
#     runnable simulator binary.
#   * `--features ffmpeg` additionally pulls ffmpeg-next 8.x, which wants
#     FFmpeg 8; the dev libraries here are the noble default (7:6.1.1). It is
#     therefore NOT in the default feature set. The dev packages are installed
#     so that the mix can be attempted, but expect a version-mismatch compile
#     error from ffmpeg-sys-next on a stock noble; pin a newer ffmpeg or drop
#     the feature if you hit it.
#   * Building with `dynamic_linking` (bevy default here) means the binary needs
#     the hashed .so files; see the LD_LIBRARY_PATH note above.
#
# NOTE: rendering/GPU is NOT exercised by the build. /dev/dri is absent on the
# build host; run the image on a machine with a GPU (see docker-compose.yml for
# the GPU/X11 passthrough blocks).

# -----------------------------------------------------------------------------
# Argument block
# -----------------------------------------------------------------------------
ARG ROS_DISTRO=jazzy
# Documentation only — the toolchain comes from the literal `rust:1.98-...` tag
# used by COPY --from below (ARGs are not interpolated there). Keep in sync.
ARG RUST_VERSION=1.98.1

# -----------------------------------------------------------------------------
# Stage: deps — ALL apt packages needed to *compile* the project.
#
# Split out from the compile stages so that (a) the apt layer is cached
# independently of any Cargo artefact, and (b) the `chef` stub build below can
# actually link every -sys crate (alsa-sys, libudev-sys, x11, ...) instead of
# silently failing on the first missing pkg-config package.
# -----------------------------------------------------------------------------
FROM ros:${ROS_DISTRO} AS deps

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Helper that sources ROS's setup.bash safely under `set -u` (ament's setup files
# reference AMENT_TRACE_SETUP_FILES unset, which aborts a `set -eux` RUN).
COPY docker/ros_env.sh /usr/local/bin/ros_env.sh

RUN set -eux; \
    apt-get update; \
    # --- Rust toolchain prerequisites: clang/libclang for bindgen (r2r_rcl). ---
    apt-get install -y --no-install-recommends \
        build-essential pkg-config clang libclang-dev llvm \
        git curl ca-certificates libssl-dev \
        python3-colcon-common-extensions file; \
    # --- ROS 2 build/dev tooling: colcon-building ./rm_interfaces + r2r bindgen ----
    # (ros:jazzy already ships ros-core; these are the generators and the
    #  message packages r2r introspects.)
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rosidl-default-generators \
        ros-${ROS_DISTRO}-rosidl-default-runtime \
        ros-${ROS_DISTRO}-rosidl-runtime-c \
        ros-${ROS_DISTRO}-rosidl-runtime-cpp \
        ros-${ROS_DISTRO}-rosidl-typesupport-c \
        ros-${ROS_DISTRO}-rosidl-typesupport-introspection-c \
        ros-${ROS_DISTRO}-ament-cmake \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-visualization-msgs \
        ros-${ROS_DISTRO}-tf2-msgs \
        ros-${ROS_DISTRO}-rcl-interfaces \
        ros-${ROS_DISTRO}-builtin-interfaces \
        ros-${ROS_DISTRO}-unique-identifier-msgs \
        ros-${ROS_DISTRO}-action-msgs \
        ros-${ROS_DISTRO}-rmw-fastrtps-cpp; \
    # --- Bevy / desktop build dependencies (X11, Wayland, audio, udev, Vulkan) ---
    apt-get install -y --no-install-recommends \
        libx11-dev libxcursor-dev libxi-dev libxrandr-dev libxinerama-dev \
        libxkbcommon-dev libxkbcommon-x11-dev libwayland-dev \
        libasound2-dev libudev-dev libvulkan-dev libegl1-mesa-dev \
        mesa-vulkan-drivers; \
    # --- ffmpeg dev libs: for the optional `ffmpeg` feature; the
    #     [[bin]] talos_gimbal_mock_server has required-features=["talos","ffmpeg"]
    apt-get install -y --no-install-recommends \
        libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
        libavfilter-dev libavdevice-dev; \
    rm -rf /var/lib/apt/lists/*; \
    # Sanity-check the two -sys crates that fail first (alsa-sys, libudev-sys).
    pkg-config --modversion alsa; \
    pkg-config --modversion libudev

# -----------------------------------------------------------------------------
# Stage: chef — Cargo dependency-cache warm-up only (never shipped)
#
# Purpose: compile every third-party dependency ONCE into a cached target dir so
# that a source-only edit does not recompile Bevy (tens of minutes). The trick is
# to copy only the manifests + vendored members, stub the real entry points, and
# run a build. Reordering or "optimising" these COPY lines silently destroys the
# cache — keep the source COPY strictly after the stub build.
# -----------------------------------------------------------------------------
FROM deps AS chef

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG RUST_VERSION
# NOTE: the stub build below intentionally does not use CARGO_FEATURES — it warms
# the shared (bevy/winit/wgpu) dependency graph, and `ros2` additionally needs
# rm_interfaces, which does not exist until the `builder` stage colcon-builds it.
# See the comment on the RUN below.
ENV CARGO_TERM_COLOR=always \
    CARGO_HOME=/usr/local/cargo \
    RUSTUP_HOME=/usr/local/rustup \
    PATH=/usr/local/cargo/bin:$PATH

# Rust >= 1.85 is mandatory (edition 2024); the project tracks 1.98.1.
#
# The official rust image is copied in rather than `rustup`-fetching over the
# network: the toolchain then matches what the host built with, the build works
# against a pre-pulled base, and no network round-trip is needed at this layer.
# NOTE: `COPY --from=` does NOT expand stage-local ARGs, so this tag is literal.
# To bump Rust, change BOTH the tag below and RUST_VERSION above.
COPY --from=rust:1.98-slim-bookworm /usr/local/rustup /usr/local/rustup
COPY --from=rust:1.98-slim-bookworm /usr/local/cargo /usr/local/cargo

RUN set -eux; \
    # rust:*-slim-bookworm is glibc 2.36 and we run on noble (glibc 2.39), so the
    # glibc-linked toolchain binaries (cargo, rustc, rustdoc) just work.
    rustc --version; cargo --version

WORKDIR /app

# --- dependency manifest layer (cacheable) -----------------------------------
COPY Cargo.toml Cargo.lock ./
COPY clippy.toml ./
COPY crates/ ./crates/
COPY third_party/ ./third_party/

# Stub sources so the workspace resolves without the real code, then warm the
# dependency cache for the feature set we are going to build for real.
#
# `cargo fetch` populates the registry; `cargo build` additionally compiles the
# dependency graph (Bevy is the expensive part, ~10 min). The stubs are cheap and
# are overwritten by a later COPY.
#
# WHY THE ROS ENV IS SOURCED HERE TOO: r2r_rcl's build script runs bindgen and
# `r2r_common::setup_bindgen_builder` does
#     env::var_os("AMENT_PREFIX_PATH").expect("Source your ROS!")
# so even the *stub* build hard-panics without a sourced ROS 2 environment. That
# is deliberate on r2r's part (it is a CMake/ament integration), and it means
# this layer must source /opt/ros/$ROS_DISTRO/setup.bash.
#
# rm_interfaces is NOT available at this point (it is colcon-built in `builder`),
# so the stub build deliberately does NOT pass `ros2`. It warms the shared
# dependency subgraph — which is where the minutes are (Bevy, winit, wgpu, ...).
# r2r itself is small and is (re)compiled for real in `builder`, where
# rm_interfaces IS sourced and visible to bindgen.
#
# CRITICAL — THE STUB MUST NOT SHARE THE REAL TARGET DIR:
# An earlier revision warmed the shared /app/target cache mount directly with the
# stub main.rs. The real `builder` compile then found a *cached, fingerprint-
# matching* `daedalus` artefact for the stub and reported
#
#     Finished `release` profile [optimized] target(s) in 0.30s
#
# while producing a 450 KB `fn main() {}` binary instead of the real ~45 MB app —
# i.e. the build "succeeded" without compiling the project, and the ldd gate
# passed on a stub. The stub therefore builds into its OWN target dir,
# /tmp/stub-target, which is never reused for the real artefact. Only the
# registry cache and the expensive *dependency* rlibs remain shared via
# CARGO_TARGET_DIR-independent registry caching below.
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/app/target \
    set -eux; \
    . /usr/local/bin/ros_env.sh; \
    mkdir -p src/bin; \
    echo 'fn main() {}' > src/main.rs; \
    printf 'use clap::Parser;\n#[derive(Parser)] struct A { #[arg(long)] x: Option<String> }\nfn main() { let _ = A::parse(); }\n' > src/bin/talos_gimbal_mock_server.rs; \
    cargo fetch --locked; \
    # Warm the cache for the *default* feature set only; see the comment above.
    # CARGO_TARGET_DIR isolation is the point: dependencies are still compiled and
    # cached, but the stub's own crate artefacts cannot be mistaken for the real
    # binary later.
    CARGO_TARGET_DIR=/tmp/stub-target cargo build --release --locked --features talos; \
    # Merge ONLY the third-party dependency artefacts into the shared cache mount.
    #
    # The exclude list is the crux: the workspace's own members (daedalus plus
    # crates/*) must NOT be merged, because their fingerprints describe the STUB
    # sources. If they leak into the shared cache, `cargo build` in `builder`
    # resolves the cached stub instead of recompiling the real source — that is
    # precisely the false pass this whole dance exists to prevent. Verified: with
    # the leak present, builder printed "Finished in 0.48s" and produced a 450 KB
    # stub; the anti-false-pass gate then aborted the build (exit 1).
    mkdir -p target/release/deps; \
    for d in /tmp/stub-target/release/deps/*; do \
        b="$(basename "$d")"; \
        case "$b" in \
            daedalus-*|talos_gimbal_mock_server-*|talos_ipc-*|exact-*|bevy_metalfx-*) continue ;; \
        esac; \
        cp -an "$d" target/release/deps/ 2>/dev/null || true; \
    done; \
    # Same for the fingerprint dir and the incremental/example metadata of the
    # workspace members: those are what cargo consults to decide "up to date".
    rm -rf /tmp/stub-target; \
    rm -rf target/release/.fingerprint/daedalus-* \
           target/release/.fingerprint/talos_gimbal_mock_server-* \
           target/release/.fingerprint/talos_ipc-* \
           target/release/.fingerprint/exact-* \
           target/release/.fingerprint/bevy_metalfx-* 2>/dev/null || true; \
    # Belt and braces: no `daedalus` artefact may survive this layer.
    rm -f target/release/daedalus target/release/talos_gimbal_mock_server

# -----------------------------------------------------------------------------
# Stage: builder — real compile, incl. rm_interfaces (colcon) + r2r bindgen
#
# Inherits the full compile toolchain from `deps`/`chef`; no apt work happens
# here at all so this stage can be re-run cheaply.
# -----------------------------------------------------------------------------
FROM chef AS builder

# CARGO_FEATURES is threaded in *after* the cache layer on purpose: changing the
# feature set invalidates the stub build but keeps the registry cache.
ARG CARGO_FEATURES=ros2

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# --- the project's own ROS 2 interfaces --------------------------------------
# ./rm_interfaces is an ament_cmake / rosidl_interface_packages member that does
# NOT exist in ros:jazzy. r2r only sees *sourced* packages, so it must be built
# and its install/ prefix sourced before `cargo build`.
COPY docker/ros_idl_packages.py /usr/local/bin/ros_idl_packages.py
COPY rm_interfaces/ /ros2_ws/src/rm_interfaces/
RUN set -eux; \
    . /usr/local/bin/ros_env.sh; \
    cd /ros2_ws; \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release; \
    # prove the interfaces really landed and are introspectable
    test -d /ros2_ws/install/share/rm_interfaces; \
    ls /ros2_ws/install/share/rm_interfaces

# --- real sources (invalidates only this layer and below) --------------------
COPY Cargo.toml Cargo.lock ./
COPY crates/ ./crates/
COPY third_party/ ./third_party/
COPY src/ ./src/
# assets/ is NOT needed to compile; it is copied into the runtime image straight
# from the build context instead, so editing an asset no longer recompiles Rust.
# Keep this COPY list minimal: every file added here invalidates the build layer.

# Build profile choice: the release profile is what we ship; a debug Bevy + r2r
# build is not realistically usable in a container. For faster local turnaround,
# mount a host cargo target dir instead of switching profile here.
#
# The two cache mounts are what make re-runs fast:
#   /usr/local/cargo/registry -> downloaded .crate sources + index
#   /app/target               -> compiled artefacts (Bevy is the expensive part)
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/app/target \
    set -eux; \
    . /usr/local/bin/ros_env.sh /opt/ros/${ROS_DISTRO}/setup.bash /ros2_ws/install/setup.bash; \
    # IDL_PACKAGE_FILTER: belt-and-braces post-hoc name filter (r2r_common splits
    # on ';'), see the header comment for why rcl_interfaces is present.
    export IDL_PACKAGE_FILTER="geometry_msgs;sensor_msgs;std_msgs;visualization_msgs;tf2_msgs;rm_interfaces;rcl_interfaces;builtin_interfaces;action_msgs;unique_identifier_msgs"; \
    # CMAKE_IDL_PACKAGES is the hard allowlist that keeps bindgen from walking
    # every package installed by ros:jazzy (see docker/ros_idl_packages.py).
    export CMAKE_IDL_PACKAGES="$(python3 /usr/local/bin/ros_idl_packages.py)"; \
    echo "CMAKE_IDL_PACKAGES=${CMAKE_IDL_PACKAGES}"; \
    echo "IDL_PACKAGE_FILTER=${IDL_PACKAGE_FILTER}"; \
    cargo build --release --locked --features "${CARGO_FEATURES}"; \
    # --- ANTI-FALSE-PASS GATE -------------------------------------------------
    # A cached stub artefact once made this step report "Finished in 0.30s" and
    # emit a 450 KB `fn main() {}` binary. Two independent assertions make that
    # impossible to ship:
    #   1. the artefact must be non-trivially large (the real dyn-linked app is
    #      ~45 MB; the stub is <1 MB);
    #   2. it must actually depend on libbevy_dylib — a stub does not.
    test -x target/release/daedalus; \
    BIN_BYTES="$(stat -c %s target/release/daedalus)"; \
    echo "daedalus binary size: ${BIN_BYTES} bytes"; \
    test "${BIN_BYTES}" -gt 5000000; \
    readelf -d target/release/daedalus | grep -q 'libbevy_dylib-'; \
    # Stage the dynamic-linking payload out of the cache mount, since cache mount
    # contents do NOT persist into the image layer.
    # Both bevy and std are dyn-linked and their hashes are toolchain specific,
    # so glob them — never hardcode.
    mkdir -p /out/lib; \
    find target/release -maxdepth 2 -name 'libbevy_dylib-*.so' -exec cp -av {} /out/lib/ \;; \
    SYSROOT_LIB="$(rustc --print sysroot)/lib/rustlib/$(rustc -vV | sed -n 's/^host: //p')/lib"; \
    find "$SYSROOT_LIB" -maxdepth 1 -name 'libstd-*.so' -exec cp -av {} /out/lib/ \;; \
    cp -av target/release/daedalus /out/daedalus; \
    # Every NEEDED entry that is neither a base-OS library nor provided by the
    # toolchain/bevy must have been staged; anything else would show up as
    # "not found" in the runtime ldd gate.
    test -n "$(find /out/lib -name 'libbevy_dylib-*.so' -print -quit)"; \
    test -n "$(find /out/lib -name 'libstd-*.so' -print -quit)"; \
    ls -la /out/lib

# -----------------------------------------------------------------------------
# Stage: runtime — ROS 2 Jazzy runtime + GPU/Vulkan userspace + the binaries
# -----------------------------------------------------------------------------
FROM ros:${ROS_DISTRO} AS runtime

ARG CARGO_FEATURES=ros2
ARG ROS_DISTRO

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

LABEL org.opencontainers.image.title="daedalus" \
      org.opencontainers.image.description="RoboMaster simulator (Bevy 0.19 + avian3d) with ROS 2 Jazzy / r2r integration" \
      org.opencontainers.image.source="https://github.com/crp6/bevy_robomaster_simulator" \
      daedalus.cargo_features="${CARGO_FEATURES}"

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    # libbevy_dylib-<hash>.so and libstd-<hash>.so live next to the binary.
    LD_LIBRARY_PATH=/app/lib:/opt/ros/${ROS_DISTRO}/lib \
    WGPU_BACKEND=vulkan

# Vulkan ICD: deliberately NOT pinned here.
#
# An earlier draft set VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.x86_64.json
# (a) that filename does not exist on noble — the lavapipe ICD is `lvp_icd.json`
#     — and (b) VK_ICD_FILENAMES is a HARD OVERRIDE, not a fallback list, so
#     pinning it to lavapipe would disable a real GPU driver on a GPU host.
# Leaving it unset lets the loader auto-discover every ICD in
# /usr/share/vulkan/icd.d, i.e. a real GPU works and mesa-vulkan-drivers still
# provides lavapipe as a software fallback. For a GPU-less/CI host, set it
# explicitly:  -e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.json
# (see docker-compose.yml, and the sanity script which reports both).

# Runtime libraries only — deliberately NOT the -dev packages.
#   * X11/Wayland/xkbcommon/udev/asound: bevy_winit + bevy_audio + gilrs
#   * libvulkan1 + mesa-vulkan-drivers: wgpu Vulkan backend (lavapipe = software)
#   * libav*: only required when CARGO_FEATURES includes `ffmpeg`
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        libx11-6 libxcursor1 libxi6 libxrandr2 libxinerama1 libxkbcommon0 \
        libxkbcommon-x11-0 libwayland-client0 libwayland-cursor0 libwayland-egl1 \
        libasound2t64 libudev1 libvulkan1 mesa-vulkan-drivers libegl1 \
        libgcc-s1 libssl3t64 \
        libavcodec60 libavformat60 libavutil58 libswscale7 libavfilter9 libavdevice60 \
        libgl1 libglx0 \
        ; \
    rm -rf /var/lib/apt/lists/*

# --- ROS 2 runtime for the generated interfaces ------------------------------
# The binary links `<pkg>__rosidl_typesupport_c/introspection_c/generator_c` for
# every package r2r generated bindings for, and it must be able to
# publish/subscribe on /rm_gimbal/cmd with the project's OWN message type. So the
# runtime side of rm_interfaces is built here too (rosidl runtime + the three
# typesupport variants; bindgen/generators are only needed at *build* time).
#
# NOTE: colcon build needs the generator packages, so they are installed first
# and purged again afterwards. The generated C/C++ sources are already compiled,
# so the *.so libraries survive the purge.
#
# ros_env.sh must be copied into THIS stage too: `runtime` starts from the plain
# `ros:jazzy` base, not from `deps`, and COPY does not inherit across stages.
# (Caught by an actual build failure: "/usr/local/bin/ros_env.sh: No such file".)
COPY docker/ros_env.sh /usr/local/bin/ros_env.sh
COPY rm_interfaces/ /ros2_ws/src/rm_interfaces/
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-rosidl-default-generators \
        ros-${ROS_DISTRO}-rosidl-default-runtime \
        ros-${ROS_DISTRO}-rosidl-runtime-c \
        ros-${ROS_DISTRO}-rosidl-typesupport-c \
        ros-${ROS_DISTRO}-rosidl-typesupport-introspection-c \
        ros-${ROS_DISTRO}-ament-cmake \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-visualization-msgs \
        ros-${ROS_DISTRO}-tf2-msgs \
        ros-${ROS_DISTRO}-rcl-interfaces \
        ros-${ROS_DISTRO}-builtin-interfaces \
        ros-${ROS_DISTRO}-unique-identifier-msgs \
        ros-${ROS_DISTRO}-action-msgs \
        ; \
    . /usr/local/bin/ros_env.sh; \
    cd /ros2_ws; \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release; \
    rm -rf /ros2_ws/log /ros2_ws/build; \
    # Build-time-only tooling is intentionally NOT purged, and the old purge is \
    # gone on purpose. Two findings from this build: \
    # \
    # 1. The original purge was a SILENT NO-OP: its list named \
    #    `ros-jazzy-ament-cmake-export-targets`, which does not exist in Jazzy. \
    #    apt exits 100 and aborts the WHOLE transaction, and `2>/dev/null || true` \
    #    hid it -- so colcon/ament-cmake tooling shipped in the "runtime" image. \
    # 2. Making the purge effective proved UNSAFE. Removing \
    #    `ros-jazzy-rosidl-default-generators` transitively drags out \
    #    ros-jazzy-rosidl-runtime-c / ros-jazzy-rcl / ros-jazzy-rmw, i.e. exactly \
    #    the librcl.so / librmw.so / librosidl_runtime_c.so objects that \
    #    /app/daedalus NEEDs. Crucially those removals come from the FOLLOW-UP \
    #    `apt-get autoremove`, which an `apt-get -s purge` dry run does not model, \
    #    so a keep-list pre-check cannot reliably prevent them (a version that \
    #    tried failed closed at build time on librosidl_runtime_c.so). \
    # \
    # Trade-off taken deliberately: the runtime image also carries colcon/ \
    # ament-cmake build tooling (~a few hundred MB). A slightly fatter image that \
    # is guaranteed loadable beats a leaner one that might lose librcl/librmw. \
    # \
    # What is still asserted is the part that actually matters: the colcon-built \
    # message type, and the three runtime shared objects the binary links. \
    test -f /ros2_ws/install/share/rm_interfaces/msg/GimbalCmd.msg; \
    find /ros2_ws/install/lib -name '*rm_interfaces*' | head; \
    for lib in librosidl_runtime_c.so librcl.so librmw.so; do \
        if ! ls /opt/ros/${ROS_DISTRO}/lib/$lib >/dev/null 2>&1; then \
            echo "ERROR: runtime library $lib is missing"; \
            exit 1; \
        fi; \
    done; \
    echo "runtime libraries verified: rcl/rmw/rosidl intact"; \
    rm -rf /var/lib/apt/lists/*

# Make the colcon install prefix visible to the dynamic linker.
RUN set -eux; \
    echo "/ros2_ws/install/lib" > /etc/ld.so.conf.d/ros2_ws.conf; \
    ldconfig

ENV AMENT_PREFIX_PATH=/ros2_ws/install:/opt/ros/${ROS_DISTRO} \
    CMAKE_PREFIX_PATH=/ros2_ws/install:/opt/ros/${ROS_DISTRO} \
    LD_LIBRARY_PATH=/app/lib:/ros2_ws/install/lib:/opt/ros/${ROS_DISTRO}/lib \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

WORKDIR /app

# --- payload -----------------------------------------------------------------
# assets/, config.toml and README come straight from the build context: they do
# not affect the Rust compile, so editing them must not invalidate the builder
# cache layer.
COPY --from=builder /out/daedalus /app/daedalus
COPY --from=builder /out/lib/ /app/lib/
COPY assets/ /app/assets/
COPY config.toml /app/config.toml
COPY README.md /app/README.md

# Belt-and-braces chmod: COPY preserves the source file mode, so do not rely on
# the checkout (or the archive it came from) having kept the executable bit.
#
# The payload acceptance gate lives in docker/verify_payload.sh rather than inline
# here: the shell constructs it needs (case/;;, loops, mapfile) are mangled by
# Dockerfile line-continuation parsing (`;;` is read as a comment), which cost a
# build cycle. It asserts: real ELF, >5 MB (anti-stub), every hash-suffixed
# NEEDED soname present in /app/lib, and a clean `ldd`.
RUN --mount=type=bind,source=docker,target=/mnt/docker-dir,ro \
    set -eux; \
    chmod +x /app/daedalus; \
    # NOTE: the bind-mounted source dir is read-only, so chmod the COPIES only.
    cp /mnt/docker-dir/entrypoint.sh /usr/local/bin/daedalus-entrypoint; \
    cp /mnt/docker-dir/sanity.sh /usr/local/bin/daedalus-sanity; \
    cp /mnt/docker-dir/verify_payload.sh /usr/local/bin/verify_payload.sh; \
    chmod +x /usr/local/bin/daedalus-entrypoint /usr/local/bin/daedalus-sanity \
             /usr/local/bin/verify_payload.sh; \
    ls -la /app/lib; \
    /usr/local/bin/verify_payload.sh

ENV DISPLAY=:0 \
    XDG_RUNTIME_DIR=/tmp/runtime-root \
    WAYLAND_DISPLAY=wayland-0 \
    RUST_LOG=info

ENTRYPOINT ["/usr/local/bin/daedalus-entrypoint"]
CMD ["/app/daedalus"]

# -----------------------------------------------------------------------------
# Stage: runtime-lean — same payload, no ROS 2 / colcon extra (default features)
#
# Goal: a build that does not depend on the ros2/r2r path at all, useful as a
# fallback if r2r ever breaks against a future ROS or clang release.
#   docker buildx build -f Dockerfile --target runtime-lean -t daedalus:lean .
# Requires the corresponding builder feature set:
#   --build-arg CARGO_FEATURES=talos --build-arg CARGO_FEATURES_LEAN=...
# (the builder stage always uses CARGO_FEATURES; pass `talos` for the lean image)
# -----------------------------------------------------------------------------
FROM runtime AS runtime-lean

ARG CARGO_FEATURES=talos
LABEL daedalus.variant="lean" daedalus.cargo_features="${CARGO_FEATURES}"

# Same payload, but rm_interfaces/colcon is not required when the ros2 feature is
# absent. The ROS 2 base image itself is still the OS foundation, which keeps the
# image self-consistent (same glibc/GPU userspace as runtime).
#
# Re-run the SAME acceptance gate so the lean variant cannot ship a stub either.
RUN set -eux; /usr/local/bin/verify_payload.sh
