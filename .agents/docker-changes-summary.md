# 本次会话改动总结：把 Daedalus 装进 Docker

> 目标：`尝试在 docker 中运行此项目`
> 验收口径（用户选定）：**镜像必须构建成功**；运行期的 GPU 渲染交给有显卡的机器。
> 特性口径（用户选定）：**包含 ROS 2 Jazzy + r2r 构建支持**。

相关文件（本次新增/修改）：

| 文件 | 行数 | 状态 |
|---|---|---|
| `Dockerfile` | 543 | 新增 |
| `.dockerignore` | 89 | 新增 |
| `docker-compose.yml` | 121 | 新增 |
| `docker/entrypoint.sh` | 102 | 新增 |
| `docker/sanity.sh` | 125 | 新增 |
| `docker/ros_env.sh` | 45 | 新增 |
| `docker/ros_idl_packages.py` | 97 | 新增 |
| `docker/verify_payload.sh` | 76 | 新增 |
| `README.md` | +91 | 新增「🐳 Docker 运行」章节 |
| `src/systems/help.rs` | 883 | **恢复**（此前缺失） |
| `src/main.rs` | 1 行 | 修改 |
| `src/ros2/plugin.rs` | 1 行 | 修改 |

---

## 1. 最先挡路的是源码，不是 Docker

开工时发现：**这个仓库在 HEAD 处根本无法编译**，与 Docker 无关。

1. `src/systems/mod.rs` 声明了 `mod help;` / `pub use help::*;`，`src/main.rs` 也 `add_plugins(HelpPlugin)`，
   但 **`src/systems/help.rs` 不存在**。
   - `git ls-tree HEAD src/systems/` 无此文件；`origin/master`、`stash@{0}` 主树也没有；
     GitHub API 查上游同样没有。
   - 上游的 `mod.rs` **根本没有 help 模块** —— 是本地提交（d8c8dc8 / 2b62207 / f9f0b7c）
     加了引用和 README 文档，却从未提交该文件。`mod help;` 未加 cfg，所以 **talos 和 ros2 都编不过**。
2. **恢复方式（关键）**：文件其实还在本仓库对象库里，藏在 stash 的 *untracked-files* 提交（第三个父提交）里：
   ```bash
   git rev-list --all --objects | grep help.rs
   # f4fe7415aad369a8dd36256708db88c315fdf520 src/systems/help.rs
   git cat-file -p f4fe7415aad369a8dd36256708db88c315fdf520 > src/systems/help.rs
   ```
   - 已校验恢复结果与原 blob **逐字节一致**（`git hash-object` 相同），28,818 B / 883 行，
     含 `pub struct HelpPlugin;`（311 行）与 `impl Plugin for HelpPlugin`（313 行）。
   - 教训：`git ls-tree stash@{0}` 只遍历 stash 的*主*树，会漏掉 untracked 提交；
     要查被 stash 吞掉的未跟踪文件，用 `git rev-list --all --objects`。

另外两个**只在 `--features ros2` 下暴露**的真实编译错误（已修，各一行）：

| 位置 | 问题 | 修法 |
|---|---|---|
| `src/main.rs` (`should_enable_talos_plugin`) | 形参是 `_app`，但 `#[cfg(feature="ros2")]` 分支里写的是 `app` → E0425 | 使用点改成 `_app`（比把形参改名更干净：非 ros2 分支下不会产生 unused 警告） |
| `src/ros2/plugin.rs:407` | `TextureFormat::bevy_default()`，但没导入 `BevyDefault` → E0599 | 加 `use bevy::image::BevyDefault;` |

> 注意 `BevyDefault` 的正确路径是 `bevy::image::BevyDefault`（`bevy_internal` 做了
> `pub use bevy_image as image;`）。编译器提示的 `bevy::bevy_image::...` 是**错的**，别照抄。

验证：`cargo check`（默认 talos 特性）→ **EXIT=0**，只有一条无关的 `proc-macro-error2` future-incompat 警告。

---

## 2. Docker 交付物（多阶段构建）

镜像基于 `ros:jazzy`（Ubuntu 24.04 noble），阶段：`deps` → `chef` → `builder` → `runtime` → `runtime-lean`。

构建命令（**本沙箱必须给 DOCKER_CONFIG**，见下）：
```bash
DOCKER_CONFIG=/tmp/dsh-docker-config docker build -f Dockerfile -t daedalus:latest --build-arg CARGO_FEATURES=ros2 .
```

几个非显然的设计点：

- **`dynamic_linking` 必须打包两个 .so**。Bevy 开了 `dynamic_linking`，最终二进制依赖
  `libbevy_dylib-<hash>.so` **和** `libstd-<hash>.so`（rustc sysroot），且 `Cargo.toml` 没有 rpath。
  两者哈希随工具链变化，所以构建时用通配符从**同一个构建阶段**取出放进 `/app/lib`，运行时靠
  `LD_LIBRARY_PATH` 命中。只拷 `libbevy_dylib*` 会漏掉 `libstd-*` 而 `ldd` 报 not found。
- **r2r 连 stub 构建都要求已 source 的 ROS**：`r2r_common` 里有
  `env::var_os("AMENT_PREFIX_PATH").expect("Source your ROS!")`，所以 `chef` 阶段也必须 source。
- **`rm_interfaces` 必须自己 colcon 构建**：它是仓库自带的 ament 包，不在 `ros:jazzy` 里；
  r2r 只认*sourced* 环境，不 build 就生成不出 `r2r::rm_interfaces::msg::GimbalCmd`。
  用 `CMAKE_IDL_PACKAGES`（`docker/ros_idl_packages.py`）限制 bindgen 遍历范围，
  并显式补上 `action_msgs` / `unique_identifier_msgs`（`r2r_actions` 需要）。
- **`set -u` 会打断 ROS 自己的 `setup.bash`**（它引用未定义的 `AMENT_TRACE_SETUP_FILES`），
  所以有 `docker/ros_env.sh` 专门在 source 期间临时关掉 nounset。
- **`docker/verify_payload.sh` 是随镜像发布的验收门禁**（Dockerfile 在 `runtime` 与 `runtime-lean`
  两处调用）。它写成文件而不是内联 RUN，是因为 Dockerfile 行连接解析会把 `;;` 当注释吃掉。

---

## 3. 关键陷阱（每个都差点造成「假通过」）

这三个是本次最有价值的产出，比构建本身更重要：

1. **stub 假通过**：`chef` 阶段的 stub 构建最初与 `builder` 共享 `/app/target`，
   cargo 命中 fingerprint 直接复用 stub 产物 → `builder` 打印 `Finished in 0.30s` 并产出
   **450 KB 的 `fn main() {}`**。它通过了 `test -x`、`.so` 通配、**以及 ldd 门禁**。
   修法：stub 用独立 `CARGO_TARGET_DIR`，把 workspace 成员的 fingerprint 从共享缓存里排除；
   再加「>5 MB + `readelf -d` 必须出现 `libbevy_dylib-`」的硬门禁。
2. **entrypoint 假通过**：`docker/entrypoint.sh` 的 `shell|bash|sh)` 分支无条件 `exec bash -l`，
   **把后面的参数全丢了**。于是 `docker run … bash -lc 'ldd /app/daedalus'` 退出码 0 且**什么都不打印**，
   而「空输出」恰好能通过 `grep -i 'not found'` —— 门禁从未被执行。
   教训：验收命令必须断言**输出非空**（本次实测 56 行），不能只断言「没有报错」。
3. **purge 静默失效**：原 apt purge 列表里写了 Jazzy 里不存在的
   `ros-jazzy-ament-cmake-export-targets`，apt 直接 exit 100 中止整个事务，
   再被 `2>/dev/null || true` 掩盖 —— 结果 colcon 一直留在 runtime 镜像里。

另外：`ros2` CLI 从未被 autoremove 删掉（那是**误诊**）；真正的问题是 `sanity.sh`
没有 source ROS，导致接口检查被静默跳过、还打出一条误导性的 `[warn] ros2 CLI not installed`。

---

## 4. 验证结果

对已验证镜像：**6/6 通过**（由独立 teammate 复现 + Lead 复核）。

| 项 | 结果 |
|---|---|
| `docker build` 成功 | PASS（另有一次 `--no-cache` 全冷构建 EXIT=0，~12m47s） |
| 构建上下文排除 `target/` | PASS —— 守护进程实测 **99.66 MB**（含必需的 92 MB assets），对比 14 GB 仓库 |
| 镜像 payload 完整 | PASS —— `/app/daedalus` 46,772,528 B；两个 .so；9 个 .glb + 2 个 .ogg；config.toml 全部 md5 与宿主一致 |
| `ldd` 无 not found | PASS —— 0 个 not found，且 **56 行非空**（防假通过） |
| 镜像体积/层数 | 2,478,962,284 B（~2.48 GB）/ 23 层 |
| `docker compose config` | PASS（exit 0，无警告） |

ros2 特性**真的链上了**（不只是编译过）：`readelf -d` 的 NEEDED 里有
`librm_interfaces__rosidl_typesupport_c.so` 与 `librm_interfaces__rosidl_generator_c.so`，
以及 `librcl.so` / `librmw.so` / 各种 `*_msgs` typesupport —— 只有 r2r 真的对 colcon 出来的
`rm_interfaces` 跑了 bindgen 才会出现。

---

## 5. 主动做出的取舍（务必知情）

- **不再做 apt purge（重要）**。试图让 purge 真正生效时发现它**不安全**：
  删 `ros-jazzy-rosidl-default-generators` 会经由后续的 `apt-get autoremove`
  连带删掉 `ros-jazzy-rosidl-runtime-c` / `rcl` / `rmw`，也就是
  `librosidl_runtime_c.so` / `librcl.so` / `librmw.so` —— 二进制运行期必需的库。
  而这些删除来自 **autoremove**，`apt-get -s purge` 干跑**不会**模拟它，
  所以「先模拟再保留」的 keep-list 无法可靠拦住（试过的那版就在
  `librosidl_runtime_c.so` 上 fail-closed 了）。
  **决定：牺牲体积换可靠性** —— runtime 镜像里保留 colcon/ament-cmake 构建工具（几百 MB），
  但断言 `rm_interfaces` 消息类型与三个运行期 .so 都在。
  > 若将来要瘦身：正确做法是把运行期包 `apt-mark hold` 住，或改用 `--no-auto-remove`
  > 并显式列出要删的包，而不是靠 purge + autoremove + 干跑模拟。

- **GPU 渲染未验证**：本机没有可用 GPU（无 `/dev/dri`，`vulkaninfo` 失败，
  宿主直接跑该二进制会 `Unable to find a GPU!` panic）。容器与宿主共用内核，
  Docker 无法凭空提供显卡 —— 必须在有显卡的机器上运行。
  `docker-compose.yml` 里 (a) Intel/AMD `--device /dev/dri`、(b) NVIDIA
  `deploy.reservations.devices` 两种直通模板已备好（默认注释）。
  - 额外（非门禁）探针：用 lavapipe 软件渲染 + Xvfb 强制 X11，容器内**跑到了**
    适配器选择（`llvmpipe`）、建窗、ROS2 插件启动、真实资产加载、
    渲染出约 10 帧（1.2–2.1 fps），随后 SIGABRT（未追）。
    这说明容器运行期接线是正确的，但**不等于** GPU 渲染可用。
  - 无 GPU 想试跑：`-e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.json`。
    注意 noble 上文件名是 `lvp_icd.json`（**没有** `.x86_64` 后缀），
    且它是**硬覆盖**，有真显卡时**不要**设，否则会屏蔽真驱动。

- **`ffmpeg` 特性未构建**：noble 是 FFmpeg 7，而 `ffmpeg-next 8.x` 想要 FFmpeg 8，
  预计 `ffmpeg-sys-next` 版本不匹配；dev 包已装好，可自行尝试
  `--build-arg CARGO_FEATURES='ros2 ffmpeg'`。`runtime-lean` 目标存在但未端到端构建过。

- **环境怪癖（不是项目缺陷）**：Docker 29 上即使 `docker build` 也会走 buildx/BuildKit 驱动，
  若 `DOCKER_CONFIG` 不可写会瞬间失败：
  `failed to update builder last activity time: ... read-only file system`。
  解法：`DOCKER_CONFIG=<可写目录>`。另外 `--no-cache` **不会**清 BuildKit 的
  `--mount=type=cache`，所以它并不会真的重编 Rust；要冷编得清 cache mount。

- **`ROS 2 CLI 与 sanity`**：`sanity.sh` / `entrypoint.sh` 现在会 source ROS，
  于是 `docker run --rm daedalus:latest sanity` 会真正跑接口检查并输出
  `[ok] rm_interfaces/msg/GimbalCmd visible`；`docker run --rm daedalus:latest ros2 topic list`
  也会真的执行 ROS CLI，而不是像以前那样回落去启动模拟器。

---

## 6. 常用命令

```bash
# 构建（ros2 + talos，默认特性叠加）
DOCKER_CONFIG=/tmp/dsh-docker-config docker build -t daedalus:latest --build-arg CARGO_FEATURES=ros2 .

# 无需显卡的可用性自检
docker run --rm daedalus:latest sanity

# 只验链接（注意断言输出非空！）
docker run --rm daedalus:latest bash -lc 'ldd /app/daedalus | wc -l'   # 期望 56

# 有显卡时运行（Intel/AMD）
docker run --rm -it --device /dev/dri:/dev/dri --group-add video --group-add render \
  -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $XDG_RUNTIME_DIR:/tmp/runtime-root:rw \
  --network host daedalus:latest

# compose
docker compose build daedalus && docker compose run --rm daedalus sanity
```

---

## 7. 最终状态与后续可做

**最终镜像已重建并复验通过**（与当前 Dockerfile 一致）：

```
daedalus:latest  manifest sha256:c2e1603f7e4d…  size=2,478,971,485 B  layers=23  created=2026-10-09T14:32:01+08:00
```

复验输出：

| 检查 | 结果 |
|---|---|
| `docker build … --features ros2` | `BUILD_EXIT=0`；构建内门禁 `[verify_payload] PASS (binary 46772528 bytes)` |
| `docker run --rm daedalus:latest sanity` | 全 `[ok]`，含 `[ok] rm_interfaces/msg/GimbalCmd visible` → `SANITY: PASS` |
| `ldd /app/daedalus` | **56 行非空**、`not found = 0`、`rm_interfaces` 命中 2 |
| `docker run --rm daedalus:latest ros2 topic list` | `/parameter_events`、`/rosout`（真跑 CLI，不再回落启动模拟器） |
| `VK_ICD_FILENAMES` 是否残留在镜像 ENV | 0（已彻底移除；有真显卡时不会被屏蔽） |
| `/app/daedalus` | 46,772,528 B |

工作区状态：**未做任何 commit**；`git status` 只显示上述新增文件与 3 个修改文件。
临时镜像（deps/chef/builder-fix/builder-lean/verify）已清理，只留 `daedalus:latest`。

## 8. GPU 端到端运行（本轮新增，此前结论作废）

先前记录的「宿主机没有 `/dev/dri`、`vulkaninfo` 失败、无法验证 GPU」是**错误的**：那是调用方被沙箱
挡住了设备节点，不是机器没有显卡。放开权限后实测：

| 项 | 值 |
|---|---|
| 显卡 | `05:00.0 VGA … AMD/ATI Phoenix1 [1002:15bf]`，即 Radeon 780M 核显 |
| 驱动 | `amdgpu` 已加载；Vulkan `RADV PHOENIX`（Mesa 26.2.3 宿主 / 25.2.8 容器） |
| 设备节点 | `/dev/dri/card1`（root:video 0660）、`/dev/dri/renderD128`（0666）；当前用户在 `video` 组 |
| 会话 | KWin Wayland（`wayland-0`）+ XWayland `:0`，屏幕 2560×1600 @1.5 缩放 |
| 容器内 AdapterInfo | `name: "AMD Radeon 780M Graphics (RADV PHOENIX)", backend: Vulkan` |

容器只需要给三样东西：`--device /dev/dri`、把 `XDG_RUNTIME_DIR` 覆盖成宿主机值、挂上
`$XDG_RUNTIME_DIR/wayland-0`（或 `/tmp/.X11-unix`）。运行镜像里 `mesa-vulkan-drivers`
（含 `radeon_icd.json` + `libvulkan_radeon.so`）和 Wayland/X11 客户端库本来就齐了，不必额外安装。
镜像 ENV 里那个 `XDG_RUNTIME_DIR=/tmp/runtime-root` 是**必须覆盖**的坑：那里没有宿主机的 socket。

对照截图（**仅本地**，`.agents/screenshots/` 已加入 `.gitignore`，不入库）：
`screenshots/docker-gpu-container-window.png`（容器实例）、
`screenshots/host-gpu-window.png`（宿主机直跑对照）。

### 8.1 顺带修掉一个只在 ros2 模式暴露的崩溃

第一次带 GPU 起容器时，窗口创建后约 1 秒进程就 `Exited (101)`：

```
thread 'Compute Task Pool (4)' panicked at src/ros2/plugin.rs:218:80:
called `Option::unwrap()` on a `None` value
Encountered a panic in system `daedalus::ros2::plugin::capture_rune`!
```

根因不是 Docker，是**资产与代码的假设不一致**：

- `capture_rune` 对每个 `ArmorRoot` 执行 `qq.of(entity).suffix("CENTER").any().one().unwrap()`；
- 实际只有 `GROUND.glb`（`_ARMOR_CENTER`）、`HERO.glb`（`1__L_ARMOR_CENTER`）、`vehicle.glb`
  （`*_ARMOR_CENTER`）带 `*_CENTER`；
- `OUTPOST.glb` 的 `A..F_ARMOR_ROOT` 子节点是 `*_ARMOR` / `*_ARMOR_BASE` / `*_ARMOR_C` /
  `*_ARMOR_MARKER` / `*_ARMOR_VERTEX_L|R`，**没有任何 `*_CENTER`**；
- 于是「加载前哨站」必然触发该 unwrap（首次出现在装甲构造完成后的那一帧）。

`default`(=talos) 构建不注册这个系统，所以宿主机那份二进制永远碰不到——这也是它长期没被发现的原因。

修法（`src/ros2/plugin.rs`）：新增局部闭包 `armor_center(entity, fallback)`，优先取 `*CENTER` 子节点，
取不到时回退到查询里本来就有、此前以 `_transform` 被刻意忽略的 `ArmorRoot` 自身 `GlobalTransform`。
缺一个可选的中心节点不该把整个模拟器打死。

修复后容器实测：`Up 10 minutes`、`panicked = 0`、稳定 20–55 FPS。

---

剩余可做：

- 若确实想要更小的 runtime 镜像：按上面 `apt-mark hold` 的思路重做瘦身，并**以 `ldd` 门禁兜底**。
- `runtime-lean` 与 `ffmpeg` 特性仍未验证（`ffmpeg` 受 noble 的 FFmpeg 7 vs ffmpeg-next 8.x 影响）。
- compose 里的 NVIDIA (b) 与纯软件渲染 (c) 两段模板仍未实测；只有 AMD (a) 段是验证过的。
- ROS 2 跨容器/跨机发现需要 `--network host`（新命令已带），本次窗口验证未涉及 DDS 连通性。
