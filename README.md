<div align="center">

# 🧪 Daedalus

**RoboMaster 视觉算法验证模拟器**

*为算法而生的实验场，让自瞄在上场前就经历真实考验*

[![Rust](https://img.shields.io/badge/Rust-Stable-orange.svg?style=for-the-badge&logo=rust)](https://www.rust-lang.org/)
[![Bevy](https://img.shields.io/badge/Bevy-Engine-3A3A3A.svg?style=for-the-badge&logo=bevy)](https://bevyengine.org/)
[![ROS2](https://img.shields.io/badge/ROS2-Integrated-22314E.svg?style=for-the-badge&logo=ros)](https://www.ros.org/)

</div>

## 🚀 功能亮点

* 🎯 **全要素战场环境仿真**
  覆盖能量机关、前哨站、大/小装甲模块等 RoboMaster 核心视觉目标，提供高保真的外观与状态模拟。

* 🤖 **多机器人模型与行为**
  支持步兵（Infantry）与英雄（Hero）机器人的移动、底盘旋转、云台控制与弹丸发射。

* 🔄 **算法-数据-控制完整闭环**
  原生打通 **图像采集 → 目标标注 → ROS2/Talos 推理 → 云台反馈**，实现“看-算-打”全流程验证。

* ⚔️ **多主体动态对抗模拟**
  己方与多个假人独立控制，支持 Tab 键实时切换，真实构造遮挡、对抗与复杂战场场景。

* 🚀 **双通道实时通信接口**
    - **ROS2 原生集成**：直接发布图像、TF 与位姿话题，零成本接入现有自瞄系统
    - **Talos 零拷贝 IPC**：与 [talos](https://github.com/Blackjack200/talos) 通过共享内存通信，支持实时姿势发布与云台命令订阅

* ⚡️ **高性能实时渲染管线**
  基于 Bevy 引擎，支持 CPU/GPU 渲染，保证高帧率与严格的时间一致性。

---

## 🎨 功能覆盖与开发路线

### ✅ 已实现

#### 🏟️ 战场环境仿真

* **能量机关完整仿真** - 大/小能量机关的激活流程与视觉状态模拟
* **前哨站完整仿真** - 前哨站外观与装甲模块状态模拟
* **装甲模块建模与渲染** - 大、小装甲模块全部图案双色灯条显示

#### 🤖 机器人模型与行为

* **步兵机器人（Infantry）** - 移动、底盘旋转、云台控制、17mm弹丸发射
* **英雄机器人（Hero）** - 大装甲模块专属配置、移动与发射行为
* **物理动力学模拟** - 基于物理的移动、旋转与碰撞响应

#### 🔌 通信接口集成

* **ROS2 原生集成** - 发布 `/image_raw`、`/camera_info`、`/tf` 等话题，订阅 `/rm_gimbal/cmd`
* **Talos 共享内存 IPC** - 与 C++ talos-cpp 零拷贝通信，发布 odom/gimbal/muzzle/camera 姿势，订阅云台控制命令

#### 📊 工具与接口

* **控制指令订阅** - 支持 ROS2 与 Talos 双通道控制指令接入
* **假人控制切换** - Tab 键实时切换活动假人，支持多机器人测试场景
* **键位提示面板** - 左下角面板随输入设备（键盘/手柄）、视角与自瞄状态实时切换键位说明（`F1` 开关）

#### ⚡️ 渲染与性能

* **高性能实时渲染管线** - 基于 Bevy 引擎，CPU/GPU 渲染支持，保证高帧率与时间一致性
* **多视角观测系统** - 自由视角、第一人称、第三人称视角切换（F3键），并可用 `O` 键把视角移到当前假人；第一人称下右下角附随云台转动的观察小窗

---

### 🔄 近期计划

* **ROS2 自定义相机外参支持**

---

### 🚀 规划中功能

* **多机器人协同仿真**（步兵 / 英雄 / 哨兵）
* **弹道模拟与落点校准验证**
* **相机成像参数模拟**（曝光、白平衡、畸变）
* **多光照条件与环境变化模拟**

---

## 💡 使用说明

### ROS2 接口

**发布话题**

* `/camera_info`
* `/image_raw`（默认；`publish_compressed` 开启时改为发布 `/image_compressed`）
* `/livox/lidar`（`livox_ros.enabled = true` 时）
* `/tf`（`map → odom → gimbal_link → muzzle_link` / `camera_optical_frame`，另含能量机关与装甲板坐标系）
* `/gimbal_pose`
* `/odom_pose`
* `/muzzle_pose`
* `/camera_pose`
* `/simulator/marker`（装甲板可视 Marker，`ns = "armors"`）
* `/simulator/tech_core/state`（科技核心状态 JSON，20 Hz）

**订阅话题**

* `/rm_gimbal/cmd`（`rm_interfaces/msg/GimbalCmd`：`yaw` / `pitch` / `distance` / `fire_advice` 等）

### Talos 共享内存接口

**发布姿势**（零拷贝共享内存）

* `odom` - 底盘里程计姿势
* `gimbal` - 云台旋转姿势
* `muzzle` - 枪口偏移姿势
* `camera` - 相机外参姿势
* `chassis_observation` - 底盘运动真值（机体速度、`wz`、四轮转速、加速度、`rpy`、陀螺/加计）

**订阅命令**

* `gimbal_cmd` - 云台控制命令（含开火建议）

---

### 控制方式

左下角键位面板（`F1` 开关）会随输入设备、视角与自瞄状态自动切换显示内容；下表与
`src/systems/help.rs` 的键位表保持一致。

#### 己方 Infantry

| 功能         | 按键                |
|------------|-------------------|
| 前后左右移动     | `W` `S` `A` `D`   |
| 底盘自旋（小陀螺）  | `Q`（切换开/关）        |
| 云台水平 / 俯仰  | `←` `→` / `↑` `↓` |
| 加速（×2）     | `Shift`（左）        |
| 发射 17mm 弹丸 | `Space`（按住连发）    |
| 发射飞镖       | `G`               |

#### 假人 Infantry

| 功能         | 按键                        |
|------------|---------------------------|
| 前后左右移动     | `I`（前）`K`（后）`J`（左）`L`（右） |
| 底盘自旋（小陀螺）  | `U`（切换开/关）                 |
| 底盘横滚       | `[` / `]`                 |
| 底盘俯仰       | `;` / `'`                 |
| 云台水平（yaw）  | `C` / `B`（按住左 `Shift` 时禁用） |
| 云台俯仰（pitch） | `F` / `V`                 |
| 加速（×2）     | `Shift`（右）                |

**Tab**：切换受控假人，在多个假人之间循环（场景中只有一个假人时不生效）。

#### 自由视角

| 功能    | 操作              |
|-------|-----------------|
| 移动相机  | `W` `A` `S` `D` |
| 上升/下降 | `N` / `J`       |
| 转向    | 移动鼠标（无需按住按键）    |

> 自由视角下 `W` `A` `S` `D` 只驱动相机，机器人平移被禁用；方向键云台、`Q`、`Tab`、`F3`、`F5` 仍然有效。

#### 全局按键

| 功能                            | 按键                 |
|-------------------------------|--------------------|
| 键位面板开关                        | `F1`               |
| 截图（输出 `./screenshot-N.png`）   | `F2`               |
| 切换视角样式（自由 / 第一人称 / 第三人称）      | `F3`               |
| 视角跟随目标（己方 / 当前假人）             | `O`                |
| 自瞄订阅开关                        | `F5`               |
| 第一人称小窗开关                      | `F6`               |
| 调试循环（装甲贴纸 / 前哨站旋转模式 / 科技核心阶段） | `Shift` + `C`      |
| 生成 UAV（1 秒冷却）                 | `P`                |

#### 手柄（Xbox 布局）

| 功能          | 按键                |
|-------------|-------------------|
| 移动 / 云台     | 左摇杆 / 右摇杆         |
| 加速          | `L3`              |
| 云台微调        | `LT`              |
| 自瞄（按住）      | `RT`              |
| 己方底盘自旋      | `LB`              |
| 假人底盘自旋      | `Y`               |
| 假人底盘横滚 / 俯仰 | `R3` + 右摇杆        |
| 假人移动        | 十字键               |
| 开火 / 飞镖     | `RB` / `X`        |
| 切换假人 / 切换视角 | `Start` / `View`  |
| 切换视角跟随目标    | `A`               |
| 第一人称小窗开关    | `B`               |

> 键盘与手柄按「最后使用者」切换：最后产生输入的设备决定面板显示哪套键位。

---

### 键位提示面板

* **F1**：开关左下角键位面板；收起后左下角只保留 `[F1] CONTROLS` 提示
* 面板内容随状态实时切换：输入设备（键盘 / Xbox 手柄）、视角、视角跟随目标、自瞄开关、
  己方与假人底盘自旋状态、命中统计（发射数 / 命中数 / 命中率）
* 自瞄开启时云台由求解器的 `fire_advice` 接管，面板会以高亮提示替代手动云台按键说明

---

### 视角切换

* **F3**：按 `自由视角 → 第一人称 → 第三人称` 循环切换

    * 自由视角：全局观察，适合算法调试（移动鼠标转向，`W` `A` `S` `D` 驱动相机）
    * 第一人称：操作手视角，相机跟随云台
    * 第三人称：机器人行为分析，相机跟随底盘
* **O**（手柄 `A`）：切换视角跟随目标——己方机器人 ↔ 当前假人（配合 `Tab` 选假人，
  即可把视角依次挪到任意假人身上；`F3` 的视角样式保持不变）
* 自由视角下 `O` 只切换目标，切回跟随模式后才生效
* 切到假人时，相机画面（含 ROS2/Talos 采集的图像）随之来自假人相机；
  `/gimbal_pose`、`/odom_pose` 等位姿话题仍发布己方机器人，便于对照

---

### 第一人称小窗（随云台转动的观察视角）

主视角为**第一人称**时，右下角会出现一个观察小窗：相机挂在**云台后方**，**朝向与第一人称完全一致**
（偏航/俯仰都跟随），所以转云台、压仰角时小窗视线同步转动——小窗里既能看到机器人姿态，也能看到
你正在瞄的方向。自由视角与第三人称下自动隐藏，小窗不渲染（不额外占用帧时间）。小窗跟随的机器人与
主视角一致：按 `O` 切到假人时，小窗同步切到该假人的云台。**`F6`（手柄 `B`）可随时开关小窗**，
不需要重启或改配置。

小窗的机位由 `pip_offset` 决定（机器人本体坐标系下的 `[横向, 上, 后]`，其中「后」是相对**云台水平朝向**
而言，所以云台转、机位跟着绕，但压仰角时不会钻进地面）。默认 `[0, 1.0, 4.5]` 时机器人位于画面下方
约 12°，构图稳定；把 `up` 调大、`behind` 调小会让视角更俯视。注意云台俯仰到接近极限时，因为视线
完全跟随云台，机器人会移出画面。

小窗本质是「多渲染一个视角」，在可见期间会多一次场景渲染（含阴影）。为此默认做了三件事：

* `pip_refresh_divisor = 2`：小窗**每 2 帧才渲染一次**，可见期间平均只多出半个视角的开销
  （设为 `1` 恢复逐帧；设大一些更省，代价是小窗画面更新变慢）
* `pip_far = 60.0`：小窗相机远平面只到 60 m，裁掉远处场景
* 尺寸降到 320×180，且关闭 MSAA 与后处理；隐藏时相机整体 `inactive`，完全不产生开销

相关配置（`config.toml` → `[camera]`）：

| 配置项                   | 默认值              | 说明                          |
|-----------------------|------------------|-----------------------------|
| `pip_enabled`         | `true`           | 小窗总开关（运行期用 `F6` 切换）          |
| `pip_size`            | `[320.0, 180.0]` | 小窗渲染目标尺寸（px，改动需重启）          |
| `pip_offset`          | `[0.0, 1.0, 4.5]`| 小窗机位 `[横向, 上, 后]`（相对云台水平朝向）   |
| `pip_margin`          | `16.0`           | 距右下角的边距（px）                 |
| `pip_opacity`         | `0.8`            | 小窗不透明度，`0.0`~`1.0`（0.8 ≈ 二成透视） |
| `pip_refresh_divisor` | `2`              | 每 N 帧渲染一次小窗（`1` = 逐帧）        |
| `pip_far`             | `60.0`           | 小窗相机远平面（m，改动需重启）            |

> `pip_enabled` / `pip_offset` / `pip_margin` / `pip_opacity` / `pip_refresh_divisor` 支持热重载；
> `pip_size` 与 `pip_far` 在启动时确定，需重启生效。

---

### 性能限制（`[performance]` / `[render]`）

模拟器默认**不限帧**：`window.present_mode = "auto_no_vsync"` 是给 Talos/ROS2 离线采集用的，
它要的是「能跑多快跑多快」。需要给主循环或渲染负载设上限时，用下面两组开关（全部支持改
`config.toml` 热重载，无需重启）。

`config.toml` → `[performance]`：

| 配置项              | 默认值   | 说明                                                                         |
|------------------|-------|----------------------------------------------------------------------------|
| `max_fps`        | `0.0` | 主循环帧率上限，`0` = 不限帧。>0 时在每帧末尾睡掉剩余时间（钳制到 `1`~`1000`）                         |
| `max_frame_delta` | `0.0` | 单帧 dt 上限（秒），`0` = 引擎默认 `0.25 s`（钳制到 `0.001`~`5.0`）                        |

* `max_fps` 只在 `present_mode` 本身不限帧时才有意义（`auto_vsync`/`fifo` 已由垂直同步压到刷新率）。
* `max_frame_delta` 是**卡顿保护**：长时间卡顿后，玩法逻辑看到的最大 dt 就是它。固定步进由虚拟时间驱动，
  所以它同时限制了单帧最多跑几个固定步（`max_frame_delta * physics.fixed_hz`），避免慢帧雪崩式追帧。
  例如 `fixed_hz = 120`、`max_frame_delta = 0.05` 时，单帧最多 6 个物理步。

`config.toml` → `[render]`（渲染负载降级开关）：

| 配置项                   | 默认值   | 说明                                                            |
|-----------------------|-------|---------------------------------------------------------------|
| `view_distance`       | `0.0` | 主相机与采集相机的远平面（m），`0` = 内置值（主相机 5e8 m、采集相机 10 km）              |
| `shadow_max_distance` | `0.0` | 阴影级联的最远距离（m），`0` = 引擎默认 `150`（钳制到 `1`~`10000`）                 |
| `shadow_cascades`     | `0`   | 阴影级联数量，`0` = 引擎默认 `4`（钳制到 `1`~`4`）                            |

* `view_distance` 只作用于**主相机与采集相机**：小窗仍用 `camera.pip_far`，深度相机仍用 `capture.depth.far`。
* `shadow_max_distance` 调小是最省事的阴影降级手段（阴影贴图覆盖范围变小、级联分辨率变高）；
  `shadow_cascades` 调小则直接减少每个光源的阴影 pass 数。
* 阴影的总开关仍是 `render.shadows`；`render.illuminance` 一并支持热重载。
* 相机内部分辨率缩放（渲染比例）在当前 Bevy 0.19 下没有可用的相机组件（`RenderScale` 已被移除），
  要降分辨率请改 `capture.color.width/height`，或用 `view_distance` / 阴影开关换取帧率。

---

## 🐳 Docker 运行

镜像基于 **ROS 2 Jazzy**（Ubuntu 24.04 noble）多阶段构建，默认打开 `talos` + `ros2`（r2r）特性。

| 文件                 | 说明                                                                     |
|--------------------|------------------------------------------------------------------------|
| `Dockerfile`       | 多阶段：`deps`（系统依赖）→ `chef`（依赖缓存预编译）→ `builder`（真实编译）→ `runtime`（运行时） |
| `.dockerignore`    | 排除 13 GB 的 `target/` 与 `.git`，构建上下文压到约 100 MB（其中约 92 MB 是必需的 `assets/`） |
| `docker-compose.yml` | 常用编排；`daedalus` 服务已带上实测过的 AMD 核显 + Wayland/X11 直通            |
| `docker/`          | 入口脚本、`sanity` 自检、ROS 环境与 IDL 包辅助脚本                                   |

### 构建

```bash
# ros2 + talos（默认特性组合）
docker build -t daedalus:latest .

# 只编译默认的 talos 特性（不涉及 r2r/bindgen，最快的可用组合）
docker build --target runtime-lean --build-arg CARGO_FEATURES=talos -t daedalus:lean .

# 额外打开 ffmpeg（talos_gimbal_mock_server 需要 talos + ffmpeg）
docker build --build-arg CARGO_FEATURES='ros2 ffmpeg' -t daedalus:ffmpeg .
```

> `--features ros2` 是**叠加**的：默认特性 `talos` 仍然生效，所以 `CARGO_FEATURES=ros2` 得到的是 talos + ros2。
> 构建过程会把 `rm_interfaces/`（仓库自带的 ament 包，不在 `ros:jazzy` 基础镜像里）用 colcon 编译并 source，
> 否则 r2r 生成不出 `r2r::rm_interfaces::msg::GimbalCmd`。

### 运行前的自检

不需要显卡，可在任意机器上确认镜像可用性（动态库解析 + ROS 环境 + Vulkan ICD）：

```bash
docker run --rm daedalus:latest sanity
```

`sanity` 会检查 `ldd` 是否有 `not found`、`rm_interfaces/msg/GimbalCmd` 是否可见。
构建时也已内置同样的 `ldd` 门禁：动态库解析不了会直接构建失败。

### 运行模拟器

运行需要 **Vulkan GPU** 和一块显示器（容器与宿主机共用内核，Docker 无法凭空提供显卡）。
下面的 Intel / AMD 命令已实测通过：容器内拿到的是真实核显，窗口正常出现。

```bash
docker run --rm -it \
  --device /dev/dri:/dev/dri \
  --group-add video --group-add render \
  -e DISPLAY="$DISPLAY" \
  -e WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
  -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY" \
  --network host \
  daedalus:latest
```

> `XDG_RUNTIME_DIR` 必须覆盖成**宿主机**的值：镜像里默认是 `/tmp/runtime-root`，那里没有宿主机的
> Wayland socket。只挂载 `wayland-0` 这一个 socket 就够了，不必把整个 `$XDG_RUNTIME_DIR` 挂进去。
> 启动日志出现 `AdapterInfo { name: "… (RADV …)", backend: Vulkan }` 说明走的是真显卡；
> 若显示 `llvmpipe` 则说明退回了软件渲染。

`/dev/dri` 是否存在取决于**宿主机**，容器内看不到设备通常不是 Docker 的问题，而是设备节点本身
不存在、当前用户不在 `video`/`render` 组，或调用方被沙箱挡住了 `ls /dev/dri`。先确认：

```bash
ls -l /dev/dri/            # 应能看到 card*/renderD*
vulkaninfo --summary | grep -E 'deviceName|driverName'
```

NVIDIA 需要 `nvidia-container-toolkit`，把上面的 `--device /dev/dri` 换成 `--gpus all`（或在 compose 里用
`deploy.resources.reservations.devices` 的写法）。`docker-compose.yml` 里 (a) 段是已验证的 Intel/AMD 配置，
(b)/(c) 是 NVIDIA 与纯软件渲染的模板。

没有显卡时可以用 **Mesa lavapipe 软件渲染**兜底（能启动，帧率很低）：

```bash
docker run --rm -it --network host \
  -e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.json \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  daedalus:latest
```

> 注意 `VK_ICD_FILENAMES` 是**硬覆盖**而不是候选列表：只在确实没有显卡时才设置它，
> 否则会把真实显卡驱动屏蔽掉。noble 上 lavapipe 的文件名是 `lvp_icd.json`（**没有** `.x86_64` 后缀）。

### docker compose

```bash
docker compose build daedalus                    # 构建（无需显卡）
docker compose run --rm daedalus sanity          # 无头自检
docker compose run --rm daedalus /app/daedalus   # 开窗口（服务里已配好 GPU/显示直通）
```

> `daedalus` 服务的默认 `command` 是 `sanity`，所以 `docker compose up daedalus` 只会跑自检然后退出；
> 要出界面得显式给命令，像上面第三行那样。

`config.toml` 以只读方式挂载进容器，配合 `notify` 热重载可以直接改参数而不用重建镜像。

### ROS2 模式下加载前哨站会崩溃（已修）

`ros2` 特性会注册 `capture_rune`（`src/ros2/plugin.rs`），它为每个 `ArmorRoot` 发布 TF / Marker 时
无条件取 `*CENTER` 子节点。但资产里只有 `GROUND.glb`（`_ARMOR_CENTER`）、`HERO.glb`
（`1__L_ARMOR_CENTER`）和 `vehicle.glb` 有 `*_CENTER`，**`OUTPOST.glb` 的 `A..F_ARMOR_ROOT`
根本没有**——于是窗口创建后约 1 秒就
`panicked at src/ros2/plugin.rs: called Option::unwrap() on a None value`，退出码 101。

`default`/`talos` 构建不注册这个系统，所以该缺陷只在 `CARGO_FEATURES=ros2` 的镜像里暴露。
现在改为查不到时回退到装甲根节点自身的 `GlobalTransform`：缺一个可选的中心节点不该拖垮整个模拟器。

### 关于动态链接

Bevy 以 `dynamic_linking` 打开，最终二进制**不会**静态包含 Bevy，而是依赖带哈希后缀的
`libbevy_dylib-<hash>.so`（以及 rustc 的 `libstd-<hash>.so`）。`Cargo.toml` 没有生成 rpath，
所以 `runtime` 阶段会把这两个 `.so` 放进 `/app/lib` 并设置 `LD_LIBRARY_PATH`；
它们的哈希与工具链相关，构建时用通配符从**同一个构建阶段**取出，不做硬编码。

---

## 📝 项目信息

* **作者**：Blackjack200
* **团队**：Actor&Thinker 战队
* **技术栈**：Rust · Bevy · ROS2(r2r) · Talos IPC
* **交流方式**：GitHub Issues / Pull Requests
* **开源协议**：AGPL v3

---

## 🌄 演示

<div align="center">
    <img src="demo.png" width="75%">
</div>

---

## 📜 开源协议说明

本项目采用 **AGPL v3** 协议。

我们选择开放仿真基础设施，是因为 RoboMaster 视觉算法的发展依赖于**可复现的实验环境**。
通过开放核心能力，希望为社区提供一个可靠的起点，让更多战队能够在此基础上进行验证、扩展与创新。
