# pure-nav 项目架构说明（Agent 阅读版）

> 面向对象：接手本仓库的人类开发者与 AI Agent。
> 编写依据：仓库当前工作区快照（`git log` 至 `6d3e484`）+ 目录骨架 + 现有源码与配置文件。
> 阅读约定：**文中所有"已实现 / 占位 / 规划"标注均以实际文件内容为准**；凡属推断的内容都显式标注为「推断」，请勿当作既成事实。
>
> **本次修订**：重写 **3.1 数据流设计**（四进程 / 多线程 + 共享内存队列），并同步修正第 1、3、7 节中与当前代码不符的陈述。

---

## 1. 项目定位

- 目标：RoboMaster **哨兵（Sentry）机器人的导航系统**，纯 C++ 实现。
- 语言标准：**C++17**（根 `CMakeLists.txt` 的 `set(CMAKE_CXX_STANDARD 17)`，且 `CMAKE_CXX_EXTENSIONS OFF`）。`.clang-format` 的 `Standard` 为 `Auto`。新代码按 C++17 书写。
- 构建系统：**CMake**（`cmake_minimum_required(VERSION 3.22)`）。
- 进程与 IPC：整机只跑 **四个进程**（`driver` / `navigation` / `autoaim` / `ui`），跨进程数据走 **iceoryx 2.95.8**（submodule `src/thirdparty/iceoryx`）零拷贝共享内存，项目侧由 `src/common/shm` 封装。整机只启动一个 `iox-roudi` 守护进程，内存池由 `config/iceoryx_roudi.toml` 定义。四个进程的职责与目标数据流见 **3.1**。
- 依赖策略：**不依赖 ROS 框架**（项目自身代码中无 `rclcpp` / `ros/ros.h` 引用）。环境里虽装有 `/opt/ros/lyrical`，但 ROS 仅可能用于上位机调试或环境初始化（`README.md` 中的 `fishros` 一键安装脚本属于环境准备，非代码依赖）。
  ⚠️ 但 `src/odometry/Super-LIO` 与 `src/perception/M-detector` 是引入的 **ROS 参考实现**，源码里仍有 `ros/ros.h`、launch、rviz 等，**尚未接入 CMake 构建**；接入前必须先去 ROS 化（见 3.1.7 与 7.4）。
- 其他外部依赖：OpenCV（`find_package(OpenCV REQUIRED)`）、`Threads`、Eigen / Pangolin / matplotlib-cpp / Livox-SDK2 / iceoryx（submodule，产物在 `build/thirdparty/`）；`acados` 已作为 submodule 引入但**尚未接入** `src/thirdparty/CMakeLists.txt`。
- 仓库：`https://github.com/LemperorD/pure-nav.git`，主分支 `main`。

> ⚠️ 当前仓库处于**骨架 + 部分实现**阶段：构建链路（CMake + ctest）已打通；`common_libs`、`common/shm`、`driver/livox_driver` 已有实现并通过测试；`perception`、`odometry`、`planner`、`controller`、`auto_aim`、`ui`、`sim` 仍为占位或未接入的参考代码。**四个业务进程本身尚未实现**——3.1 描述的是确认后的目标架构，不是既成事实。详见第 7 节。

---

## 2. 目录结构总览

```
pure-nav/
├── CMakeLists.txt              # 顶层构建入口（原生 CMake，无自定义宏）
├── README.md                   # 仅含环境安装提示
├── ARCHITECTURE.md             # 本文档
├── .clang-format               # 代码格式化规则（中文注释，LLVM 基线改造）
├── .gitignore                  # 忽略 bin/* 与 build/*
├── .vscode/settings.json       # Conventional Commits scopes 白名单
│
├── bin/                        # 本项目可执行文件输出目录（仅 .gitkeep）
├── build/                      # CMake 构建目录（仅 .gitkeep）
│   └── thirdparty/             #   ★ 第三方库专属区域：build/ install/ deps/ logs/ env.sh
├── config/                     # 运行时参数配置（空，推断用于 yaml/json）
├── autostart/                  # 开机自启脚本（空，推断 systemd/rc.local）
├── assets/img/                 # 静态资源（空）
├── scripts/
│   ├── autoBuild.sh            # 一键构建脚本：第三方库 + 本项目（见 5.2）
│   └── gitPush.sh              # 提交推送脚本（当前内容有误，见 7.3）
│
└── src/
    ├── CMakeLists.txt          # 聚合层：按依赖顺序 add_subdirectory
    ├── app/                    # 【已实现·占位】程序入口 -> bin/pure_nav
    ├── common/                 # 【部分实现】跨模块共享基础设施
    │   ├── shm/                #   共享内存 IPC（include/ + src/，空壳）
    │   ├── common_libs/        #   通用算法库（filters 已实现）
    │   └── type_alias/         #   全局类型别名（纯头文件 INTERFACE 库）
    ├── driver/                 # 【规划】硬件/传感器驱动
    ├── perception/             # 【规划】感知
    ├── odometry/               # 【规划】里程计/状态估计
    ├── planner/                # 【规划】规划
    ├── controller/             # 【规划】控制
    ├── auto_aim/               # 【规划】自动瞄准
    ├── ui/                     # 【规划】调试可视化 -> bin/pure_nav_ui
    ├── sim/                    # 【规划】仿真 -> bin/pure_nav_sim
    ├── thirdparty/             # 第三方库源码（git submodule）+ 只做「接入」的 CMakeLists
    └── test/                   # 【部分实现】单元测试
        ├── test_common/        #   common 模块测试（3 个测试目标 + 迷你框架 test_check.hpp）
        └── test_data/          #   测试数据（filters/*.csv + Python 生成脚本）
```

### CMakeLists 的分布规则（重要）

**每个 `src` 直接子目录只有一个 `CMakeLists.txt`，到此为止，不再向下拆分。**

- `src/common/` 下的 `type_alias`、`common_libs`、`shm` 三个目标**全部写在 `src/common/CMakeLists.txt` 里**，没有 `src/common/shm/CMakeLists.txt`。
- 全仓库共 **14 个** `CMakeLists.txt`：根目录 1 个 + `src/` 1 个 + 12 个模块各 1 个。
- **不使用任何自定义宏 / 函数 / `include()`**：只用 `add_library`、`add_executable`、`target_*` 等标准命令，任何人（和 Agent）都能直接读懂。

### 关键事实

| 项 | 状态 |
|---|---|
| 骨架目录 | `src/` 下所有模块目录**均已放入 CMakeLists.txt**，因此已随 git 跟踪，clone 后骨架完整 |
| `config/`、`autostart/`、`assets/img/` | 仍为空目录，**未被 git 跟踪**（需要时补 `.gitkeep`） |
| 构建链路 | 已打通（配置 + 构建 + ctest 均已验证，详见 5.1） |

> Agent 注意：`config/`、`autostart/`、`assets/img/` 这些空目录不会随 `git clone` 复制，需要时先 `mkdir -p` 或补 `.gitkeep`。（`src/test/test_data/` 已放入真实数据文件，不再是空目录。）

---

## 3. 模块职责与预期接口

下面是每个模块的职责定义。**加「推断」的条目是基于 RoboMaster 哨兵系统的常规分层给出的设计意图，尚未有代码支撑**；Agent 实现时应与用户确认接口，不要自行假定。模块被装进哪个进程、通过哪些 topic 交互，见 **3.1 数据流设计**（已确认的目标架构）。

| 模块 | 职责 | 预期上游 | 预期下游 | 状态 |
|---|---|---|---|---|
| `app` | 四个进程入口、模块装配、主循环/线程编排、生命周期管理 | 命令行 / `config/` | 全部模块 | 占位：`pure_nav.cpp` 只打印 Hello；`driver/navigation/auto_aim/ui.cpp` 为空文件且引用了已删除的旧 `SharedMemory` API，**未接入 CMake** |
| `common/shm` | 共享内存消息总线：iceoryx 运行时 / 发布 / 订阅封装、主题与内存池约定、POD 消息约束 | driver、app | 全部进程 | **已实现**：`shm.hpp`（`ShmRuntime` / `ShmPublisher<T>` / `ShmSubscriber<T>`）+ `shm_payload.hpp`（`FixedVector`），`test_shm` 与 `test_shm_cross_process` 通过 |
| `common/common_libs` | 通用算法：滤波器（滑动窗口均值/中值/一阶低通/一维 KF）、数学工具等 | — | 所有模块 | filters 与 `pure_math_tools` 已实现并测试通过 |
| `common/type_alias` | 全局类型别名、单位约定、枚举 | — | 所有模块 | 空 |
| `driver` | 独占硬件：雷达 / 相机 / 串口 / CAN 的采集，以及执行器指令下发；**不含算法** | 硬件 | shm（点云/图像/裁判/云台状态） | **部分实现**：`livox_driver` 已实现并通过离线测试；`serial_driver` 为空文件；相机（hik）驱动未写 |
| `perception` | 装甲板/目标检测、点云或图像处理、动态障碍检测 | shm（点云/图像） | shm（障碍物）、auto_aim | 未接入：`perception/M-detector` 为引入的 ROS 参考实现 |
| `odometry` | 里程计解算、状态估计、坐标系变换 | shm（点云/IMU） | shm（位姿） | 未接入：`odometry/Super-LIO` 为引入的 ROS 参考实现 |
| `planner` | 全局/局部路径规划、代价地图、行为决策 | shm（位姿/障碍物/决策） | shm（路径） | 空 |
| `controller` | 轨迹跟踪、底盘速度/舵轮控制、PID 等 | shm（位姿/路径） | shm（底盘指令）、driver | 空 |
| `auto_aim` | 装甲板检测、目标预测、弹道解算、火控 | shm（图像/位姿/云台状态） | shm（云台指令/开火/瞄准状态）、driver | 空 |
| `ui` | 只读可视化、参数在线调节、日志呈现 | shm（全部 topic） | 人 | 空 |
| `sim` | 仿真环境、离线回放、算法验证 | 测试数据 | planner、controller | 空 |
| `thirdparty` | 第三方库接入层（源码为 submodule，编译产物在 `build/thirdparty/`） | — | 全部模块 | 5 个 submodule：Livox-SDK2 / Pangolin / matplotlib-cpp / acados / iceoryx；其中前四与 iceoryx 由 `scripts/autoBuild.sh` 独立编译，`src/thirdparty/CMakeLists.txt` 只做接入（acados 未接入） |
| `test` | 单元测试与测试数据 | 被测模块 | CI/开发者 | 6 个目标：`test_common`、`test_filters_data`、`test_filters_consumer`、`test_livox_driver`、`test_shm`、`test_shm_cross_process` |

### 3.1 数据流设计（四进程 / 多线程 / 共享内存队列）

> **状态：设计稿，尚未实现。** 本节是确认后的目标架构，不是当前代码的既成事实。
> 支撑现状：`common/shm` 已实现 iceoryx 封装并有跨进程测试；`driver/livox_driver` 已实现；
> `config/iceoryx_roudi.toml` 已存在；四个进程入口 `src/app/{driver,navigation,auto_aim,ui}.cpp` 为空文件且未接入 CMake。
> 数据流中的模块库（`perception` / `odometry` / `planner` / `controller` / `auto_aim`）目前是占位库。
>
> 标注约定：**【设计】**= 目标设计；**【已实现】**= 仓库中已有可用代码。

#### 3.1.1 进程划分

整机**只跑四个进程**，进程内再按职责分线程。**跨进程通信一律走 iceoryx 共享内存；进程内部优先用进程内队列。**

| 进程 | 职责边界 | 内部线程 | 状态 |
|---|---|---|---|
| `driver` | **独占全部传感器与执行器通道，不含任何算法** | livox-点云组帧、livox-IMU 分发、hik-取流、serial-rx、serial-tx + 命令看门狗 | 雷达驱动【已实现】；串口/相机【设计】 |
| `navigation` | 定位、感知、决策、规划、控制 | 定位、感知、决策、规划、控制（另需 IMU 定序 intake 线程） | 【设计】 |
| `autoaim` | 装甲板检测、目标预测、弹道解算、火控 | 检测、预测、弹道解算、火控 | 【设计】 |
| `ui` | 只读可视化与在线调试 | 1 个 WaitSet 收数线程 + 1 个渲染线程 | 【设计】 |

三条硬约束：

1. `driver` **不实现算法**。它是最关键的进程（传感器全在这里），必须足够简单、可长时间稳定运行。
2. `ui` **严格只读**，且其崩溃或卡顿**不得影响任何其它进程**（实现方式见 3.1.4 规则 3）。
3. 进程间依赖**单向无环**。唯一的逻辑环 `navigation(决策) ↔ autoaim(火控)` 必须在设计上打破，见 3.1.6。

#### 3.1.2 进程间拓扑与 Topic 总表

```
   ┌──────────────────────────────── driver ────────────────────────────────┐
   │  独占硬件，不含算法                                                     │
   │  发布：sensor/lidar/points、sensor/lidar/imu、sensor/camera/image、     │
   │        sensor/referee、sensor/gimbal_state                             │
   │  订阅：cmd/chassis、cmd/gimbal、cmd/fire        （+ 命令看门狗）        │
   └───┬───────────────────────┬──────────────────────────────▲────────────┘
       │ 点云 / IMU / 裁判       │ 图像 / 云台状态               │ 指令
       ▼                       ▼                              │
   ┌────────────────────┐  ┌─────────────────────┐             │
   │    navigation      │  │       autoaim       │             │
   │  定位 感知 决策     │  │  检测 预测 弹道 火控 │─────────────┘
   │  规划 控制          │  │                     │
   └──┬──────────────▲──┘  └──────────▲──────────┘
      │ state/pose   │                │ state/pose
      │              └── state/aim ───┘  （autoaim → navigation 决策）
      │
      │ state/* 与 sensor/* 的全部 topic（只读订阅）
      ▼
   ┌──────────────── ui ─────────────────┐
   │  WaitSet 收数 → 最新值快照 → 渲染    │
   └─────────────────────────────────────┘
```

**Topic 总表**（topic 名一旦确定即为跨进程 ABI，改名需同步四个进程）：

| Topic | 发布者 | 订阅者（建议队列深度） | sizeof 量级 | 频率 |
|---|---|---|---|---|
| `sensor/lidar/points` | driver | 感知(3)、定位(3)、ui(1) | ~400 KB | 10 Hz |
| `sensor/lidar/imu` | driver | 定位(16)、ui(1) | 56 B | 200 Hz |
| `sensor/camera/image` | driver | 检测(2) | 1.3–6 MB | 30–100 Hz |
| `sensor/referee` | driver | 决策(2)、ui(1) | ~200 B | 10–50 Hz |
| `sensor/gimbal_state` | driver | 火控(2)、ui(1) | 64 B | 100–1000 Hz |
| `state/pose` | 定位 | 规划(1)、控制(1)、autoaim(2)、ui(1) | ~200 B | 200 Hz |
| `state/obstacles` | 感知 | 规划(1)、ui(1) | 1–100 KB | 10 Hz |
| `state/decision` | 决策 | 规划(1)、autoaim(1)、ui(1) | ~200 B | 10 Hz |
| `state/plan` | 规划 | 控制(1)、ui(1) | 1–10 KB | 20 Hz |
| `state/aim` | autoaim | 决策(1)、ui(1) | ~200 B | 50–100 Hz |
| `cmd/chassis` | 控制 | driver(1) | 64 B | 500 Hz |
| `cmd/gimbal` | 火控 | driver(1) | 64 B | 200–500 Hz |
| `cmd/fire` | 火控 | driver(1) | 32 B | 事件 |
| `debug/image`（可选） | autoaim | ui(1) | 200–300 KB | 10 Hz |

两条命名与所有权规则：

- **每条 topic 只有一个发布者。** 底盘指令只由 `控制` 发，云台/开火只由 `火控` 发；不要让两个进程抢同一条 topic。
- **高频/大消息不共享给 ui 原始数据。** 原始图像只给 `检测` 订；ui 看 autoaim 发布的降采样 `debug/image`。原因见 3.1.4 规则 3。

#### 3.1.3 navigation 内部不是"流水线"，而是多速率 DAG

**这是本节最容易搞错的一点。** 感知/定位/规划/控制/决策**不能串成 A→B→C 的流水线**，原因有三：

1. **感知与定位是并行兄弟**：都直接吃点云，互不依赖。串起来会白等一次定位耗时（20–50 ms）。
2. **决策不在数据流中间**：它是低频策略节点，吃裁判系统 + 位姿 + 障碍 + autoaim 状态，输出"打还是走、打谁"，去喂**规划**。
3. **速率差一到两个数量级，且触发源不同**。`控制`必须每 2 ms 出一次指令——**哪怕规划 100 ms 没出新路径也要继续跑**；写成"规划出结果才跑控制"会让一次规划卡顿直接导致停车。

```
   sensor/lidar/imu ─────────────┐
                                 ▼
   sensor/lidar/points ──┬──► [定位] ──► state/pose (200Hz) ─────────────► [控制] ──► cmd/chassis
                         │                                 ▲                ▲
                         └──► [感知] ──► state/obstacles ──┼──► [规划] ─────┘
                                                           │    state/plan
   sensor/referee ──┐                                      │
   state/aim ───────┴──► [决策] ──► state/decision ────────┘
```

> 注：`规划`、`控制`、`决策` 都直接读**最新的** `state/pose`；不要用"定位输出唤醒规划"这种串联触发。

**各阶段的触发源、速率与输入语义**（每个线程必须明确这三件事，再加一个超时兜底）：

| 阶段 | 触发源 | 周期/频率 | 输入读取语义 | 输出 | 输入陈旧时的行为 |
|---|---|---|---|---|---|
| 定位 | 点云帧（配准）+ IMU（传播） | 100 ms / 5 ms | 点云逐帧；IMU 全量定序 | `state/pose` | — |
| 感知 | 点云帧 | 100 ms | 点云逐帧 | `state/obstacles` | — |
| 决策 | 定时器 | 100 ms | 全部读最新值 | `state/decision` | 任一输入陈旧 → 保守模式 |
| 规划 | 定时器 | 50 ms | 全部读最新值 | `state/plan` | 障碍陈旧 → 沿用旧图并降速（**不得当作无障碍**） |
| 控制 | **硬定时器** | 2 ms | 全部读最新值 | `cmd/chassis` | 位姿超时 → 急停（不能拿旧位姿继续跑） |

实现要点：

- **IMU 必须由独立 intake 线程持续 drain 进进程内定序环形缓冲**，定位线程只在点云到来时去读。不要让定位线程自己取 IMU：它一次配准 20–50 ms，期间 IMU 会积压甚至被丢（iceoryx 默认丢最旧）。
- 进程内队列复用 `driver/livox_driver/include/livox_driver/blocking_queue.hpp`（预分配环形缓冲、满则丢旧、`push` 永不阻塞）。建议把它**提升到 `common`**，否则 navigation 用它会形成"感知依赖驱动实现细节"的怪依赖。
- 定位线程**独占一个核**，并显式限制 Super-LIO 内部的 OpenMP 线程数，否则它会把控制线程挤爆。控制线程高优先级 + 独立核。

#### 3.1.4 传输规则：跨进程 vs 进程内

**规则 1：跨进程必走 shm；阶段输出统一发 shm，进程内下游与 ui 共用同一次发布。**

- 凡是 ui 要看的阶段输出，**一律提升为 shm topic**。一次 `publish`，进程内的下游线程作为一个订阅者、ui 作为另一个订阅者，零拷贝共享同一块 chunk。这样 ui 看到的**就是**流水线真实数据，不存在"影子通道与真实通道不一致"；将来把感知拆成独立进程也只改订阅、不改结构。
- **严格序列流例外**：只有 IMU 定序缓冲走进程内阻塞队列，绝不能因为"ui 也要看 IMU"就把定序逻辑搬到 shm 上。
- **高频内部中间量不进 shm**：LIO 的 IMU 预积分状态、控制器的积分项、火控的滤波器内部状态，都是纯进程内私有。

**规则 2：每阶段自己定节奏 + 读最新值，输入语义用队列深度表达。**

- `queue_capacity = 1` ⇒ 真正的**最新值邮箱**（配合 iceoryx 默认的 `DISCARD_OLDEST_DATA`，见 vendored 源码 `iceoryx_posh/include/iceoryx_posh/popo/subscriber_options.hpp`）。适合控制、规划、ui、火控。
- `queue_capacity = 3~5` ⇒ **容忍偶发慢帧**，适合感知、定位这类"每帧都想处理"的阶段。
- 在 `ShmSubscriberConfig` 里显式设 `queue_capacity`，**不要用默认值**（默认是 iceoryx 的 `MAX_CAPACITY`，会让慢消费者钉住过多 chunk）。

**规则 3：ui 绝不能出现在关键路径的资源预算里。**

iceoryx 的 mempool **按大小分档、所有 topic 共享**；订阅者队列里每一项都是一个指向该池 chunk 的引用，**一个卡住的订阅者会一直占着它队列深度那么多的 chunk**。若 ui 以较大深度订阅点云或原始图像，ui 一卡（渲染一帧几十毫秒很正常）就可能耗干 chunk 池 → driver 的 `publish` 返回 `OUT_OF_CHUNKS` → **整个系统帧率被调试 UI 拖垮**；更糟的是会影响 `检测` 的帧率，进而恶化瞄准。因此：

- ui 对所有 topic 一律 `queue_capacity = 1`；
- 原始图像（1.3–6 MB）**只给检测订**，ui 只看 `debug/image`（降采样到 640×480 灰度约 300 KB、10 Hz）；
- 点云也建议 ui 自行抽帧或降采样。

#### 3.1.5 消息类型与内存池

**消息类型约束（沿袭 `shm.hpp` 的 `static_assert`）**：必须可平凡复制、可默认构造、不含进程内地址（禁用 `std::string` / `std::vector` / 裸指针 / 虚函数 / Eigen 动态类型）。变长数据用 `shm_payload.hpp` 的 `FixedVector<T,N>`。

> 现状：`livox_driver::PointCloudFrame` 目前用 `std::vector<PointXYZI>`，**不可直接发布**；需改成 `FixedVector` 版本，才能让组帧线程用 `publish(fill)` 直接在 shm chunk 里解码写入，省掉每帧一次 400 KB 拷贝。

**每条消息头部至少带**：`layout_version`、`seq`（单调递增）、`t_sensor_ns`、`t_publish_ns`。用途：`seq` 让消费者判断丢了多少帧（`hasMissedData()` 只能说"丢过"）；时间戳让每个消费者做陈旧判断；定位输出额外带 `scan_seq`，标明该位姿对应第几帧点云，供感知标注障碍物时对齐。

**内存池分档**（对应 `config/iceoryx_roudi.toml`，多进程下由 RouDi 配置决定）：

| 档位 | 承载 | 建议 count | 说明 |
|---|---|---|---|
| 256 B | IMU、`cmd/*`、`gimbal_state`、`referee` | 2000–4000 | 高频、多订阅者 |
| 4 KB | `state/pose`、`state/decision`、`state/aim`、小 `state/plan` | 1000 | 200 Hz 的 pose 是主要占用 |
| 128 KB | `state/obstacles`、大 `state/plan` | 100–200 | |
| 512 KB | 点云（400 KB）、`debug/image`（300 KB） | 40–50 | **两者共用同一档，count 要按二者之和算** |
| 2 MB（**建议新增**） | 海康黑白相机（1280×1024 ≈ 1.3 MB） | 12 | 现有档位 512K→4M 之间为空档，1.3 MB 图会被塞进 4 MB chunk，浪费约 3 倍共享内存 |
| 4 MB | 彩色 / 更大分辨率相机 | 8–12 | |
| 8 MB | 1920×1080 彩色 | 6–8 | 用不到就删除该档 |

注意事项：

- **chunk_count 是全局预算，不是单条 topic 的队列深度**。它要覆盖"所有 topic 的订阅者队列深度之和 + 正在被算法持有的 chunk"。配小了的表现就是 `publish` 返回 `OUT_OF_CHUNKS`。
- 相机档位是 `/dev/shm` 占用的主要来源。当前 `config/iceoryx_roudi.toml` 注释自述约 **254 MB** 预分配；上机前按 `分辨率 × 通道 × 帧率 × 在飞块数` 重算，Jetson 这类机器上砍到 64–96 MB 通常够用，并先看 `df -h /dev/shm`。

#### 3.1.6 生命周期、看门狗与单写者仲裁

**启动顺序**：`iox-roudi` → `driver` → `navigation` → `autoaim` → `ui`。

- `RouDi` 是单点故障：它挂了四个进程的收发全废。用 systemd 管理并 `Restart=always`；driver 要能感知 RouDi 重启并重建端口。
- 后启动的进程会漏掉前面的数据：`state/*` 这类状态量给发布者设 `historyCapacity > 0`，晚加入者能补到最近若干帧；点云/图像漏几帧无所谓。
- driver 采到传感器后不要立刻放开：轮询 `publisher.hasSubscribers()` 或依赖 history 再使能雷达/相机（`shm` README 已记录 `history_capacity = 0` 时首帧可能丢失）。
- **退出顺序**：先停发布 → 排空队列 → 销毁订阅者/发布者 → 最后 RouDi 退出。`shm.hpp` 已警告不要做全局对象（静态析构顺序无法保证）。

**看门狗（四进程架构的安全底线）**：因为感知/定位/规控/决策全在 `navigation` 一个进程里，感知一崩控制链就断，所以必须逐级设置超时兜底：

| 监视者 | 被监视对象 | 超时行为 |
|---|---|---|
| driver | `cmd/chassis` | 100 ms 无更新 → 底盘零速 |
| driver | `cmd/gimbal` / `cmd/fire` | 100 ms 无更新 → 云台保持 / 停火 |
| 控制 | `state/pose` | 200 ms 无更新 → 急停 |
| 规划 | `state/obstacles` | 超时 → 沿用旧障碍图并降速（**不得当作无障碍**） |
| 决策 | 全部输入 | 任一陈旧 → 进入保守模式 |

**`navigation(决策) ↔ autoaim(火控)` 的逻辑环及打破方式**：决策想知道 autoaim 的目标可用性（`state/aim`），而火控又想拿 navigation 的许可（`state/decision`）。进程间异步通信不会造成链接级环，但逻辑上会震荡。两种收敛方式，二选一：

- **推荐（无环）**：autoaim 只做"瞄准服务"，发布 `state/aim`，**不碰扳机**；navigation 的决策是**唯一**开火授权者，直接发 `cmd/fire`。数据流退化为 DAG。
- **备选（火控持扳机）**：火控是 `cmd/fire` 的唯一写者，把 `state/decision::allow_fire` 与自身就绪状态做 AND；同时**决策不要再读 `state/aim`**，避免成环。

无论选哪种，都必须保证**每条 topic 单写者**；driver 分别订阅 `cmd/chassis` 与 `cmd/gimbal`/`cmd/fire` 的设计正是为此。

#### 3.1.7 落地前提

实现 3.1 之前必须先解决以下阻塞项：

1. **去 ROS 化**：`odometry/Super-LIO`（`ROSWrapper.cpp`、`super_lio_node.cpp` 含 `ros/ros.h`）与 `perception/M-detector`（launch/rviz/config）目前是 ROS 参考实现。要把 ROS 消息、参数服务器、TF、可视化替换为项目 POD + shm + 自己的类型。这是最大的一块工作量。
2. **`shm.hpp` 补事件驱动取数**：当前 `waitForData` 是 200 µs 轮询，单这一项引入最多 200 µs 抖动。若云台内环要在 PC 上闭合到 500 Hz–1 kHz，必须补 WaitSet/Listener 接口。（若内环在 MCU 上，PC 侧 100–200 Hz 时轮询可接受。）
3. **消息类型落盘**：新增统一定义 shm 消息的头文件（建议 `common/shm/include/shm_msgs.hpp`），四个进程只许 include 这一份；并同步 `common/type_alias` 的单位与坐标系约定（m / rad / rad/s，`map`/`odom`/`body`/`gimbal` 的旋转方向）。
4. **四个进程入口与 CMake**：`src/app/{driver,navigation,auto_aim,ui}.cpp` 为空文件，其中 `driver.cpp` 还引用了已删除的 `pure::common::SharedMemory` API；`src/app/CMakeLists.txt` 目前只声明了 `pure_nav` 一个可执行目标。
5. **相机/串口驱动**：`driver/serial_driver` 与相机（hik）驱动尚未实现。

---

## 4. 代码组织与命名约定

已从现有文件观察到的**强制约定**（新代码必须遵守）：

| 维度 | 约定 | 依据 |
|---|---|---|
| 目录组织 | 每个模块统一 `include/`（头文件）+ `src/`（实现） | `common/shm`、`common/common_libs` |
| 文件命名 | `snake_case.hpp` / `snake_case.cpp` | `filters.hpp`、`type_alias.hpp` |
| 头文件保护 | `#pragma once` | `filters.hpp` |
| 命名空间 | `pure::<模块名>`，如 `pure::common` | `filters.hpp` 的 `namespace pure::common` |
| 命名空间缩进 | 内缩（`NamespaceIndentation: Inner`） | `.clang-format` |
| 缩进 | 4 空格，禁用 Tab（`UseTab: Never`） | `.clang-format` |
| 行宽 | 200 列（`ColumnLimit: 200`） | `.clang-format` |
| 指针/引用 | 左对齐，如 `int* p`、`const T& x`（`PointerAlignment: Left`） | `.clang-format` |
| 连续赋值/声明 | 要求对齐（`AlignConsecutiveAssignments/Declarations: true`） | `.clang-format` |
| include 排序 | 自动排序（`SortIncludes: true`） | `.clang-format` |
| 注释语言 | 中文 | `main.cpp`、`filters.hpp`、`README.md` |

> 提交前请执行 `clang-format -i <file>`，避免格式噪声污染 diff。

### 4.1 提交信息规范

采用 **Conventional Commits**，scope 白名单由 `.vscode/settings.json` 的 `conventionalCommits.scopes` 控制（目前仅 `common`，新增模块后需同步扩充）。

历史提交示例：

```
feat(common): :sparkles: 增加filters功能
    将使用模板实现多种filter
```

即：`<type>(<scope>): <gitmoji> <中文描述>` + 正文说明。

---

## 5. 构建与运行

推荐使用一键脚本 `scripts/autoBuild.sh`（会自动处理四个第三方库 Livox-SDK2 / Pangolin / matplotlib-cpp / iceoryx，以及本机缺失 C++ 前端的免 root 兜底）：

```bash
# 第三方库 + 本项目（第三方库已就绪时自动跳过）
scripts/autoBuild.sh

# 只构建第三方库 / 只构建本项目
scripts/autoBuild.sh thirdparty
scripts/autoBuild.sh project --tests

# 常用选项与清理
scripts/autoBuild.sh project -j 8 -t Debug --werror
scripts/autoBuild.sh clean        # 清第三方库构建中间产物（保留 install）
scripts/autoBuild.sh distclean    # 清 build/ 与 bin/ 下全部产物
scripts/autoBuild.sh status       # 查看产物状态
scripts/autoBuild.sh help
```

也可以直接用 CMake（前提：`build/thirdparty/install` 已由脚本生成）：

```bash
# 1) 配置（源目录与构建目录分离，不要在仓库根直接跑 cmake）
cmake -S . -B build

# 2) 编译（产物：bin/ 可执行文件，build/lib/ 静态库）
cmake --build build -j$(nproc)

# 3) 运行
./bin/pure_nav

# 可选：调试构建 / 打开测试 / 警告即错误
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake -S . -B build -DPURE_NAV_BUILD_TESTS=ON
cmake -S . -B build -DPURE_NAV_WARNINGS_AS_ERRORS=ON

# 可选：跑单元测试（需先 -DPURE_NAV_BUILD_TESTS=ON）
ctest --test-dir build --output-on-failure
```

| 选项 | 默认值 | 说明 |
|---|---|---|
| `CMAKE_BUILD_TYPE` | `Release` | 哨兵上跑优化版本；调试用 `Debug` |
| `PURE_NAV_BUILD_TESTS` | `OFF` | 构建 `src/test` 下的测试并注册到 ctest |
| `PURE_NAV_WARNINGS_AS_ERRORS` | `OFF` | 追加 `-Werror` |

- `bin/` 为**本项目可执行文件**的统一输出目录（主程序、`ui`、`sim`、测试），`build/lib/` 放本项目中间静态库，二者内容均不入库。
- **第三方库产物单独存放**在 `build/thirdparty/`，与项目模块产物完全分开（见 5.2）。
- `compile_commands.json` 已在顶层开启（生成于 `build/`），供 clangd / IDE / Agent 使用。
- 全局编译标志：`-std=c++11`、静态库 `-fPIC`、可执行 `-fPIE`、`-Wall -Wextra -Wpedantic -Wshadow`。
- `config/` 预期存放运行时可调参数，供程序读取（推断：yaml/json）。
- `autostart/` 预期存放哨兵上电自启脚本（推断：systemd unit 或 shell）。
- 环境要求：CMake ≥ 3.22、支持 C++11 的编译器、已初始化的 `git submodule`（第三方库源码）；Pangolin 另需 Eigen3 / OpenGL / GLEW / X11 等系统库。

### 5.1 验证状态

| 项 | 结果 |
|---|---|
| `cmake -S . -B build`（默认选项） | ✅ 通过（CMake 4.2.3） |
| `cmake --build build` | ✅ 通过，产出 `bin/pure_nav` + `build/lib/libpure_nav_{common_libs,shm}.a` |
| `-DPURE_NAV_BUILD_TESTS=ON` | ✅ 通过，`ctest` 3 个测试全绿：`test_common`（22 用例/66 断言）、`test_filters_data`（CSV golden 比对）、`test_filters_consumer`（下游引用） |
| `-DPURE_NAV_WARNINGS_AS_ERRORS=ON` | ✅ 通过，`-Werror` 正确注入 |
| 真实 C++ 编译 | ✅ 已用免 root 解包的 g++-15（GNU 15.2.0）在 `-Werror` 下完整编译并运行测试；本机**系统级**仍无 `g++`（见 7.2，现已由脚本自动兜底） |
| `scripts/autoBuild.sh thirdparty` | ✅ 三个库全部编译安装到 `build/thirdparty/install`：Livox-SDK2（静态 + 动态）、Pangolin（全部组件 + CMake 包）、matplotlib-cpp（头文件 + CMake 包 + 17 个 examples） |
| `scripts/autoBuild.sh` / `project --tests` | ✅ 第三方库 + 本项目全链路通过，配置摘要显示三个库「已接入」，`ctest` 全绿 |

### 5.2 构建产物布局（第三方库与项目模块分开）

脚本把第三方库和本项目**分别配置、分别编译、分别存放**：

```
build/
├── thirdparty/                     ★ 第三方库专属区域（scripts/autoBuild.sh 管理）
│   ├── build/Livox-SDK2/             各库各自独立的 CMake 构建树
│   ├── build/Pangolin/
│   ├── build/matplotlib-cpp/
│   ├── install/                      统一安装前缀
│   │   ├── include/                  livox_*.h / pangolin/ / matplotlibcpp.h
│   │   ├── lib/                      liblivox_lidar_sdk_*.a/.so、libpango_*.so
│   │   └── lib/cmake/...             PangolinConfig.cmake、matplotlib_cppConfig.cmake
│   ├── deps/                         免 root 补装的构建期依赖（如 libepoxy-dev 头文件）
│   ├── logs/<库名>.log               各库完整构建日志
│   └── env.sh                        source 后可让外部工具找到这些库
├── lib/                           本项目静态库（libpure_nav_*.a）
├── src/  CMakeFiles/ ...          本项目构建树
└── compile_commands.json
bin/                               本项目可执行文件
```

`src/thirdparty/CMakeLists.txt` **只做接入**，不再 `add_subdirectory` 第三方源码：
从 `build/thirdparty/install` 里 `find_package`，并提供统一的 imported target，
所以第三方产物不会混进 `build/lib/`。

| 第三方库 | 接入方式 | 目标名 |
|---|---|---|
| Livox-SDK2 | 手工声明 imported target（上游不导出 CMake 包） | `livox_sdk2` |
| Pangolin | `PangolinConfig.cmake` | `pango_core` / `pango_display` 等（上游未加命名空间） |
| matplotlib-cpp | `matplotlib_cppConfig.cmake` | `matplotlib_cpp::matplotlib_cpp` |

> Pangolin 把 `HAVE_EPOXY` 作为 PUBLIC 编译定义导出，消费方包含 `<pangolin/pangolin.h>`
> 时会间接 `#include <epoxy/gl.h>`；本机只装了 `libepoxy0` 运行库。脚本会把免 root 解包的
> `libepoxy-dev` 头文件与链接库一并补进 `build/thirdparty/install`，因此该前缀是**自包含**的
> ——链接任意 `pango_*` 目标即可编译，无需额外 `-I`。


---

## 6. 新增模块 / 新增源文件的流程（Checklist）

### 6.1 给已有模块加实现（最常见的操作）

1. 新建 `src/<module>/include/`（对外头文件）与 `src/<module>/src/`（实现）。
2. 打开该模块**唯一**的 `CMakeLists.txt`，把 `INTERFACE` 占位库改成 STATIC，并显式列出源文件：
   ```cmake
   add_library(pure_nav_driver STATIC
       src/serial.cpp
       src/referee.cpp)
   ```
   占位库里已经写好了可直接复制粘贴的注释模板。
3. 头文件写 `#pragma once`，代码放入 `namespace pure::<module>`。
4. 其他模块要用你时，在**它的** `CMakeLists.txt` 的 `target_link_libraries` 里加 `pure_nav_<module>`；**不要**用 `#include "../../x.hpp"` 这类相对路径跨模块引用。
5. 用 `clang-format -i` 格式化后按 Conventional Commits 提交。

> ⚠️ 本项目**不做源文件通配（GLOB）**：新增/删除 `.cpp` 必须手工更新 `CMakeLists.txt` 的源文件列表。这是"无宏、纯显式 CMake"的代价，换来的是完全可预测的构建。

### 6.2 新增一个模块

1. `mkdir src/<module>`，新建 `src/<module>/CMakeLists.txt`（照抄任一现有模块）。
2. 在 `src/CMakeLists.txt` 中按依赖顺序加一行 `add_subdirectory(<module>)`。
3. 若该模块在 `src/` 下一级之下还有子功能（如 `common` 那样），**不要**再加下级 CMakeLists，全部写在 `src/<module>/CMakeLists.txt` 里。
4. 更新 `.vscode/settings.json` 的 `conventionalCommits.scopes`。
5. 更新 `ARCHITECTURE.md` 第 2、3、9 节。
6. 用 `clang-format -i` 格式化后按 Conventional Commits 提交。

### 6.3 新增测试

1. 在 `src/test/test_<suite>/` 放测试源码，包含 `test_check.hpp`（src/test 内部共用的迷你测试框架：断言宏 + 失败统计 + `main` 收尾）；
2. 在 `src/test/CMakeLists.txt` 里加 `add_executable` + `target_link_libraries` + `add_test` 三件套（照抄现有的 `test_common`）；
3. 测试若用到 `<cmath>` 的 `sqrt`，记得链接 `libm`（`src/test/CMakeLists.txt` 里的 `PURE_NAV_TEST_EXTRA_LIBS`）；
4. 数据驱动测试把数据放在 `src/test/test_data/`，并通过 `target_compile_definitions(... PURE_NAV_TEST_DATA_DIR=...)` 注入**绝对路径**，避免依赖运行时工作目录（照抄 `test_filters_data`）。

---

## 7. 当前状态与已知问题（Agent 必读）

### 7.1 CMake 构建链路 ✅ 已修复

原先顶层 `CMakeLists.txt` 写的是 `add_subdictionary(src)`（非法命令且拼写错误）、且 `src/CMakeLists.txt` 与各模块 `CMakeLists.txt` 全部缺失。现已全部补齐并验证通过，见第 5 节与第 9 节。

### 7.2 系统缺少 C++ 前端 ✅ 已自动化兜底

```
CMake Error: No CMAKE_CXX_COMPILER could be found.
```

本机 `gcc` 存在，但**没有 C++ 前端**（无 `g++`/`clang++`，`/usr/lib/gcc/*/*/cc1plus` 也不存在），
因此裸 CMake 会在 `project(... LANGUAGES CXX)` 阶段直接失败。推荐直接装：

```bash
sudo apt install g++        # 需要 sudo；装好后脚本会自动优先使用系统 g++
```

> 在拿到 sudo 之前，`scripts/autoBuild.sh` 会**自动免 root 兜底**：`apt-get download
> g++-15-x86-64-linux-gnu` → `dpkg-deb -x` 解包取出 `cc1plus`，再在
> `~/.cache/pure-nav/toolchain` 生成包装脚本 `bin/g++-wrapped`（用 `-B` 指向解包出的
> 前端，并软链系统头文件/`liblto_plugin.so`），最后以 `-DCMAKE_CXX_COMPILER=<包装脚本>`
> 配置。可用 `PURE_NAV_TOOLCHAIN_DIR` 改缓存目录，或用 `PURE_NAV_NO_LOCAL_TOOLCHAIN=1`
> 关闭该行为。这是**环境问题，不是代码问题**；装好系统 `g++` 后脚本会自动切回。

### 7.3 脚本内容错误 🟠

| 文件 | 问题 | 建议 |
|---|---|---|
| `scripts/autoBuild.sh` | ✅ 已重写：支持 `all/thirdparty/project/clean/distclean/status/help` 命令，自动处理第三方库、编译器兜底、陈旧 CMakeCache | 直接 `scripts/autoBuild.sh [命令]`，`-h` 查看帮助 |
| `scripts/gitPush.sh` | 内容与旧的 `autoBuild.sh` 完全相同，实为复制粘贴残留，未做任何 git 操作 | 改为 `git add -A && git commit && git push`（或按需简化） |

### 7.4 其他

- `src/common/common_libs/filters.hpp` 提供模板声明，`filters.cpp` 提供实现 + 对 `float`/`double` 的显式实例化：`SlidingWindowFilter`（滑动窗口均值，别名 `MovingAverageFilter`）、`MedianFilter`、`FirstOrderLowPassFilter`、`KalmanFilter1D`。
- `src/test/test_common/test_filters.cpp`（单元）、`test_filters_data.cpp`（CSV 数据驱动）、`test_filters_consumer.cpp`（模拟下游模块引用）三个目标共用 `test_check.hpp` 里的迷你框架（断言 + 统计 + `main` 收尾，不依赖 GoogleTest/doctest/Catch2），失败时返回非 0，`ctest` 可直接判定。`PURE_NAV_BUILD_TESTS` 在根 `CMakeLists.txt` 中默认 **ON**；`test_livox_driver` 为雷达驱动的离线单元测试；`test_shm` 用进程内 RouDi，`test_shm_cross_process` 用外部 `iox-roudi` + fork 验证真正的跨进程共享内存（后者在 RouDi 不存在时 SKIP，iceoryx 未接入时这两个目标不存在）。
- `src/test/test_data/filters/*.csv` 的 `expected` 列由 `src/test/test_data/generate_filters_data.py` 用**独立**参考实现生成（golden），C++ 侧实测最大误差 ≤ 9e-16。数据文件已提交，**构建不需要 Python**；改数据时重跑脚本即可（见 `src/test/test_data/README.md`）。
- 数据驱动测试用**粗糙度**（一阶差分标准差）而非"与真值的 RMSE"评价降噪效果：平滑滤波器都存在相位滞后，阶跃/正弦上 filtered 的 RMSE 可能反而大于 raw。
- `.clang-format` 的键值写成 `Key:Value`（冒号后缺空格），**当前不是合法 YAML**，`clang-format -i` 会直接报 `not a mapping`。修复前请手工遵守第 4 节格式约定。
- 项目标准已是 **C++17**（见第 1 节），因此 `namespace pure::common {` 这种嵌套命名空间简写是**合法**的；现有代码用的是 `namespace pure { namespace common { ... } }` 全展开写法，新增代码跟随现有风格即可。
- `src/driver` **已经不再是纯占位**：`livox_driver` 有完整实现（SDK 生命周期、点云组帧、IMU 解码、丢包统计）并通过离线测试；`serial_driver` 的 `include/` 与 `src/` 下两个文件都是 **0 字节空文件**；相机（hik）驱动未写。`perception` / `odometry` / `planner` / `controller` / `auto_aim` 仍是 **INTERFACE 占位库**（无实现文件），其 `include/` 目录尚不存在，因此没有任何 `-I` 生效；一旦放入源文件并按 6.1 改成 STATIC 即可。
- `src/perception/M-detector` 与 `src/odometry/Super-LIO` 是**已拉进仓库但未接入构建**的 ROS 参考实现（含 `package.xml` / `launch` / `rviz` / `ros/ros.h`）。接入前必须去 ROS 化，见 3.1.7。
- `src/app/{driver,navigation,auto_aim,ui}.cpp` 中，`navigation.cpp` / `auto_aim.cpp` / `ui.cpp` 为空文件，`driver.cpp` 引用了**已不存在**的 `pure::common::SharedMemory` API（当前 `shm.hpp` 只有 `ShmRuntime` / `ShmPublisher<T>` / `ShmSubscriber<T>`），**该文件无法编译**；`src/app/CMakeLists.txt` 目前也只声明了 `pure_nav` 一个可执行目标。四个进程入口需按 3.1 重写并补齐 target。
- `src/ui`、`src/sim` 尚无源文件，`add_executable` 需要至少一个源文件，因此这两个目标在各自的 `CMakeLists.txt` 中**以注释形式给出模板**，取消注释即可产出 `bin/pure_nav_ui` / `bin/pure_nav_sim`。
- `src/thirdparty/CMakeLists.txt` **只做接入**：从 `build/thirdparty/install` 找预编译的 Pangolin / matplotlib-cpp / iceoryx，并为 Livox-SDK2 声明 `livox_sdk2` imported target；它不编译任何第三方源码（见 5.2）。第三方源码都是 git submodule（Livox-SDK2 / Pangolin / matplotlib-cpp / acados / iceoryx），需 `git submodule update --init --recursive`。其中 **acados 尚未在 `src/thirdparty/CMakeLists.txt` 中接入**。

---

## 8. 给 Agent 的工作约定

**应当：**
- 动手前先读本文件第 7 节：本机**系统级**缺 C++ 前端（见 7.2，可免 root 绕过），业务代码本身没有硬阻塞。
- 保持 C++11、`namespace pure { namespace <模块> {`（C++11 不支持 `namespace a::b` 写法，见 7.4）、中文注释、`#pragma once` 的一致性。
- 新增源码文件后**必须**把它加进所属模块 `CMakeLists.txt` 的源文件列表（本项目不用 GLOB 自动收集）；新增**模块**或新增**跨模块依赖**时同样要改 CMakeLists（见第 6、9 节）。
- 修改后运行 `clang-format -i`（注意 7.4：`.clang-format` 当前是非法 YAML，修好前请手工对齐格式），并按 Conventional Commits 提交。
- 新增/变更模块时同步更新本文件的第 2、3、9 节。
- 涉及模块间接口时**先读 3.1 数据流设计**：topic 名、消息布局、队列语义、单写者归属都在那里定义；需要变更时先与用户确认并同步更新 3.1，不要自创约定。

**不应当：**
- 不要引入 ROS / colcon / ament 或其他重型框架依赖，除非用户明确要求。
- 不要提交 `bin/`、`build/` 内产物。
- 不要把「推断」的模块接口当成既定契约实现。
- 不要在未确认的情况下删除 `src/` 下的模块目录。

---

## 9. CMake 目标与依赖图

### 9.1 目标命名与产物

| 目标 | 类型 | 产物 | 说明 |
|---|---|---|---|
| `pure_nav_type_alias` | INTERFACE | — | 纯头文件模块（无 .cpp 即为 INTERFACE 库） |
| `pure_nav_common_libs` | STATIC | `build/lib/libpure_nav_common_libs.a` | `src/common/common_libs` |
| `pure_nav_shm` | STATIC | `build/lib/libpure_nav_shm.a` | `src/common/shm` |
| `pure_nav_driver` / `_perception` / `_odometry` / `_planner` / `_controller` / `_auto_aim` | INTERFACE（暂） | — | 占位库；加入实现后需**手工**改成 STATIC 并列出源文件（见 6.1） |
| `pure_nav` | EXECUTABLE | `bin/pure_nav` | 主程序（`src/app`） |
| `pure_nav_ui` | EXECUTABLE | `bin/pure_nav_ui` | 待 `src/ui` 放入 `main()` |
| `pure_nav_sim` | EXECUTABLE | `bin/pure_nav_sim` | 待 `src/sim` 放入 `main()` |
| `test_<suite>` | EXECUTABLE | `bin/test_<suite>` | 需 `-DPURE_NAV_BUILD_TESTS=ON` |

### 9.2 依赖图（`target_link_libraries` 的 `DEPS`）

实际声明的依赖边（`A → B` 表示 A 的 DEPS 里有 B）：

| 目标 | DEPS |
|---|---|
| `pure_nav_common_libs` | `pure_nav_type_alias` |
| `pure_nav_shm` | `pure_nav_common_libs` |
| `pure_nav_driver` | `pure_nav_common_libs`, `pure_nav_shm` |
| `pure_nav_perception` | `pure_nav_common_libs`, `pure_nav_driver` |
| `pure_nav_odometry` | `pure_nav_common_libs`, `pure_nav_driver` |
| `pure_nav_planner` | `pure_nav_common_libs`, `pure_nav_odometry`, `pure_nav_perception` |
| `pure_nav_controller` | `pure_nav_common_libs` |
| `pure_nav_auto_aim` | `pure_nav_common_libs`, `pure_nav_perception`, `pure_nav_odometry` |
| `pure_nav`（app） | 上述全部库 |
| `pure_nav_ui` | `pure_nav_shm`, `pure_nav_odometry`, `pure_nav_perception`, `pure_nav_auto_aim`, `pure_nav_planner` |
| `pure_nav_sim` | `pure_nav_common_libs`, `pure_nav_odometry`, `pure_nav_planner`, `pure_nav_controller` |
| `test_common` | `pure_nav_common_libs`, `pure_nav_type_alias` |
| `test_filters_data` | `pure_nav_common_libs`, `pure_nav_type_alias`（+ 编译定义 `PURE_NAV_TEST_DATA_DIR`）|
| `test_filters_consumer` | **仅** `pure_nav_common_libs`（刻意不加 `-I`，用来验证 PUBLIC include 传递与显式实例化符号可链接）|

依赖必须**单向无环**。若两个模块互相需要（典型如 planner ↔ controller 争用路径数据），不要互相 link，正确做法是把共享的数据结构下沉到 `common/type_alias` 或 `common/shm`，让双方都依赖下层。

> 说明：依赖这些库只传递 include 目录与链接关系，**不代表数据流方向**。数据流见 3.1（四个进程通过 `common/shm` 的 topic 交互，`navigation` 内部为多速率 DAG），二者应当一致；若发现不一致，以 3.1 为准调整 DEPS。

### 9.3 每个 CMakeLists 负责什么

| 文件 | 负责的目标 |
|---|---|
| `CMakeLists.txt`（根） | `project()`、C++11、`bin/` 输出目录、编译警告、`find_package(Threads)`、选项、`add_subdirectory(src)` |
| `src/CMakeLists.txt` | 只做 `add_subdirectory` 聚合（12 个模块 + 条件加入 `test`） |
| `src/common/CMakeLists.txt` | `pure_nav_type_alias`、`pure_nav_common_libs`、`pure_nav_shm` **三个目标全在这里** |
| `src/app/CMakeLists.txt` | `pure_nav`（可执行）+ 对全部库的链接 |
| `src/driver/`…`src/auto_aim/CMakeLists.txt` | 每个文件一个 `INTERFACE` 占位库 + 其依赖；加入实现后就地改成 `STATIC` |
| `src/ui/`、`src/sim/CMakeLists.txt` | 目前只有注释形式的 `add_executable` 模板 |
| `src/thirdparty/CMakeLists.txt` | 只做接入：`find_package(Pangolin)` / `find_package(matplotlib_cpp)` + `livox_sdk2` imported target（预编译产物在 `build/thirdparty/install`） |
| `src/test/CMakeLists.txt` | `test_common` / `test_filters_data` / `test_filters_consumer`（可执行）+ `add_test` 注册，受 `PURE_NAV_BUILD_TESTS` 门控 |

**只有根文件用全局设置**（`CMAKE_RUNTIME_OUTPUT_DIRECTORY` 等）；模块文件不做重复设置，因此新增可执行目标会自动落到 `bin/`。

### 9.4 修改构建时的注意事项

- `enable_testing()` **必须留在顶层 `CMakeLists.txt`**：放在 `src/` 里会导致 `ctest` 在构建根目录看到 0 个测试（已实测踩过）。
- `Threads::Threads` 挂在 `pure_nav_common_libs` 上并 PUBLIC 传递，全项目都能拿到 `-pthread`；不要在各模块重复链接。
- `PURE_NAV_BIN_DIR` 由顶层定义，所有可执行目标都往这里输出；新增可执行目标不要再 `set_target_properties` 到别处。
- 不要把 `CMakeLists.txt` 拆到 `src` 下一级以下（例如不要建 `src/common/shm/CMakeLists.txt`）；第三方库由 `scripts/autoBuild.sh` 在 `build/thirdparty/` 独立编译，`src/thirdparty/CMakeLists.txt` 只负责接入，**不要**在那里 `add_subdirectory` 第三方源码。
