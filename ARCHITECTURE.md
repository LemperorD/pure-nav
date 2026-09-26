# pure-nav 项目架构说明（Agent 阅读版）

> 面向对象：接手本仓库的人类开发者与 AI Agent。
> 编写依据：仓库当前工作区快照（`git log` 至 `ae4c207`）+ 目录骨架 + 现有配置文件。
> 阅读约定：**文中所有"已实现 / 占位 / 规划"标注均以实际文件内容为准**；凡属推断的内容都显式标注为「推断」，请勿当作既成事实。

---

## 1. 项目定位

- 目标：RoboMaster **哨兵（Sentry）机器人的导航系统**，纯 C++ 实现。
- 语言标准：**C++11**（`CMakeLists.txt` 中 `set(CMAKE_CXX_STANDARD 11)`，`.clang-format` 中 `Standard: Cpp11`）。新增代码不要使用 C++14/17/20 特性。
- 构建系统：**CMake**（`cmake_minimum_required(VERSION 3.22)`）。
- 依赖策略：**不依赖 ROS 框架**（当前代码中无 `rclcpp` / `ros/ros.h` 等引用）。环境里虽装有 `/opt/ros/lyrical`，但项目自身走独立进程 + 共享内存的路线，ROS 仅可能用于上位机调试或环境初始化（`README.md` 中的 `fishros` 一键安装脚本属于环境准备，非代码依赖）。
- 仓库：`https://github.com/LemperorD/pure-nav.git`，主分支 `main`。

> ⚠️ 当前仓库处于**骨架阶段**：除 `src/common/common_libs/filters.hpp` 的命名空间空壳外，其余业务模块均为占位或空目录，构建链路尚未打通。详见第 7 节。

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

下面是每个模块的职责定义。**加「推断」的条目是基于 RoboMaster 哨兵系统的常规分层给出的设计意图，尚未有代码支撑**；Agent 实现时应与用户确认接口，不要自行假定。

| 模块 | 职责 | 预期上游 | 预期下游 | 状态 |
|---|---|---|---|---|
| `app` | 进程入口、模块装配、主循环/线程编排、生命周期管理 | 命令行 / `config/` | 全部模块 | 占位（打印 Hello） |
| `common/shm` | 共享内存布局、跨进程消息读写、无锁/加锁同步 | driver、app | planner、ui | 空壳 |
| `common/common_libs` | 通用算法：滤波器（滑动窗口均值/中值/一阶低通/一维 KF）等 | — | 所有模块 | filters 已实现并测试通过 |
| `common/type_alias` | 全局类型别名、单位约定、枚举 | — | 所有模块 | 空 |
| `driver` | 串口/CAN/网络通信、裁判系统、电机与传感器数据采集 | 硬件 | perception、odometry | 空 |
| `perception` | 装甲板/目标检测、点云或图像处理 | driver | auto_aim、odometry | 空 |
| `odometry` | 里程计解算、状态估计、坐标系变换 | driver、perception | planner、controller | 空 |
| `planner` | 全局/局部路径规划、代价地图、行为决策 | odometry、perception | controller | 空 |
| `controller` | 轨迹跟踪、底盘速度/舵轮控制、PID 等 | planner | driver | 空 |
| `auto_aim` | 弹道解算、云台控制、目标预测 | perception、odometry | driver | 空 |
| `ui` | 可视化、参数在线调节、日志呈现 | 全部模块 | 人 | 空 |
| `sim` | 仿真环境、离线回放、算法验证 | 测试数据 | planner、controller | 空 |
| `thirdparty` | 第三方库接入层（源码为 submodule，编译产物在 `build/thirdparty/`） | — | 全部模块 | 3 个库已独立编译安装，项目侧只 `find_package` |
| `test` | 单元测试与测试数据 | 被测模块 | CI/开发者 | 3 个测试目标（单元 / 数据驱动 / 下游引用）全部通过 |

### 3.1 推断的数据流（待确认）

```
                 ┌──────────────── 硬件 / 仿真 ────────────────┐
                 ▼                                             │
   driver ──► perception ──► auto_aim ──────────────────────► driver
      │            │                                          │
      │            └──► odometry ──► planner ──► controller ──►┘
      │                    ▲
      └────────────────────┘
                 common/shm  ◄── 跨进程数据总线（推断）
                 common_libs ◄── 被所有模块复用（滤波器/工具）
```

哨兵的关键特点是**全自动**：`auto_aim` 与 `planner/controller` 需在同一决策链上协同（打与走的权衡），因此 `app` 层的调度策略是整个系统的核心难点。

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

推荐使用一键脚本 `scripts/autoBuild.sh`（会自动处理三个第三方库，以及本机缺失 C++ 前端的免 root 兜底）：

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
- `src/test/test_common/test_filters.cpp`（单元）、`test_filters_data.cpp`（CSV 数据驱动）、`test_filters_consumer.cpp`（模拟下游模块引用）三个目标共用 `test_check.hpp` 里的迷你框架（断言 + 统计 + `main` 收尾，不依赖 GoogleTest/doctest/Catch2），失败时返回非 0，`ctest` 可直接判定。`PURE_NAV_BUILD_TESTS` 仍默认 `OFF`（按需开启）。
- `src/test/test_data/filters/*.csv` 的 `expected` 列由 `src/test/test_data/generate_filters_data.py` 用**独立**参考实现生成（golden），C++ 侧实测最大误差 ≤ 9e-16。数据文件已提交，**构建不需要 Python**；改数据时重跑脚本即可（见 `src/test/test_data/README.md`）。
- 数据驱动测试用**粗糙度**（一阶差分标准差）而非"与真值的 RMSE"评价降噪效果：平滑滤波器都存在相位滞后，阶跃/正弦上 filtered 的 RMSE 可能反而大于 raw。
- `.clang-format` 的键值写成 `Key:Value`（冒号后缺空格），**当前不是合法 YAML**，`clang-format -i` 会直接报 `not a mapping`。修复前请手工遵守第 4 节格式约定。
- C++11 下**不能**写 `namespace pure::common {`（那是 C++17 语法，`-Wpedantic`/`-Werror` 会报 `c++17-extensions`），必须写成嵌套的 `namespace pure { namespace common { ... } }`。
- `src/{driver,perception,odometry,planner,controller,auto_aim}` 目前是 **INTERFACE 占位库**（无实现文件）。它们声明的 `include/` 目录尚不存在，因此没有任何 `-I` 生效；一旦放入源文件并按 6.1 改成 STATIC 即可。
- `src/ui`、`src/sim` 尚无源文件，`add_executable` 需要至少一个源文件，因此这两个目标在各自的 `CMakeLists.txt` 中**以注释形式给出模板**，取消注释即可产出 `bin/pure_nav_ui` / `bin/pure_nav_sim`。
- `src/thirdparty/CMakeLists.txt` **只做接入**：从 `build/thirdparty/install` 找预编译的 Pangolin / matplotlib-cpp，并为 Livox-SDK2 声明 `livox_sdk2` imported target；它不编译任何第三方源码（见 5.2）。三个库的源码是 git submodule，需 `git submodule update --init --recursive`。

---

## 8. 给 Agent 的工作约定

**应当：**
- 动手前先读本文件第 7 节：本机**系统级**缺 C++ 前端（见 7.2，可免 root 绕过），业务代码本身没有硬阻塞。
- 保持 C++11、`namespace pure { namespace <模块> {`（C++11 不支持 `namespace a::b` 写法，见 7.4）、中文注释、`#pragma once` 的一致性。
- 新增源码文件后**必须**把它加进所属模块 `CMakeLists.txt` 的源文件列表（本项目不用 GLOB 自动收集）；新增**模块**或新增**跨模块依赖**时同样要改 CMakeLists（见第 6、9 节）。
- 修改后运行 `clang-format -i`（注意 7.4：`.clang-format` 当前是非法 YAML，修好前请手工对齐格式），并按 Conventional Commits 提交。
- 新增/变更模块时同步更新本文件的第 2、3、9 节。
- 涉及模块间接口（尤其 `common/shm` 的共享内存布局）时，**先与用户确认**，不要自创约定。

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

> 说明：依赖这些库只传递 include 目录与链接关系，**不代表数据流方向**。数据流见第 3.1 节（`driver → odometry/perception → planner → controller`），二者应当一致；若发现不一致，以数据流为准调整 DEPS。

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
