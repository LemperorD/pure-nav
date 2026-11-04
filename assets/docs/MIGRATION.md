# guganav → pure-nav 迁移与验收

本次交付是 **可运行的无 ROS 二维导航主链和原算法核心迁移**，不是 guganav 全功能等价版本。
原仓库保持不变；pure-nav 原有 ring_buffer、共享内存、驱动及用户未提交修改保留。

## 一键验收

在项目根目录执行：

```bash
./scripts/acceptance.sh
```

依赖：C++17 编译器、CMake ≥ 3.22、Eigen3、yaml-cpp；回归测试另需 GTest。
此入口不需要 source ROS 环境，不使用 colcon/ament、Nav2、Gazebo、PCL、OpenCV 或硬件 SDK。
项目中的 Python 仅生成验收图，不参与导航运行。

输出位于 `build/standalone/acceptance/`：

- `index.html` / `trajectories.svg`：实际运行得到的轨迹，真值与定位估计对照。
- `ctest.xml` / `tests.log`：测试结果。
- `configure.log` / `build.log`：构建日志。
- `navigation.csv`、`supply.csv`、`blocked.csv`、`sensor-loss.csv`：逐帧数据。
- `rmul_2025_route.csv`：原 rmul_2025 地图上的规划路径。

也可逐步执行：

```bash
cmake --preset standalone
cmake --build --preset standalone
ctest --preset standalone
./bin/pure_nav --scenario navigation --output build/navigation.csv
./bin/pure_nav_sim --scenario supply --output build/supply.csv
./bin/pure_nav --scenario blocked --output build/blocked.csv
./bin/pure_nav --scenario sensor-loss --output build/sensor-loss.csv
./bin/pure_nav --map assets/maps/rmul_2025.yaml \
  --start 0.775,-5.525,0 --goal 10.525,0.225,0 \
  --plan-only --output build/rmul_2025_route.csv
```

`pure_nav` 与 `pure_nav_sim` 目前共用仿真/离线规划 CLI。退出码 0 为场景验收通过，1 为未到达/不可达/指标未通过，2 为输入错误或碰撞。
`blocked` 的成功条件是保持零速，不是到达；`sensor-loss` 在第 20 帧后丢失点云，成功条件是随后持续零速。
导入地图目前只支持 `--plan-only`；不声称已在原比赛场景做物理仿真。

## 迁移范围

| 原 guganav 模块 | pure-nav 实现 | 状态与边界 |
|---|---|---|
| jps_planner | `src/planner/jps` | 原 JPS 核心及原 32 项测试迁入；去除 Nav2 插件壳。补空地图指针保护。导航适配负责跳点加密、贴角绕行和完整碰撞复核。 |
| bspline_optimizer | `src/planner/bspline` | 原核心、Eigen 依赖和原 13 项测试迁入，独立 `pure_nav_bspline` 库。主闭环当前使用 JPS 加密路径；尚未接入平滑/ESDF 优化策略。只修正一个遮蔽变量名。 |
| pb_omni_pid_pursuit_controller | `src/controller/omni_pid` | 原 PID、参数类型、原 9 项测试迁入。新增无 ROS 全向跟踪适配，含速度限制、目标航向和膨胀地图预测碰撞检查；尚非原 Nav2 控制器全部参数行为的等价实现。 |
| simple_decision | `src/decision` | 原 Decision / EnvironmentContext / 类型及原 57 项测试迁入。接入比赛门控、默认目标、低血量/缺弹补给、补给恢复、攻击目标与底盘模式输出。 |
| terrain_analysis | `src/perception/terrain` | 原持续体素地图和逐帧高度估计及原 43 项算法测试迁入。PCL 点容器替换为明确命名的 std::vector 容器，保留地面分位数、接收带、时间衰减与障碍高度语义。 |
| 地图 | `assets/maps/rmul_2025.*` | 原地图原样复制；无 ROS YAML/PGM 读取器支持原点、上下翻转、阈值、未知格与 negate。仅支持 trinary、8 位 P2/P5、零旋转原点。 |
| Gazebo/RMOSS 仿真 | `src/sim` | 新增确定性二维全向底盘、射线点云、带比例误差编码器、真值碰撞检查。**未迁移 Gazebo 物理插件和机器人模型。** |
| Point-LIO / small_gicp | `src/odometry` | 新增二维编码器预测 + SE(2) ICP 扫描匹配，输入已知地图与近似初始位姿，定位不读取仿真真值。**原 Point-LIO、small_gicp 三维重定位尚未迁移。** |
| MPC / MPPI / MINCO / ROG 地图层 | — | 尚未迁移、未在主链启用。 |
| guga_rmul_strategy / 行为树 | — | 尚未迁移；当前决策来自 simple_decision。 |
| 串口、Livox、相机、UI | 原 pure-nav 代码保留 | 本次不把仿真命令接入实车输出；原驱动与基础库另做兼容构建测试。 |

源文件及原始 SHA-256 见 `migration_sources.json`；`adapted` 表示有明确适配改动。
授权文件见 `assets/licenses/guganav-LICENSE` 及 `src/controller/omni_pid/LICENSE`，原作者版权注释保留。

## 数据流和接口约定

```text
仿真底盘 → 编码器 / 点云 → ScanLocalizer → 位姿
                                  ↓
                   点云变换 → PersistentVoxelMap → PerFrameHeightMap
                                  ↓
                       障碍地图 + 0.22m 膨胀
                                  ↓
裁判状态 / 比赛状态 / 带时间戳目标 → Decision → JPS → Pursuit → 底盘速度
```

`Navigation::tick(scan, measured_velocity, stamp, robot_status)` 是运行入口；`onGameStatus` 和 `onTarget` 注入外部事件。
所有导航实例 API 由同一调度线程调用；不承担跨进程 IPC，不持有 ROS 节点。

- 长度米、角度弧度、时间秒，时间戳必须有限且严格递增；相邻帧超过 0.2 秒停止本帧输出。
- 点云 XY 为底盘局部坐标，Z 为地图高度（仅平面仿真成立）；不可直接把有俯仰的真实雷达点云当作此输入。
- 速度为底盘系 vx/vy/wz；路径、目标、估计位姿为地图坐标。
- 目标输入是地图坐标，0.5 秒内且在 combat_max_distance 内有效；过期恢复默认/补给决策。
- `chassis_mode` 输出原决策模式；CLI 不驱动物理底盘小陀螺。
- 定位失败、空路径、未知格、越界、丢帧时不给非零平移命令；控制前复核预测路径。
- 机器人仿真半径 0.20m，规划膨胀半径 0.22m；这些是当前固定参数，尚未形成完整配置系统。
- 原点云数据精简为 XYZ+intensity，尚未接入真实每点时间、IMU 时间同步和去畸变。

## 验证与后续验收边界

当前回归覆盖原核心 154 个测试，以及新增导航合同测试、4 个闭环场景、原地图规划。
场景通过只能说明给定二维场景下闭环成立，不能证明实车性能、复杂动态环境鲁棒性或原系统等价。

完整迁移仍需依次完成：

1. 定义真实 LiDAR/IMU 帧、时间同步、外参、姿态和坐标变换，移除 Point-LIO 的 ROS 消息/TF/参数依赖；录包对照原算法轨迹与去畸变结果。
2. 移植 small_gicp 重定位与 INITIAL/FINE/SUSPECT/LOST 恢复流程；验证错误初始位姿、长时间失定位及重新捕获。
3. 接入 Gazebo 原模型、传感器和控制协议，验证坡面、碰撞、动态障碍；当前不包含这些资源的可运行移植。
4. 将 B-spline/ESDF、MPC/MPPI 作为可选择后端接入，迁移原控制器完整行为与配置。
5. 按真实串口/IPC 数据格式接入裁判、决策、底盘、传感器；硬件在环与实车验收。

检查器构建：

```bash
cmake --preset sanitized
cmake --build --preset sanitized
ctest --preset sanitized
```

原完整工程构建应使用项目已有依赖前缀，避免选到 `/opt/ros/humble` 的不同 iceoryx 版本：

```bash
cmake -S . -B build/compat -DPURE_NAV_STANDALONE=OFF \
  -DPURE_NAV_THIRDPARTY_PREFIX="$PWD/build/thirdparty/install"
cmake --build build/compat -j4
ctest --test-dir build/compat --output-on-failure
```

若该构建目录此前缓存过另一版本 iceoryx，配置时添加 `'-Uiceoryx_*_DIR'` 清除相关 CMake 缓存；不需要改 ROS 安装。
