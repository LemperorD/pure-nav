# pure-nav

C++17 导航项目。已从 guganav 迁入 JPS、B-spline、PID、simple_decision 和地形分析核心，提供不依赖 ROS 的二维导航闭环。

```bash
./scripts/acceptance.sh
```

验收报告：`build/standalone/acceptance/index.html`。手动运行：

```bash
cmake --preset standalone
cmake --build --preset standalone
ctest --preset standalone
./bin/pure_nav --scenario navigation
./bin/pure_nav --help
```

依赖 CMake ≥ 3.22、C++17、Eigen3、yaml-cpp；测试需 GTest。

**目前不是完整的 guganav 等价迁移**：Point-LIO/small_gicp、Gazebo 原物理仿真、MPC/MPPI、实车接线仍未完成。定位目前使用新建二维 ICP，仿真使用新建二维底盘。详见 [迁移范围、接口、验收及剩余工作](assets/docs/MIGRATION.md)。

原硬件/共享内存工程和第三方子模块仍保留：

```bash
git submodule update --init --recursive
./scripts/autoBuild.sh
```
