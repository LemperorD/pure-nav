# 本次验收实测结果

- 独立构建：通过。
- CTest：10/10 组通过（154 项原算法测试 + 10 项新增合同测试 + 4 个仿真场景）。
- ASan / UBSan / LeakSanitizer：10/10 组通过。LeakSanitizer 在沙箱 ptrace 下不能运行，已获准在沙箱外重跑通过。
- 原工程完整构建：通过，使用 `build/thirdparty/install` 中的项目依赖。
- 原工程兼容 CTest：沙箱内 16/17 组通过，test_shm 被 socket 权限限制；获准在沙箱外重跑 test_shm 与 test_shm_cross_process，两项通过。
- 导航：282 帧到达（最后一帧 14.05s）；最大定位位置误差 0.02323m。
- 补给：142 帧到达（7.05s）；最大定位位置误差 0.02001m。
- 不可达地图：41 帧持续零速，符合场景期望。
- 点云丢失：第 20 帧起无点云，随后零速，符合场景期望。
- 原 rmul_2025 地图规划：成功，198 个输出路径点。
- 来源清单：129 个文件的原仓库哈希核对通过；未适配文件逐字节一致。

重新验收：`./scripts/acceptance.sh`。日志、CSV 与轨迹图位于 `build/standalone/acceptance/`。
完整范围见 [MIGRATION.md](MIGRATION.md)。这些结果不是 Point-LIO、Gazebo 或实车验收。
