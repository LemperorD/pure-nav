# ============================================================================
# src 聚合层
#
# 组织约定：src 的每个直接子目录各自持有一个 CMakeLists.txt，且**不再向下拆分**
# （例如 common 的 type_alias / common_libs / shm 三个目标全部写在
# src/common/CMakeLists.txt 里，没有 src/common/shm/CMakeLists.txt）。
# ============================================================================

add_subdirectory(thirdparty)     # 第三方库（当前为空）
add_subdirectory(common)         # 基础设施：type_alias / common_libs / shm
add_subdirectory(driver)         # 硬件与通信
add_subdirectory(perception)     # 感知
add_subdirectory(odometry)       # 状态估计
add_subdirectory(planner)        # 规划
add_subdirectory(controller)     # 控制
add_subdirectory(auto_aim)       # 自动瞄准
add_subdirectory(ui)             # 调试可视化
add_subdirectory(sim)            # 仿真
add_subdirectory(app)            # 主程序 -> bin/pure_nav

# 单元测试：默认关闭，用 -DPURE_NAV_BUILD_TESTS=ON 开启
if(PURE_NAV_BUILD_TESTS)
    add_subdirectory(test)
endif()
