# src 聚合层

add_subdirectory(thirdparty)     # 第三方库
add_subdirectory(common)         # 基础设施：type_alias / common_libs / shm
add_subdirectory(driver)         # 硬件与通信
add_subdirectory(perception)     # 感知
add_subdirectory(odometry)       # 状态估计（定位）
add_subdirectory(planner)        # 规划
add_subdirectory(controller)     # 控制
add_subdirectory(auto_aim)       # 自瞄
add_subdirectory(ui)             # 调试可视化
add_subdirectory(sim)            # 仿真
add_subdirectory(app)            # 主程序

# 单元测试：默认关闭，用 -DPURE_NAV_BUILD_TESTS=ON 开启
if(PURE_NAV_BUILD_TESTS)
    add_subdirectory(test)
endif()
