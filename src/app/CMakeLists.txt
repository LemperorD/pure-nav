# ============================================================================
# app —— 程序主入口 -> bin/pure_nav
#
# app 层只做"装配 + 调度"：读 config/ 参数、构造各模块对象、启动线程与主循环、
# 处理退出信号，不实现具体算法。
#
# 新增源文件时，把文件名加进下面的 add_executable 列表。
# 输出目录由顶层 CMAKE_RUNTIME_OUTPUT_DIRECTORY 统一指定为 bin/。
# ============================================================================

add_executable(pure_nav
    main.cpp)

target_link_libraries(pure_nav PRIVATE
    pure_nav_driver
    pure_nav_perception
    pure_nav_odometry
    pure_nav_planner
    pure_nav_controller
    pure_nav_auto_aim
    pure_nav_shm
    pure_nav_common_libs
    pure_nav_type_alias)
