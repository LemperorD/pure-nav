# ============================================================================
# auto_aim —— 自动瞄准（目标预测、弹道解算、云台控制）
#
# 目前没有任何实现文件，先声明为 INTERFACE 占位库；加入实现后改为 STATIC：
#
#   add_library(pure_nav_auto_aim STATIC
#       src/xxx.cpp)
#   target_include_directories(pure_nav_auto_aim PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
#   target_link_libraries(pure_nav_auto_aim PUBLIC
#       pure_nav_common_libs pure_nav_perception pure_nav_odometry)
# ============================================================================

add_library(pure_nav_auto_aim INTERFACE)

target_include_directories(pure_nav_auto_aim INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(pure_nav_auto_aim INTERFACE
    pure_nav_common_libs
    pure_nav_perception
    pure_nav_odometry)
