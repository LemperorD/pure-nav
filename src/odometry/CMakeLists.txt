# ============================================================================
# odometry —— 里程计与状态估计（位姿解算、坐标变换、EKF）
#
# 目前没有任何实现文件，先声明为 INTERFACE 占位库；加入实现后改为 STATIC：
#
#   add_library(pure_nav_odometry STATIC
#       src/xxx.cpp)
#   target_include_directories(pure_nav_odometry PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
#   target_link_libraries(pure_nav_odometry PUBLIC pure_nav_common_libs pure_nav_driver)
# ============================================================================

add_library(pure_nav_odometry INTERFACE)

target_include_directories(pure_nav_odometry INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(pure_nav_odometry INTERFACE
    pure_nav_common_libs
    pure_nav_driver)
