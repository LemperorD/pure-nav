# ============================================================================
# planner —— 规划层（全局/局部路径规划、代价地图、行为决策）
#
# 目前没有任何实现文件，先声明为 INTERFACE 占位库；加入实现后改为 STATIC：
#
#   add_library(pure_nav_planner STATIC
#       src/xxx.cpp)
#   target_include_directories(pure_nav_planner PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
#   target_link_libraries(pure_nav_planner PUBLIC
#       pure_nav_common_libs pure_nav_odometry pure_nav_perception)
# ============================================================================

add_library(pure_nav_planner INTERFACE)

target_include_directories(pure_nav_planner INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(pure_nav_planner INTERFACE
    pure_nav_common_libs
    pure_nav_odometry
    pure_nav_perception)
