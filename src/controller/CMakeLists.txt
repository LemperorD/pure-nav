# ============================================================================
# controller —— 控制层（轨迹跟踪、底盘/舵轮控制、PID）
#
# 目前没有任何实现文件，先声明为 INTERFACE 占位库；加入实现后改为 STATIC：
#
#   add_library(pure_nav_controller STATIC
#       src/xxx.cpp)
#   target_include_directories(pure_nav_controller PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
#   target_link_libraries(pure_nav_controller PUBLIC pure_nav_common_libs)
# ============================================================================

add_library(pure_nav_controller INTERFACE)

target_include_directories(pure_nav_controller INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(pure_nav_controller INTERFACE
    pure_nav_common_libs)
