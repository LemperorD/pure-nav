# ============================================================================
# driver —— 硬件与通信层（串口 / CAN / 网络 / 裁判系统 / 电机反馈）
#
# 目前没有任何实现文件，先声明为 INTERFACE 占位库，使上游可以正常 link。
# 加入实现文件后改成 STATIC，例如：
#
#   add_library(pure_nav_driver STATIC
#       src/serial.cpp
#       src/referee.cpp)
#   target_include_directories(pure_nav_driver PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
#   target_link_libraries(pure_nav_driver PUBLIC pure_nav_common_libs pure_nav_shm)
# ============================================================================

add_library(pure_nav_driver INTERFACE)

target_include_directories(pure_nav_driver INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(pure_nav_driver INTERFACE
    pure_nav_common_libs
    pure_nav_shm)
