# ============================================================================
# driver —— 硬件与通信层（串口 / CAN / 网络 / 雷达 / 裁判系统 / 电机反馈）
#
# 每个具体驱动在各自的子目录里声明一个 STATIC 库，再统一挂到聚合目标
# `pure_nav_driver` 上，上游只需要 link pure_nav_driver 即可。
#
# 当前子驱动：
#   livox_driver/  览沃雷达（Livox-SDK2）
#
# 后续新增驱动时按同样方式 add_subdirectory + 追加到下面的链接列表。
# ============================================================================

add_subdirectory(livox_driver)

add_library(pure_nav_driver INTERFACE)

target_link_libraries(pure_nav_driver INTERFACE
    pure_nav_common_libs
    pure_nav_shm
    pure_nav_livox_driver)
