# ============================================================================
# perception —— 感知层（装甲板检测、目标识别、点云/图像处理）
#
# 目前没有任何实现文件，先声明为 INTERFACE 占位库；加入实现后改为 STATIC：
#
#   add_library(pure_nav_perception STATIC
#       src/xxx.cpp)
#   target_include_directories(pure_nav_perception PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
#   target_link_libraries(pure_nav_perception PUBLIC pure_nav_common_libs pure_nav_driver)
# ============================================================================

add_library(pure_nav_perception INTERFACE)

target_include_directories(pure_nav_perception INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(pure_nav_perception INTERFACE
    pure_nav_common_libs
    pure_nav_driver)
