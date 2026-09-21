# ============================================================================
# ui —— 调试可视化程序 -> bin/pure_nav_ui
#
# 目前没有任何源文件，而 add_executable 至少需要一个源文件，因此暂不生成目标。
# 新建 src/ui/main.cpp 后，取消下面的注释即可：
#
# add_executable(pure_nav_ui
#     main.cpp)
# target_link_libraries(pure_nav_ui PRIVATE
#     pure_nav_shm
#     pure_nav_odometry
#     pure_nav_perception
#     pure_nav_auto_aim
#     pure_nav_planner)
# ============================================================================
