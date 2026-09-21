# ============================================================================
# sim —— 仿真 / 离线回放程序 -> bin/pure_nav_sim
#
# 目前没有任何源文件，而 add_executable 至少需要一个源文件，因此暂不生成目标。
# 新建 src/sim/main.cpp 后，取消下面的注释即可：
#
# add_executable(pure_nav_sim
#     main.cpp)
# target_link_libraries(pure_nav_sim PRIVATE
#     pure_nav_common_libs
#     pure_nav_odometry
#     pure_nav_planner
#     pure_nav_controller)
# ============================================================================
