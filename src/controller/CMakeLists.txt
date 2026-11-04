add_library(pure_nav_controller STATIC omni_pid/src/core/pid.cpp src/pursuit.cpp)
target_include_directories(pure_nav_controller PUBLIC omni_pid/include include ../navigation/include)
