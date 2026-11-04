add_library(pure_nav_odometry STATIC src/scan_localizer.cpp)
target_include_directories(pure_nav_odometry PUBLIC include ../navigation/include)
