add_library(pure_nav_planner STATIC
    jps/src/jps_algorithm.cpp jps/src/detail/jps_obstacle.cpp
    jps/src/detail/jps_neighbor.cpp jps/src/detail/jps_jump.cpp)
target_include_directories(pure_nav_planner PUBLIC jps/include)
find_package(Eigen3 REQUIRED)
add_library(pure_nav_bspline STATIC
    bspline/src/bspline_optimizer.cpp bspline/src/detail/basis.cpp
    bspline/src/detail/cost_cache.cpp bspline/src/detail/cost_function.cpp
    bspline/src/detail/esdf_utils.cpp bspline/src/detail/gradient_descent.cpp
    bspline/src/detail/grid_utils.cpp)
target_include_directories(pure_nav_bspline PUBLIC bspline/include)
target_link_libraries(pure_nav_bspline PUBLIC Eigen3::Eigen)
