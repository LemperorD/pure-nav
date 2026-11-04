add_library(pure_nav_simulation STATIC src/simulator.cpp src/simulation_cli.cpp)
target_include_directories(pure_nav_simulation PUBLIC include ../navigation/include)
add_executable(pure_nav_sim main.cpp)
target_link_libraries(pure_nav_sim PRIVATE pure_nav_navigation pure_nav_simulation)

target_link_libraries(pure_nav_simulation PUBLIC pure_nav_navigation)
