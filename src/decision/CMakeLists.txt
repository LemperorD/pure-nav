add_library(pure_nav_decision STATIC src/core/decision.cpp src/core/context_environment_context.cpp)
target_include_directories(pure_nav_decision PUBLIC include)
target_link_libraries(pure_nav_decision PUBLIC Threads::Threads)
