add_library(pure_nav_perception STATIC
    terrain/src/persistent_voxel_map.cpp terrain/src/per_frame_height_map.cpp)
target_include_directories(pure_nav_perception PUBLIC terrain/include ../navigation/include)
