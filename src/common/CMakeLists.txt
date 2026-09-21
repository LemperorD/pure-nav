# ============================================================================
# common —— 通用基础设施（所有模块的公共依赖）
#
# 本文件在 src 下一级收口，三个子功能的目标全部写在这里：
#   type_alias/   纯头文件（INTERFACE 库）
#   common_libs/  通用算法库（STATIC 库）
#   shm/          共享内存 IPC（STATIC 库）
# ============================================================================

# --- type_alias：全局类型别名 / 单位约定 / 枚举，纯头文件，无 .cpp -------------
add_library(pure_nav_type_alias INTERFACE)

target_include_directories(pure_nav_type_alias INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/type_alias")

# --- common_libs：通用算法库（滤波器、数学工具等） ----------------------------
# 新增 .cpp 时，把文件名加进下面的源文件列表。
add_library(pure_nav_common_libs STATIC
    common_libs/src/filters.cpp)

target_include_directories(pure_nav_common_libs PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/common_libs/include")

# Threads::Threads 挂在 common_libs 上并对外传递，全项目都能拿到 -pthread
target_link_libraries(pure_nav_common_libs
    PUBLIC
        pure_nav_type_alias
        Threads::Threads)

# --- shm：共享内存 IPC --------------------------------------------------------
# 共享内存的布局一旦确定即为跨进程 ABI，修改需同步所有使用方。
add_library(pure_nav_shm STATIC
    shm/src/shm.cpp)

target_include_directories(pure_nav_shm PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/shm/include")

target_link_libraries(pure_nav_shm
    PUBLIC
        pure_nav_common_libs)
