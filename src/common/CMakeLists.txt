# ============================================================================
# common —— 通用基础设施（所有模块的公共依赖）
#
# 本文件在 src 下一级收口，三个子功能的目标全部写在这里：
#   type_alias/   纯头文件（INTERFACE 库）
#   common_libs/  通用算法库（STATIC 库）
#   shm/          基于 iceoryx 的共享内存 IPC（STATIC 库）
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

# --- shm：基于 iceoryx 的共享内存 IPC ----------------------------------------
# shm.hpp 封装了 iceoryx 的运行时 / 发布 / 订阅；主题与消息布局一旦确定即为
# 跨进程 ABI，修改需同步所有使用方。
#
# iceoryx 缺失时（未执行 scripts/autoBuild.sh thirdparty）退化为空的 INTERFACE
# 目标：已有的 target_link_libraries(pure_nav_shm) 不会报"找不到库"，真正
# #include "shm.hpp" 的代码才会在编译期明确报出 iceoryx 头文件缺失。
if(TARGET iceoryx_posh::iceoryx_posh)
    add_library(pure_nav_shm STATIC
        shm/src/shm.cpp)

    target_include_directories(pure_nav_shm PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/shm/include")

    target_link_libraries(pure_nav_shm
        PUBLIC
            Threads::Threads
            iceoryx_posh::iceoryx_posh)

    # 进程内 RouDi（单进程多线程 / 单元测试用）。单独成一个库，让生产模块
    # 只链接 pure_nav_shm 时不被迫拖进整个 RouDi 实现。
    add_library(pure_nav_shm_inproc STATIC
        shm/src/shm_inproc.cpp)

    target_include_directories(pure_nav_shm_inproc PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/shm/src")

    target_link_libraries(pure_nav_shm_inproc
        PUBLIC
            pure_nav_shm
            iceoryx_posh::iceoryx_posh_roudi)
else()
    add_library(pure_nav_shm INTERFACE)
    target_include_directories(pure_nav_shm INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/shm/include")
    message(WARNING
        "未找到 iceoryx，pure_nav_shm 退化为占位目标（无法真正使用共享内存）。\n"
        "  请先执行：scripts/autoBuild.sh thirdparty")
endif()
