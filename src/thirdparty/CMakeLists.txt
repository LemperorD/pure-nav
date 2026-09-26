# thirdparty —— 第三方库接入层
#
# 三个第三方库（Livox-SDK2 / Pangolin / matplotlib-cpp）**不在本项目构建树内编译**：
#   scripts/autoBuild.sh thirdparty
# 会把它们各自独立配置、编译、安装到 build/thirdparty/install/，与项目模块的
# build/lib/、build/src/ 产物完全分开（见脚本头部注释）。
#
# 本文件只负责从预编译前缀找到它们，供各业务模块 target_link_libraries 使用：
#   Pangolin        -> pango_core / pango_display / ...（PangolinConfig.cmake，无命名空间）
#   matplotlib-cpp  -> matplotlib_cpp::matplotlib_cpp（header-only）
#   Livox-SDK2      -> livox_sdk2（手工声明的 imported target）
#
# 相关变量：
#   PURE_NAV_THIRDPARTY_PREFIX    预编译第三方库安装前缀
#   PURE_NAV_THIRDPARTY_REQUIRED  缺失时是否直接报错（默认 OFF，骨架阶段还没有模块引用）

set(PURE_NAV_THIRDPARTY_PREFIX "${CMAKE_BINARY_DIR}/thirdparty/install"
    CACHE PATH "预编译第三方库安装前缀（由 scripts/autoBuild.sh thirdparty 生成）")

option(PURE_NAV_THIRDPARTY_REQUIRED "第三方库缺失时直接报错" OFF)

list(APPEND CMAKE_PREFIX_PATH "${PURE_NAV_THIRDPARTY_PREFIX}")

# --- Pangolin（可视化，导出 lib/cmake/Pangolin/PangolinConfig.cmake） --------
find_package(Pangolin QUIET CONFIG)

# --- matplotlib-cpp（header-only，导出 lib/matplotlib_cpp/cmake/...） --------
find_package(matplotlib_cpp QUIET CONFIG)

# --- Livox-SDK2（未导出 CMake 包，手工声明 imported target） ------------------
if(NOT TARGET livox_sdk2)
    set(_livox_include "${PURE_NAV_THIRDPARTY_PREFIX}/include")
    set(_livox_static "${PURE_NAV_THIRDPARTY_PREFIX}/lib/liblivox_lidar_sdk_static.a")
    if(EXISTS "${_livox_include}/livox_lidar_api.h" AND EXISTS "${_livox_static}")
        find_package(Threads REQUIRED)
        # GLOBAL 必须加：imported target 默认只在**声明它的目录及其子目录**可见，
        # 而使用方（src/driver 等）与 src/thirdparty 是兄弟目录，不加 GLOBAL
        # 会出现"这里提示已接入、那边 if(TARGET livox_sdk2) 却是假"的怪象。
        add_library(livox_sdk2 STATIC IMPORTED GLOBAL)
        set_target_properties(livox_sdk2 PROPERTIES
            IMPORTED_LOCATION "${_livox_static}"
            INTERFACE_INCLUDE_DIRECTORIES "${_livox_include}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads")
    endif()
endif()

# --- 配置摘要 ----------------------------------------------------------------
if(TARGET livox_sdk2)
    message(STATUS "  第三方库 Livox-SDK2     : 已接入 (${PURE_NAV_THIRDPARTY_PREFIX})")
else()
    message(STATUS "  第三方库 Livox-SDK2     : 未找到")
endif()

if(Pangolin_FOUND)
    message(STATUS "  第三方库 Pangolin       : 已接入 (${PURE_NAV_THIRDPARTY_PREFIX})")
else()
    message(STATUS "  第三方库 Pangolin       : 未找到")
endif()

if(matplotlib_cpp_FOUND)
    message(STATUS "  第三方库 matplotlib-cpp : 已接入 (${PURE_NAV_THIRDPARTY_PREFIX})")
else()
    message(STATUS "  第三方库 matplotlib-cpp : 未找到")
endif()

if(PURE_NAV_THIRDPARTY_REQUIRED)
    if(NOT TARGET livox_sdk2 OR NOT Pangolin_FOUND OR NOT matplotlib_cpp_FOUND)
        message(FATAL_ERROR
            "第三方库不完整。请先执行：scripts/autoBuild.sh thirdparty\n"
            "安装前缀：${PURE_NAV_THIRDPARTY_PREFIX}")
    endif()
endif()

unset(_livox_include)
unset(_livox_static)
