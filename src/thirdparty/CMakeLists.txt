# thirdparty —— 第三方库接入层
#
# 第三方库（Livox-SDK2 / Pangolin / matplotlib-cpp / iceoryx）**不在本项目构建树内编译**：
#   scripts/autoBuild.sh thirdparty
# 会把它们各自独立配置、编译、安装到 build/thirdparty/install/，与项目模块的
# build/lib/、build/src/ 产物完全分开（见脚本头部注释）。
#
# 本文件只负责从预编译前缀找到它们，供各业务模块 target_link_libraries 使用：
#   Pangolin        -> pango_core / pango_display / ...（PangolinConfig.cmake，无命名空间）
#   matplotlib-cpp  -> matplotlib_cpp::matplotlib_cpp（header-only）
#   Livox-SDK2      -> livox_sdk2（手工声明的 imported target）
#   iceoryx         -> iceoryx_posh::iceoryx_posh / iceoryx_posh::iceoryx_posh_roudi
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

# --- iceoryx（共享内存零拷贝 IPC，导出 iceoryx_posh / iceoryx_hoofs / iceoryx_platform） ---
# 由 scripts/autoBuild.sh thirdparty 构建；源码在 src/thirdparty/iceoryx，
# 构建入口是 iceoryx_meta/（iceoryx 源码根目录没有顶层 CMakeLists.txt）。
# 头文件装在 ${PREFIX}/include/iceoryx/v<版本>/，CMake 包装在 ${PREFIX}/lib/cmake。
find_package(iceoryx_posh QUIET CONFIG)

if(iceoryx_posh_FOUND)
    # cpptoml 是 header-only 依赖，iceoryx_posh_config 的导入目标仍把 `cpptoml`
    # 作为链接项导出，但我们的安装前缀没有导出该 target。补一个空 INTERFACE
    # 目标，避免使用方链接 iceoryx_posh_config 时 CMake 把它当成 -lcpptoml。
    if(NOT TARGET cpptoml)
        add_library(cpptoml INTERFACE)
    endif()

    # IMPORTED target 默认只在声明它的目录及其子目录可见；iceoryx 的消费方
    # （src/common、src/driver 等）与本目录是兄弟目录，必须提升为 GLOBAL，
    # 否则那边 if(TARGET iceoryx_posh::iceoryx_posh) 会判断为假。
    foreach(_iox_target IN ITEMS
            iceoryx_posh::iceoryx_posh
            iceoryx_posh::iceoryx_posh_roudi
            iceoryx_posh::iceoryx_posh_config
            iceoryx_posh::iceoryx_posh_gateway
            iceoryx_hoofs::iceoryx_hoofs
            iceoryx_platform::iceoryx_platform)
        if(TARGET ${_iox_target})
            set_target_properties(${_iox_target} PROPERTIES IMPORTED_GLOBAL TRUE)
        endif()
    endforeach()
    unset(_iox_target)
endif()

# --- 配置摘要 ----------------------------------------------------------------
if(TARGET iceoryx_posh::iceoryx_posh)
    message(STATUS "  第三方库 iceoryx         : 已接入 (${iceoryx_posh_DIR})")
else()
    message(STATUS "  第三方库 iceoryx         : 未找到")
endif()
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
    if(NOT TARGET livox_sdk2 OR NOT Pangolin_FOUND OR NOT matplotlib_cpp_FOUND
       OR NOT TARGET iceoryx_posh::iceoryx_posh)
        message(FATAL_ERROR
            "第三方库不完整。请先执行：scripts/autoBuild.sh thirdparty\n"
            "安装前缀：${PURE_NAV_THIRDPARTY_PREFIX}")
    endif()
endif()

unset(_livox_include)
unset(_livox_static)
