# ============================================================================
# pure-nav —— RoboMaster 哨兵导航系统
#
# 纯 C++11 + 原生 CMake：不使用 ROS / colcon / ament，也不使用自定义宏。
# 所有目标都用标准命令（add_library / add_executable / target_*）声明。
#
# 构建：
#   cmake -S . -B build
#   cmake --build build -j$(nproc)
#
# 产物：
#   bin/        所有可执行文件
#   build/lib/  中间静态库
# ============================================================================

cmake_minimum_required(VERSION 3.22)

project(pure_nav
    VERSION     0.1.0
    DESCRIPTION "RoboMaster sentry navigation system (pure C++11)"
    LANGUAGES   CXX)

# ----------------------------------------------------------------------------
# 语言标准
# ----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD          11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

# Ubuntu 默认生成 PIE 可执行文件，静态库必须带 -fPIC 才能链接进去
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 生成 build/compile_commands.json，供 clangd / IDE 使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ----------------------------------------------------------------------------
# 输出目录：可执行文件统一放 bin/，静态库放 build/lib/
# ----------------------------------------------------------------------------
set(PURE_NAV_BIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/bin")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PURE_NAV_BIN_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# 多配置生成器（Ninja Multi-Config / Xcode / MSVC）需要逐配置设置
foreach(_cfg IN ITEMS DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${_cfg} "${PURE_NAV_BIN_DIR}")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${_cfg} "${CMAKE_BINARY_DIR}/lib")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${_cfg} "${CMAKE_BINARY_DIR}/lib")
endforeach()
unset(_cfg)

# 单配置生成器默认 Release；调试请 -DCMAKE_BUILD_TYPE=Debug
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "构建类型" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        Debug Release RelWithDebInfo MinSizeRel)
endif()

# ----------------------------------------------------------------------------
# 选项
# ----------------------------------------------------------------------------
option(PURE_NAV_BUILD_TESTS        "构建 src/test 下的单元测试"     OFF)
option(PURE_NAV_WARNINGS_AS_ERRORS "把编译警告当作错误 (-Werror)"   OFF)

# enable_testing() 必须出现在顶层目录，否则 ctest 在构建根目录看不到任何测试
if(PURE_NAV_BUILD_TESTS)
    enable_testing()
endif()

# ----------------------------------------------------------------------------
# 编译警告
# ----------------------------------------------------------------------------
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
elseif(MSVC)
    add_compile_options(/W4)
endif()

if(PURE_NAV_WARNINGS_AS_ERRORS)
    if(MSVC)
        add_compile_options(/WX)
    else()
        add_compile_options(-Werror)
    endif()
endif()

# ----------------------------------------------------------------------------
# 外部依赖（仅标准工具链，无 ROS）
# ----------------------------------------------------------------------------
find_package(Threads REQUIRED)

# ----------------------------------------------------------------------------
# 源码
# ----------------------------------------------------------------------------
add_subdirectory(src)

# ----------------------------------------------------------------------------
# 配置摘要
# ----------------------------------------------------------------------------
message(STATUS "")
message(STATUS "=================== pure-nav 配置摘要 ===================")
message(STATUS "  版本           : ${PROJECT_VERSION}")
message(STATUS "  C++ 标准       : C++${CMAKE_CXX_STANDARD}")
message(STATUS "  构建类型       : ${CMAKE_BUILD_TYPE}${CMAKE_CONFIGURATION_TYPES}")
message(STATUS "  编译器         : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  可执行输出     : ${PURE_NAV_BIN_DIR}")
message(STATUS "  静态库输出     : ${CMAKE_BINARY_DIR}/lib")
message(STATUS "  构建测试       : ${PURE_NAV_BUILD_TESTS}")
message(STATUS "=========================================================")
message(STATUS "")
