# ============================================================================
# livox_driver —— 览沃(Livox)雷达驱动
#
# 结构：
#   include/livox_driver/livox_types.hpp      公共数据类型 + 解码/组帧（不依赖 SDK）
#   include/livox_driver/blocking_queue.hpp   预分配环形缓冲的阻塞队列
#   include/livox_driver/livox_driver.hpp     驱动类
#   src/livox_types.cpp                       线格式解码、组帧、配置校验
#   src/livox_driver.cpp                      SDK 生命周期、全局回调路由、工作线程
#
# 依赖 Livox-SDK2：src/thirdparty/CMakeLists.txt 在第三方预编译前缀可用时
# 会声明 imported target `livox_sdk2`。找不到时本库退化为占位实现
# （start() 直接失败并给出提示），这样其余模块仍可正常构建与跑测试。
# ============================================================================

add_library(pure_nav_livox_driver STATIC
    src/livox_types.cpp
    src/livox_driver.cpp)

target_include_directories(pure_nav_livox_driver PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(pure_nav_livox_driver
    PUBLIC
        pure_nav_common_libs
        pure_nav_shm)

if(TARGET livox_sdk2)
    # 第三方头文件不受本项目代码风格约束，标记为 SYSTEM 以免其告警污染编译输出
    get_target_property(_pure_nav_livox_include_dirs livox_sdk2 INTERFACE_INCLUDE_DIRECTORIES)
    if(_pure_nav_livox_include_dirs)
        set_target_properties(livox_sdk2 PROPERTIES
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_pure_nav_livox_include_dirs}")
    endif()
    unset(_pure_nav_livox_include_dirs)

    target_link_libraries(pure_nav_livox_driver PUBLIC livox_sdk2)
    # 定义了它，livox_types.cpp 才会做"复刻结构体 vs SDK 结构体"的 static_assert
    target_compile_definitions(pure_nav_livox_driver PUBLIC PURE_NAV_HAS_LIVOX_SDK2=1)
    message(STATUS "  livox_driver            : 已接入 Livox-SDK2")
else()
    message(WARNING
        "未找到 Livox-SDK2：livox_driver 将编译为占位实现，start() 会直接失败。\n"
        "  如需真实取流，请先执行 scripts/autoBuild.sh thirdparty 后重新 cmake。")
endif()
