# test —— 单元测试（由顶层选项 PURE_NAV_BUILD_TESTS 控制）

# test_data 的绝对路径
set(PURE_NAV_TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test_data")

# 测试的指标计算用到 <cmath> 的 std::sqrt，Linux 下必须显式链接 libm
set(PURE_NAV_TEST_EXTRA_LIBS "")
if(UNIX)
    list(APPEND PURE_NAV_TEST_EXTRA_LIBS m)
endif()

# --- test_common：手工构造小数据的单元测试 ----------------------------------
add_executable(test_common
    test_common/test_filters.cpp)

target_link_libraries(test_common PRIVATE
    pure_nav_common_libs
    pure_nav_type_alias
    ${PURE_NAV_TEST_EXTRA_LIBS})

add_test(NAME test_common COMMAND test_common)

# --- test_filters_data：基于 test_data/ 中 CSV 的数据驱动测试 ----------------
add_executable(test_filters_data
    test_common/test_filters_data.cpp)

target_link_libraries(test_filters_data PRIVATE
    pure_nav_common_libs
    pure_nav_type_alias
    ${PURE_NAV_TEST_EXTRA_LIBS})

target_compile_definitions(test_filters_data PRIVATE
    PURE_NAV_TEST_DATA_DIR="${PURE_NAV_TEST_DATA_DIR}")

add_test(NAME test_filters_data COMMAND test_filters_data)

# --- test_filters_consumer：模拟下游模块引用 ---------------------------------
# 刻意**只**链接 pure_nav_common_libs：若 PUBLIC include 传递失效、或 filters.cpp
# 里显式实例化的模板符号缺失，这个目标就会编译/链接失败，从而把问题暴露出来。
add_executable(test_filters_consumer
    test_common/test_filters_consumer.cpp)

target_link_libraries(test_filters_consumer PRIVATE
    pure_nav_common_libs
    ${PURE_NAV_TEST_EXTRA_LIBS})

add_test(NAME test_filters_consumer COMMAND test_filters_consumer)

# --- test_livox_driver：雷达驱动的离线单元测试 --------------------------------
# 只链接 pure_nav_livox_driver：既验证驱动的公共头可用，也验证解码/组帧/队列
# 这些纯逻辑在没有硬件的环境下能独立跑通。SDK 缺失时该目标仍应能编译并通过
# （驱动退化为占位实现，但这些数据结构与算法不依赖 SDK）。
add_executable(test_livox_driver
    test_driver/test_livox_driver.cpp)

target_include_directories(test_livox_driver PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test_common")

target_link_libraries(test_livox_driver PRIVATE
    pure_nav_livox_driver
    ${PURE_NAV_TEST_EXTRA_LIBS})

add_test(NAME test_livox_driver COMMAND test_livox_driver)

# --- test_shm：共享内存发布/订阅单元测试 --------------------------------------
# 链接 pure_nav_shm_inproc：它在进程内启动一个 RouDi，因此测试不依赖外部
# iox-roudi 守护进程，ctest 可以直接运行。iceoryx 未接入时该目标不存在。
if(TARGET pure_nav_shm_inproc)
    add_executable(test_shm
        test_common/test_shm.cpp)

    target_include_directories(test_shm PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/test_common")

    target_link_libraries(test_shm PRIVATE
        pure_nav_shm_inproc
        ${PURE_NAV_TEST_EXTRA_LIBS})

    add_test(NAME test_shm COMMAND test_shm)

    # --- test_shm_cross_process：外部 iox-roudi + fork 子进程的真实跨进程测试 ---
    # 只链接 pure_nav_shm（核心库），RouDi 由测试自己 fork 执行；路径在配置期
    # 注入，运行期不存在时会明确 SKIP。
    set(PURE_NAV_IOX_ROUDI "${PURE_NAV_THIRDPARTY_PREFIX}/bin/iox-roudi")

    add_executable(test_shm_cross_process
        test_common/test_shm_cross_process.cpp)

    target_include_directories(test_shm_cross_process PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/test_common")

    target_link_libraries(test_shm_cross_process PRIVATE
        pure_nav_shm
        ${PURE_NAV_TEST_EXTRA_LIBS})

    if(EXISTS "${PURE_NAV_IOX_ROUDI}")
        target_compile_definitions(test_shm_cross_process PRIVATE
            PURE_NAV_IOX_ROUDI="${PURE_NAV_IOX_ROUDI}")
    endif()

    add_test(NAME test_shm_cross_process COMMAND test_shm_cross_process)
    set_tests_properties(test_shm_cross_process PROPERTIES TIMEOUT 60)
endif()
