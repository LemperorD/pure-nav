# ============================================================================
# test —— 单元测试（由顶层选项 PURE_NAV_BUILD_TESTS 控制）
#
# src/CMakeLists.txt 已做门控：只有 -DPURE_NAV_BUILD_TESTS=ON 时才会进入本目录。
#
# 运行：
#   cmake -S . -B build -DPURE_NAV_BUILD_TESTS=ON
#   cmake --build build -j
#   ctest --test-dir build --output-on-failure
#
# 三个测试目标（都自带 main 与断言，失败返回非 0，不依赖第三方测试框架）：
#   test_common            手工构造小数据的单元测试（test_filters.cpp）
#   test_filters_data      读取 test_data/filters/*.csv 的数据驱动测试
#   test_filters_consumer  模拟下游模块引用 common_libs（刻意只链接库、不加 -I）
#
# test_common/test_check.hpp 是三个目标共用的迷你测试框架。
# test_data/ 只存放测试数据，不是测试目标，不在此注册。
# ============================================================================

# 数据驱动测试需要知道 test_data 的绝对路径（编译期注入，因此 ctest 从任意目录
# 运行都能找到数据文件）
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
