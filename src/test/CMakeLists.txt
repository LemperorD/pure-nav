# test —— 单元测试（由顶层选项 PURE_NAV_BUILD_TESTS 控制）

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
