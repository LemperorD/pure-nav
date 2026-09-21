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
# ⚠️ test_common/test_filters.cpp 目前既没有 main() 也没有断言，且项目尚未引入
#    测试框架（GoogleTest / doctest / Catch2）。开启测试后会因缺少 main 而在链接
#    阶段失败，需先补上测试框架或自带 main。
#
# test_data/ 只存放测试数据，不是测试目标，不在此注册。
# ============================================================================

add_executable(test_common
    test_common/test_filters.cpp)

target_link_libraries(test_common PRIVATE
    pure_nav_common_libs
    pure_nav_type_alias)

add_test(NAME test_common COMMAND test_common)
