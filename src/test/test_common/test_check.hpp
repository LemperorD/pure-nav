// test_check.hpp —— src/test 内部共用的迷你测试框架
//
// 为什么不用 GoogleTest / doctest / Catch2：本项目坚持"纯 C++11 + 原生 CMake、
// 不引入额外依赖"，而测试需求只是"断言 + 统计 + 失败返回非 0"，几十行就够了。
// 任何一项断言失败都会让进程返回非 0，因此 add_test 可直接交给 ctest 判定。
//
// 用法（每个测试可执行文件包含一次）：
//     CHECK(condition);
//     CHECK_NEAR(lhs, rhs, tolerance);
//     CHECK_SERIES(actual_vector, expected_vector, tolerance);
//     int main() { pure_nav_test::run("用例名", 函数指针); ... return pure_nav_test::summary(); }

#pragma once

#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace pure_nav_test {

// ----------------------------------------------------------------------------
// 统计：函数内静态变量，保证同一可执行文件内（单/多翻译单元）只有一份计数
// ----------------------------------------------------------------------------
inline int& checks() {
    static int value = 0;
    return value;
}

inline int& failures() {
    static int value = 0;
    return value;
}

inline void report(bool passed, const std::string& expression, const char* file, int line) {
    ++checks();
    if (!passed) {
        ++failures();
        std::cerr << "[FAIL] " << file << ":" << line << ": " << expression << std::endl;
    }
}

template <typename T>
bool nearly_equal(T lhs, T rhs, double tolerance) {
    return std::fabs(static_cast<double>(lhs) - static_cast<double>(rhs)) <= tolerance;
}

template <typename T>
void check_series(const std::vector<T>& actual, const std::vector<T>& expected, double tolerance, const char* file, int line) {
    bool passed = actual.size() == expected.size();
    if (passed) {
        for (std::size_t i = 0; i < actual.size(); ++i) {
            if (!nearly_equal(actual[i], expected[i], tolerance)) {
                passed = false;
                break;
            }
        }
    }

    ++checks();
    if (!passed) {
        ++failures();
        std::cerr << "[FAIL] " << file << ":" << line << ": 序列不一致\n        实际值 = [";
        for (std::size_t i = 0; i < actual.size(); ++i) {
            std::cerr << (i == 0 ? "" : ", ") << actual[i];
        }
        std::cerr << "]\n        期望值 = [";
        for (std::size_t i = 0; i < expected.size(); ++i) {
            std::cerr << (i == 0 ? "" : ", ") << expected[i];
        }
        std::cerr << "]" << std::endl;
    }
}

typedef void (*TestFunction)();

inline void run(const char* name, TestFunction function) {
    const int failures_before = failures();
    std::cout << "[ RUN  ] " << name << std::endl;
    try {
        function();
    } catch (const std::exception& error) {
        ++failures();
        std::cerr << "[FAIL] " << name << ": 抛出未预期的异常: " << error.what() << std::endl;
    }
    std::cout << (failures() == failures_before ? "[  OK  ] " : "[ FAIL ] ") << name << std::endl;
}

inline int summary() {
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "断言总数: " << checks() << "，失败: " << failures() << std::endl;
    if (failures() == 0) {
        std::cout << "结果: 全部通过 ✅" << std::endl;
        return 0;
    }
    std::cout << "结果: 存在失败 ❌" << std::endl;
    return 1;
}

// ----------------------------------------------------------------------------
// 序列辅助函数（多种测试共用）
// ----------------------------------------------------------------------------

// 取序列最后 n 个元素（用于跳过启动阶段的暂态）
inline std::vector<double> last_n(const std::vector<double>& values, std::size_t n) {
    if (values.size() <= n) {
        return values;
    }
    return std::vector<double>(values.end() - static_cast<std::ptrdiff_t>(n), values.end());
}

inline double variance(const std::vector<double>& values) {
    if (values.size() < 2) {
        return 0.0;
    }
    double mean = 0.0;
    for (std::size_t i = 0; i < values.size(); ++i) {
        mean += values[i];
    }
    mean /= static_cast<double>(values.size());

    double accumulator = 0.0;
    for (std::size_t i = 0; i < values.size(); ++i) {
        const double deviation = values[i] - mean;
        accumulator += deviation * deviation;
    }
    return accumulator / static_cast<double>(values.size());
}

// 平均绝对误差
inline double mean_absolute_error(const std::vector<double>& actual, const std::vector<double>& expected) {
    if (actual.empty() || actual.size() != expected.size()) {
        return 0.0;
    }
    double accumulator = 0.0;
    for (std::size_t i = 0; i < actual.size(); ++i) {
        accumulator += std::fabs(actual[i] - expected[i]);
    }
    return accumulator / static_cast<double>(actual.size());
}

// 均方根误差
inline double root_mean_square_error(const std::vector<double>& actual, const std::vector<double>& expected) {
    if (actual.empty() || actual.size() != expected.size()) {
        return 0.0;
    }
    double accumulator = 0.0;
    for (std::size_t i = 0; i < actual.size(); ++i) {
        const double difference = actual[i] - expected[i];
        accumulator += difference * difference;
    }
    return std::sqrt(accumulator / static_cast<double>(actual.size()));
}

// 粗糙度：一阶差分的标准差，用来衡量信号的高频抖动（噪声）大小。
// 平滑滤波器（无论是否有相位滞后）都应该让粗糙度下降，因此比"与真值的 RMSE"
// 更适合评估滤波器对噪声的抑制能力。
inline double roughness(const std::vector<double>& values) {
    if (values.size() < 2) {
        return 0.0;
    }
    std::vector<double> differences;
    differences.reserve(values.size() - 1);
    for (std::size_t i = 1; i < values.size(); ++i) {
        differences.push_back(values[i] - values[i - 1]);
    }
    return std::sqrt(variance(differences));
}

} // namespace pure_nav_test

#define CHECK(condition) ::pure_nav_test::report((condition), #condition, __FILE__, __LINE__)

#define CHECK_NEAR(lhs, rhs, tolerance) ::pure_nav_test::report(::pure_nav_test::nearly_equal((lhs), (rhs), (tolerance)), std::string(#lhs) + " ~= " + #rhs, __FILE__, __LINE__)

#define CHECK_SERIES(actual, expected, tolerance) ::pure_nav_test::check_series((actual), (expected), (tolerance), __FILE__, __LINE__)
