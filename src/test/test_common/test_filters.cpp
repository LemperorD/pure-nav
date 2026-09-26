// test_filters.cpp —— common_libs/filters 单元测试（手工构造的小数据）
//
// 断言宏、失败统计与 main() 的收尾统一由 test_check.hpp 提供；本文件只写用例。
// 任何一项断言失败都会让进程返回非 0，因此可直接被 ctest 判定为通过 / 失败。
//
// 基于 test_data/ 中 CSV 的数据驱动测试见 test_filters_data.cpp。
//
// 运行：
//   cmake -S . -B build -DPURE_NAV_BUILD_TESTS=ON
//   cmake --build build -j
//   ctest --test-dir build --output-on-failure

#include "filters.hpp"
#include "test_check.hpp"

#include <stdexcept>
#include <vector>

namespace {

// 复用测试框架里的序列辅助函数
using pure_nav_test::last_n;
using pure_nav_test::variance;


// ----------------------------------------------------------------------------
// SlidingWindowFilter / MovingAverageFilter
// ----------------------------------------------------------------------------
void testSlidingWindowBasic() {
    pure::common::SlidingWindowFilter<double> filter(3);
    CHECK(filter.window_size() == 3);
    CHECK(filter.size() == 0);

    const std::vector<double> input    = {1.0, 2.0, 3.0, 4.0, 5.0};
    const std::vector<double> expected = {1.0, 1.5, 2.0, 3.0, 4.0}; // 启动阶段按已有样本求平均

    CHECK_SERIES(filter.apply(input), expected, 1e-12);
    CHECK(filter.size() == 3);
    CHECK_NEAR(filter.sum(), 12.0, 1e-12); // 3 + 4 + 5
}

void testSlidingWindowWindowSizeOne() {
    pure::common::SlidingWindowFilter<double> filter(1);

    const std::vector<double> input    = {1.0, -2.0, 3.5};
    const std::vector<double> expected = {1.0, -2.0, 3.5}; // 窗口为 1 时直通

    CHECK_SERIES(filter.apply(input), expected, 1e-12);
    CHECK(filter.size() == 1);
}

void testSlidingWindowOnlineAndReset() {
    pure::common::MovingAverageFilter<double> filter(2); // 使用语义别名

    CHECK_NEAR(filter.update(2.0), 2.0, 1e-12);
    CHECK_NEAR(filter.update(4.0), 3.0, 1e-12);
    CHECK_NEAR(filter.update(6.0), 5.0, 1e-12);
    CHECK(filter.size() == 2);

    filter.reset();
    CHECK(filter.size() == 0);
    CHECK_NEAR(filter.sum(), 0.0, 1e-12);
    CHECK_NEAR(filter.update(10.0), 10.0, 1e-12);
}

void testSlidingWindowApplyResetsState() {
    pure::common::SlidingWindowFilter<double> filter(2);
    filter.update(100.0); // 人为制造历史状态

    const std::vector<double> input    = {1.0, 3.0};
    const std::vector<double> expected = {1.0, 2.0}; // apply 会先 reset，历史不应参与

    CHECK_SERIES(filter.apply(input), expected, 1e-12);
}

void testSlidingWindowEmptyInput() {
    pure::common::SlidingWindowFilter<double> filter(4);
    const std::vector<double> empty;

    CHECK(filter.apply(empty).empty());
    CHECK(filter.size() == 0);
}

void testSlidingWindowInvalidWindow() {
    bool threw = false;
    try {
        pure::common::SlidingWindowFilter<double> filter(0);
        (void)filter;
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
}

// ----------------------------------------------------------------------------
// MedianFilter
// ----------------------------------------------------------------------------
void testMedianRejectsSpike() {
    pure::common::MedianFilter<double> filter(3);

    // 中间的 100 是野值，中值滤波应把它完全滤掉
    const std::vector<double> input    = {1.0, 1.0, 100.0, 1.0, 1.0};
    const std::vector<double> expected = {1.0, 1.0, 1.0, 1.0, 1.0};

    CHECK_SERIES(filter.apply(input), expected, 1e-12);
}

void testMedianEvenWindow() {
    pure::common::MedianFilter<double> filter(4);

    // 偶数窗口：中位数取中间两个值的平均
    const std::vector<double> input    = {1.0, 2.0, 3.0, 4.0};
    const std::vector<double> expected = {1.0, 1.5, 2.0, 2.5};

    CHECK_SERIES(filter.apply(input), expected, 1e-12);
}

void testMedianWindowSizeTwo() {
    pure::common::MedianFilter<double> filter(2);

    const std::vector<double> input    = {5.0, 1.0, 3.0};
    const std::vector<double> expected = {5.0, 3.0, 2.0};

    CHECK_SERIES(filter.apply(input), expected, 1e-12);
}

void testMedianResetAndEmptyInput() {
    pure::common::MedianFilter<double> filter(3);
    filter.update(9.0);
    filter.update(9.0);
    CHECK(filter.size() == 2);

    filter.reset();
    CHECK(filter.size() == 0);

    const std::vector<double> empty;
    CHECK(filter.apply(empty).empty());
}

void testMedianInvalidWindow() {
    bool threw = false;
    try {
        pure::common::MedianFilter<double> filter(0);
        (void)filter;
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
}

// ----------------------------------------------------------------------------
// FirstOrderLowPassFilter
// ----------------------------------------------------------------------------
void testLowPassAlphaOneIsPassthrough() {
    pure::common::FirstOrderLowPassFilter<double> filter(1.0);
    CHECK(!filter.initialized());

    const std::vector<double> input    = {1.0, 2.0, 3.0};
    CHECK_SERIES(filter.apply(input), input, 1e-12);
    CHECK(filter.initialized());
}

void testLowPassStepResponse() {
    pure::common::FirstOrderLowPassFilter<double> filter(0.5);

    CHECK_NEAR(filter.update(0.0), 0.0, 1e-12); // 首样本直接作为初值
    CHECK_NEAR(filter.update(1.0), 0.5, 1e-12);
    CHECK_NEAR(filter.update(1.0), 0.75, 1e-12);
    CHECK_NEAR(filter.update(1.0), 0.875, 1e-12);

    for (int i = 0; i < 20; ++i) {
        filter.update(1.0);
    }
    CHECK_NEAR(filter.value(), 1.0, 1e-5); // 最终收敛到输入常值

    filter.reset();
    CHECK(!filter.initialized());
    CHECK_NEAR(filter.update(7.0), 7.0, 1e-12);
}

void testLowPassSmoothsAlternatingNoise() {
    pure::common::FirstOrderLowPassFilter<double> filter(0.2);

    std::vector<double> noisy;
    for (int i = 0; i < 200; ++i) {
        noisy.push_back((i % 2 == 0) ? 9.0 : 11.0); // 均值 10、方差 1
    }
    const std::vector<double> filtered = filter.apply(noisy);

    const std::vector<double> tail_noisy    = last_n(noisy, 100);
    const std::vector<double> tail_filtered = last_n(filtered, 100);

    CHECK(variance(tail_filtered) < 0.5 * variance(tail_noisy)); // 明显更平滑
    CHECK(filtered.size() == noisy.size());
}

void testLowPassInvalidAlpha() {
    const double invalid_alphas[] = {0.0, -0.5, 1.5};

    for (std::size_t i = 0; i < 3; ++i) {
        bool threw = false;
        try {
            pure::common::FirstOrderLowPassFilter<double> filter(invalid_alphas[i]);
            (void)filter;
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);
    }
}

// ----------------------------------------------------------------------------
// KalmanFilter1D
// ----------------------------------------------------------------------------
void testKalmanFirstUpdateMatchesHandComputation() {
    // q = 0, r = 0.1, x0 = 0, p0 = 1
    pure::common::KalmanFilter1D<double> filter(0.0, 0.1, 0.0, 1.0);
    CHECK_NEAR(filter.estimate(), 0.0, 1e-12);
    CHECK_NEAR(filter.error_covariance(), 1.0, 1e-12);

    // 预测后 p = 1；K = 1 / (1 + 0.1) = 0.909090...
    // x = 0 + K * (5 - 0) = 5 / 1.1；p = (1 - K) * 1 = 0.1 / 1.1
    CHECK_NEAR(filter.update(5.0), 5.0 / 1.1, 1e-12);
    CHECK_NEAR(filter.error_covariance(), 0.1 / 1.1, 1e-12);
}

void testKalmanConvergesToConstant() {
    pure::common::KalmanFilter1D<double> filter(1e-4, 1e-2, 0.0, 1.0);

    for (int i = 0; i < 200; ++i) {
        filter.update(5.0);
    }

    CHECK_NEAR(filter.estimate(), 5.0, 1e-2);
    CHECK(filter.error_covariance() > 0.0);
    CHECK(filter.error_covariance() < 1e-3); // 协方差收敛到稳态
}

void testKalmanSmoothsNoise() {
    pure::common::KalmanFilter1D<double> filter(1e-3, 1.0, 0.0, 1.0);

    std::vector<double> noisy;
    for (int i = 0; i < 200; ++i) {
        noisy.push_back((i % 2 == 0) ? 4.0 : 6.0); // 均值 5、方差 1
    }
    const std::vector<double> filtered = filter.apply(noisy);

    const std::vector<double> tail_filtered = last_n(filtered, 100);
    CHECK(variance(tail_filtered) < 0.5 * variance(last_n(noisy, 100)));
    CHECK(!filtered.empty());
    CHECK_NEAR(filtered.back(), 5.0, 0.1);
}

void testKalmanReset() {
    pure::common::KalmanFilter1D<double> filter(1e-3, 1e-2, 2.0, 1.0);
    filter.update(10.0);
    CHECK(filter.estimate() > 2.0);

    filter.reset();
    CHECK_NEAR(filter.estimate(), 2.0, 1e-12);
    CHECK_NEAR(filter.error_covariance(), 1.0, 1e-12);
}

void testKalmanInvalidParameters() {
    bool threw = false;
    try {
        pure::common::KalmanFilter1D<double> filter(-1e-3, 1e-2);
        (void)filter;
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);

    threw = false;
    try {
        pure::common::KalmanFilter1D<double> filter(1e-3, 0.0);
        (void)filter;
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
}

// ----------------------------------------------------------------------------
// 多态调用 + float 实例化
// ----------------------------------------------------------------------------
void testPolymorphicApply() {
    std::vector<pure::common::Filter<double>*> filters;
    filters.push_back(new pure::common::SlidingWindowFilter<double>(2));
    filters.push_back(new pure::common::MedianFilter<double>(2));
    filters.push_back(new pure::common::FirstOrderLowPassFilter<double>(0.5));
    filters.push_back(new pure::common::KalmanFilter1D<double>(1e-3, 1e-2));

    const std::vector<double> input = {1.0, 2.0, 3.0, 4.0};

    for (std::size_t i = 0; i < filters.size(); ++i) {
        const std::vector<double> output = filters[i]->apply(input);
        CHECK(output.size() == input.size());
    }

    // 滑动平均 / 中值 / 一阶低通的首个输出都直接等于首个输入
    for (std::size_t i = 0; i < 3; ++i) {
        const std::vector<double> output = filters[i]->apply(input);
        CHECK_NEAR(output.front(), input.front(), 1e-9);
    }

    // 卡尔曼滤波从初值 0 出发，首个输出严格介于初值与观测之间
    const std::vector<double> kalman_output = filters[3]->apply(input);
    CHECK(kalman_output.front() > 0.0);
    CHECK(kalman_output.front() < input.front());

    for (std::size_t i = 0; i < filters.size(); ++i) {
        delete filters[i]; // 基类虚析构，安全
    }
}

void testFloatInstantiation() {
    pure::common::SlidingWindowFilter<float> filter(2);

    const std::vector<float> input    = {1.0f, 3.0f, 5.0f};
    const std::vector<float> expected = {1.0f, 2.0f, 4.0f};

    CHECK_SERIES(filter.apply(input), expected, 1e-5);

    pure::common::KalmanFilter1D<float> kalman(1e-3f, 1e-2f, 0.0f, 1.0f);
    CHECK(kalman.update(5.0f) > 0.0f);
}

} // namespace

using pure_nav_test::run;

int main() {
    std::cout << "===== common_libs/filters 单元测试 =====" << std::endl;

    run("SlidingWindowFilter 基本滑动平均", testSlidingWindowBasic);
    run("SlidingWindowFilter 窗口为 1 直通", testSlidingWindowWindowSizeOne);
    run("SlidingWindowFilter 在线更新与 reset", testSlidingWindowOnlineAndReset);
    run("SlidingWindowFilter apply 前重置状态", testSlidingWindowApplyResetsState);
    run("SlidingWindowFilter 空输入", testSlidingWindowEmptyInput);
    run("SlidingWindowFilter 非法窗口抛异常", testSlidingWindowInvalidWindow);

    run("MedianFilter 抑制脉冲野值", testMedianRejectsSpike);
    run("MedianFilter 偶数窗口取中间两值平均", testMedianEvenWindow);
    run("MedianFilter 窗口为 2", testMedianWindowSizeTwo);
    run("MedianFilter reset 与空输入", testMedianResetAndEmptyInput);
    run("MedianFilter 非法窗口抛异常", testMedianInvalidWindow);

    run("FirstOrderLowPass alpha=1 直通", testLowPassAlphaOneIsPassthrough);
    run("FirstOrderLowPass 阶跃响应", testLowPassStepResponse);
    run("FirstOrderLowPass 平滑交替噪声", testLowPassSmoothsAlternatingNoise);
    run("FirstOrderLowPass 非法 alpha 抛异常", testLowPassInvalidAlpha);

    run("KalmanFilter1D 首次更新手算校验", testKalmanFirstUpdateMatchesHandComputation);
    run("KalmanFilter1D 收敛到常值", testKalmanConvergesToConstant);
    run("KalmanFilter1D 平滑观测噪声", testKalmanSmoothsNoise);
    run("KalmanFilter1D reset", testKalmanReset);
    run("KalmanFilter1D 非法参数抛异常", testKalmanInvalidParameters);

    run("基类指针多态调用", testPolymorphicApply);
    run("float 显式实例化可用", testFloatInstantiation);

    return pure_nav_test::summary();
}
