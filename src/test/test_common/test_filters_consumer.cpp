// test_filters_consumer.cpp —— 模拟"下游模块"引用 common_libs/filters
//
// 这个目标在 CMake 里**只链接 pure_nav_common_libs**，没有额外添加任何 include
// 目录。因此它能编译、链接并跑通，本身就证明了三件事：
//
//   1. filters.hpp 能通过 target_link_libraries(... PUBLIC ...) 的 include 传递
//      被下游目标找到（下游不需要关心头文件放在哪）；
//   2. filters.cpp 里显式实例化（float / double）的模板符号能被独立目标链接，
//      不会出现 "undefined reference to SlidingWindowFilter<double>::update"；
//   3. 别的模块可以像用普通库一样使用滤波器：既可以组合进自己的类，也可以用
//      基类指针多态调用（虚函数表同样被正确链接）。
//
// 真实模块（odometry / perception 等）接入方式与此完全一致，只是把这里的
// RangeSmoother 换成模块自己的封装。

#include "filters.hpp"
#include "test_check.hpp"

#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>
#include <vector>

namespace {

// 下游模块里常见的封装方式：把滤波器组合进自己的类
class RangeSmoother {
public:
    explicit RangeSmoother(std::size_t window) : filter_(window) {}

    double push(double raw) { return filter_.update(raw); }
    void reset() { filter_.reset(); }

private:
    pure::common::SlidingWindowFilter<double> filter_;
};

// 确定性噪声，便于复现
double lcg_noise(unsigned& state, double amplitude) {
    state = state * 1103515245u + 12345u;
    const double unit = static_cast<double>((state >> 8) & 0xFFFFu) / 65535.0;
    return amplitude * (2.0 * unit - 1.0);
}

// 逐一使用每种滤波器的 double / float 版本，确保所有显式实例化的符号都参与链接
void touch_all_instantiations() {
    const std::vector<double> sample_d = {1.0, 2.0, 3.0, 4.0};
    const std::vector<float> sample_f  = {1.0f, 2.0f, 3.0f, 4.0f};

    pure::common::SlidingWindowFilter<double> moving_average_d(3);
    pure::common::MedianFilter<double> median_d(3);
    pure::common::FirstOrderLowPassFilter<double> low_pass_d(0.5);
    pure::common::KalmanFilter1D<double> kalman_d(1e-3, 1e-2);

    pure::common::SlidingWindowFilter<float> moving_average_f(3);
    pure::common::MedianFilter<float> median_f(3);
    pure::common::FirstOrderLowPassFilter<float> low_pass_f(0.5f);
    pure::common::KalmanFilter1D<float> kalman_f(1e-3f, 1e-2f);

    // 基类指针多态调用：顺带验证虚函数表能跨静态库正确链接
    std::vector<pure::common::Filter<double>*> filters;
    filters.push_back(&moving_average_d);
    filters.push_back(&median_d);
    filters.push_back(&low_pass_d);
    filters.push_back(&kalman_d);

    for (std::size_t i = 0; i < filters.size(); ++i) {
        const std::vector<double> output = filters[i]->apply(sample_d);
        ::pure_nav_test::report(output.size() == sample_d.size(), "下游模块：基类多态 apply 结果长度不一致", __FILE__, __LINE__);
    }

    ::pure_nav_test::report(moving_average_f.apply(sample_f).size() == sample_f.size(), "下游模块：SlidingWindowFilter<float> 不可用", __FILE__, __LINE__);
    ::pure_nav_test::report(median_f.apply(sample_f).size() == sample_f.size(), "下游模块：MedianFilter<float> 不可用", __FILE__, __LINE__);
    ::pure_nav_test::report(low_pass_f.apply(sample_f).size() == sample_f.size(), "下游模块：FirstOrderLowPassFilter<float> 不可用", __FILE__, __LINE__);
    ::pure_nav_test::report(kalman_f.apply(sample_f).size() == sample_f.size(), "下游模块：KalmanFilter1D<float> 不可用", __FILE__, __LINE__);
}

} // namespace

int main() {
    std::cout << "===== filters 下游模块引用测试（模拟 odometry 使用 common_libs）=====" << std::endl;

    // 1) 编译 + 链接：所有显式实例化类型都能被独立目标使用
    std::cout << "[ 引用 ] 头文件包含 + 静态库链接" << std::endl;
    touch_all_instantiations();

    // 2) 组合使用：模拟一段带噪声的距离传感器读数（20 -> 35 的阶跃）
    const std::size_t sample_count = 12;
    std::vector<double> truth;
    std::vector<double> raw;
    unsigned state = 20240607u;
    for (std::size_t i = 0; i < sample_count; ++i) {
        const double value = (i < 6) ? 20.0 : 35.0;
        truth.push_back(value);
        raw.push_back(value + lcg_noise(state, 3.0));
    }

    RangeSmoother smoother(4);
    std::vector<double> smoothed;
    for (std::size_t i = 0; i < raw.size(); ++i) {
        smoothed.push_back(smoother.push(raw[i]));
    }

    std::cout << "  下游模块调用滤波器后的运行结果:" << std::endl;
    std::cout << "    index        raw     smoothed      truth" << std::endl;
    for (std::size_t i = 0; i < raw.size(); ++i) {
        std::cout << "    " << i << "  " << raw[i] << "  " << smoothed[i] << "  " << truth[i] << std::endl;
    }

    // 3) 结果校验
    ::pure_nav_test::report(smoothed.size() == raw.size(), "下游模块：输出长度与输入不一致", __FILE__, __LINE__);
    ::pure_nav_test::report(pure_nav_test::roughness(smoothed) < pure_nav_test::roughness(raw), "下游模块：滤波后噪声未下降", __FILE__, __LINE__);

    const double steady_state_error = std::fabs(smoothed.back() - truth.back());
    std::cout << "  末样本: raw=" << raw.back() << "  smoothed=" << smoothed.back()
              << "  truth=" << truth.back() << "  |误差|=" << steady_state_error << std::endl;
    ::pure_nav_test::report(steady_state_error < 3.0, "下游模块：稳态偏离真值过大", __FILE__, __LINE__);

    return pure_nav_test::summary();
}
