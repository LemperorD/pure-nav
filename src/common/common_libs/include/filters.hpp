#pragma once

#include <cstddef>
#include <deque>
#include <vector>

// ============================================================================
// filters —— 通用数字滤波器
//
// 使用约定：
//   1. 所有滤波器都继承抽象基类 Filter<T>，对外接口统一为：
//        - update(v)   在线逐点滤波：喂入一个新样本，返回当前滤波输出
//        - apply(data) 离线批量滤波：先 reset()，再对 data 逐点 update()，
//                      返回与输入等长的滤波结果
//        - reset()     清空历史状态，回到未初始化状态
//   2. 类模板的**声明**放在本头文件，**实现**放在 src/filters.cpp，并在该文件
//      末尾对 float / double 做显式实例化（explicit instantiation）。因此本库
//      默认只支持 float 与 double；若要支持其它标量类型（如 int、long double），
//      在 filters.cpp 的显式实例化列表补一行即可。
//   3. 构造函数参数非法（窗口长度为 0、系数越界等）时抛 std::invalid_argument。
// ============================================================================

namespace pure {
namespace common {

// ----------------------------------------------------------------------------
// Filter：滤波器抽象基类
// ----------------------------------------------------------------------------
template <typename T>
class Filter {
public:
    virtual ~Filter() = default;

    // 在线滤波：喂入一个新样本 value，返回当前滤波输出
    virtual T update(T value) = 0;

    // 离线滤波：先 reset()，再逐点 update()，返回与输入等长的结果
    virtual std::vector<T> apply(const std::vector<T>& data) = 0;

    // 清空内部状态
    virtual void reset() = 0;
};

// ----------------------------------------------------------------------------
// SlidingWindowFilter：滑动窗口均值滤波（滑动平均 / moving average）
//
// 维护一个长度为 window_size 的 FIFO 窗口，输出为窗口内样本的算术平均。
// 窗口未填满时（启动阶段）对已有样本求平均，因此第一个样本会原样输出。
// ----------------------------------------------------------------------------
template <typename T>
class SlidingWindowFilter : public Filter<T> {
public:
    explicit SlidingWindowFilter(std::size_t window_size);

    T update(T value) override;
    std::vector<T> apply(const std::vector<T>& data) override;
    void reset() override;

    std::size_t window_size() const { return window_size_; }
    std::size_t size() const { return window_.size(); }
    T sum() const { return sum_; }

private:
    std::size_t window_size_; // 窗口容量
    std::deque<T> window_;    // 当前窗口内的样本，front 为最旧样本
    T sum_;                   // 当前窗口内样本之和（增量维护）
};

// 语义别名：滑动窗口均值滤波 == 滑动平均滤波
template <typename T>
using MovingAverageFilter = SlidingWindowFilter<T>;

// ----------------------------------------------------------------------------
// MedianFilter：滑动窗口中值滤波
//
// 输出为窗口内样本的中位数：样本数为奇数取中间值，为偶数取中间两个值的平均。
// 与均值滤波相比，中值滤波对脉冲噪声（野值）不敏感，代价是需要排序。
// ----------------------------------------------------------------------------
template <typename T>
class MedianFilter : public Filter<T> {
public:
    explicit MedianFilter(std::size_t window_size);

    T update(T value) override;
    std::vector<T> apply(const std::vector<T>& data) override;
    void reset() override;

    std::size_t window_size() const { return window_size_; }
    std::size_t size() const { return window_.size(); }

private:
    std::size_t window_size_; // 窗口容量
    std::deque<T> window_;    // 当前窗口内的样本，front 为最旧样本
};

// ----------------------------------------------------------------------------
// FirstOrderLowPassFilter：一阶低通滤波（指数加权移动平均 EMA）
//
//   y[k] = alpha * x[k] + (1 - alpha) * y[k-1]，alpha ∈ (0, 1]
//
// 第一个样本直接作为初值（y[0] = x[0]）。alpha 越小输出越平滑、滞后越大；
// alpha = 1 时退化为直通（不做任何滤波）。
// ----------------------------------------------------------------------------
template <typename T>
class FirstOrderLowPassFilter : public Filter<T> {
public:
    explicit FirstOrderLowPassFilter(T alpha);

    T update(T value) override;
    std::vector<T> apply(const std::vector<T>& data) override;
    void reset() override;

    T alpha() const { return alpha_; }
    T value() const { return value_; }
    bool initialized() const { return initialized_; }

private:
    T alpha_;         // 滤波系数，越大越信任新样本
    T value_;         // 上一次的滤波输出
    bool initialized_; // 是否已经吃过第一个样本
};

// ----------------------------------------------------------------------------
// KalmanFilter1D：一维卡尔曼滤波（常值状态 + 随机游走过程模型）
//
// 状态 x 为待估计标量（例如某个传感器读数），过程噪声 q、观测噪声 r：
//   预测：p = p + q
//   更新：K = p / (p + r);  x = x + K * (z - x);  p = (1 - K) * p
//
// q 越大越信任观测（跟随快、抖动大），r 越大越信任模型（平滑强、滞后大）。
// 注意：本实现不做"首样本直接初始化"，构造后的 estimate 即为初值，符合标准
// 卡尔曼滤波语义；如需快速收敛可把 initial_error 设得相对 r 大一些。
// ----------------------------------------------------------------------------
template <typename T>
class KalmanFilter1D : public Filter<T> {
public:
    KalmanFilter1D(T process_noise, T measurement_noise, T initial_estimate = T(), T initial_error = T(1));

    T update(T value) override;
    std::vector<T> apply(const std::vector<T>& data) override;
    void reset() override;

    T estimate() const { return estimate_; }
    T error_covariance() const { return error_covariance_; }
    T process_noise() const { return process_noise_; }
    T measurement_noise() const { return measurement_noise_; }

private:
    T process_noise_;        // 过程噪声方差 q
    T measurement_noise_;    // 观测噪声方差 r
    T estimate_;             // 状态估计 x
    T error_covariance_;     // 估计误差协方差 p
    T initial_estimate_;     // reset() 时恢复的初值
    T initial_error_;        // reset() 时恢复的初始协方差
};

} // namespace common
} // namespace pure
