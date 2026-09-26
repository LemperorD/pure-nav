#include "filters.hpp"

#include <algorithm>
#include <stdexcept>

// ============================================================================
// filters 实现
//
// 头文件里只有类模板的声明（见 include/filters.hpp），所有成员函数的定义都在
// 本文件内。为了让其它翻译单元能够链接到这些符号，文件末尾使用显式实例化
// （explicit instantiation）生成 float / double 两个版本的具体代码。
// ============================================================================

namespace pure {
namespace common {

// ----------------------------------------------------------------------------
// SlidingWindowFilter
// ----------------------------------------------------------------------------
template <typename T>
SlidingWindowFilter<T>::SlidingWindowFilter(std::size_t window_size) : window_size_(window_size), sum_(T()) {
    if (window_size_ == 0) {
        throw std::invalid_argument("SlidingWindowFilter: window_size must be greater than 0");
    }
}

template <typename T>
T SlidingWindowFilter<T>::update(T value) {
    window_.push_back(value);
    sum_ += value;

    // 窗口满了就丢掉最旧的样本，保持 sum_ 与 window_ 一致
    if (window_.size() > window_size_) {
        sum_ -= window_.front();
        window_.pop_front();
    }

    return sum_ / static_cast<T>(window_.size());
}

template <typename T>
std::vector<T> SlidingWindowFilter<T>::apply(const std::vector<T>& data) {
    reset();

    std::vector<T> output;
    output.reserve(data.size());
    for (std::size_t i = 0; i < data.size(); ++i) {
        output.push_back(update(data[i]));
    }
    return output;
}

template <typename T>
void SlidingWindowFilter<T>::reset() {
    window_.clear();
    sum_ = T();
}

// ----------------------------------------------------------------------------
// MedianFilter
// ----------------------------------------------------------------------------
template <typename T>
MedianFilter<T>::MedianFilter(std::size_t window_size) : window_size_(window_size) {
    if (window_size_ == 0) {
        throw std::invalid_argument("MedianFilter: window_size must be greater than 0");
    }
}

template <typename T>
T MedianFilter<T>::update(T value) {
    window_.push_back(value);
    if (window_.size() > window_size_) {
        window_.pop_front();
    }

    // 复制一份再取中位数，不能破坏窗口内的顺序（需要按时间顺序淘汰最旧样本）
    std::vector<T> sorted(window_.begin(), window_.end());
    const std::size_t count = sorted.size();
    const std::size_t middle = count / 2;

    // nth_element 后 sorted[middle] 即为第 middle 小的元素
    std::nth_element(sorted.begin(), sorted.begin() + middle, sorted.end());
    const T upper = sorted[middle];

    if (count % 2 != 0) {
        return upper;
    }

    // 偶数个样本：中位数 = 中间两个值的平均，左半边最大值位于 middle - 1
    std::nth_element(sorted.begin(), sorted.begin() + (middle - 1), sorted.begin() + middle);
    const T lower = sorted[middle - 1];
    return (lower + upper) / static_cast<T>(2);
}

template <typename T>
std::vector<T> MedianFilter<T>::apply(const std::vector<T>& data) {
    reset();

    std::vector<T> output;
    output.reserve(data.size());
    for (std::size_t i = 0; i < data.size(); ++i) {
        output.push_back(update(data[i]));
    }
    return output;
}

template <typename T>
void MedianFilter<T>::reset() {
    window_.clear();
}

// ----------------------------------------------------------------------------
// FirstOrderLowPassFilter
// ----------------------------------------------------------------------------
template <typename T>
FirstOrderLowPassFilter<T>::FirstOrderLowPassFilter(T alpha) : alpha_(alpha), value_(T()), initialized_(false) {
    // 用 !(a && b) 的写法可以同时挡住 NaN
    if (!(alpha_ > T(0) && alpha_ <= T(1))) {
        throw std::invalid_argument("FirstOrderLowPassFilter: alpha must be in (0, 1]");
    }
}

template <typename T>
T FirstOrderLowPassFilter<T>::update(T value) {
    if (!initialized_) {
        // 第一个样本直接作为初值，避免从 0 缓慢爬升
        value_       = value;
        initialized_ = true;
    } else {
        value_ = alpha_ * value + (T(1) - alpha_) * value_;
    }
    return value_;
}

template <typename T>
std::vector<T> FirstOrderLowPassFilter<T>::apply(const std::vector<T>& data) {
    reset();

    std::vector<T> output;
    output.reserve(data.size());
    for (std::size_t i = 0; i < data.size(); ++i) {
        output.push_back(update(data[i]));
    }
    return output;
}

template <typename T>
void FirstOrderLowPassFilter<T>::reset() {
    value_       = T();
    initialized_ = false;
}

// ----------------------------------------------------------------------------
// KalmanFilter1D
// ----------------------------------------------------------------------------
template <typename T>
KalmanFilter1D<T>::KalmanFilter1D(T process_noise, T measurement_noise, T initial_estimate, T initial_error)
    : process_noise_(process_noise),
      measurement_noise_(measurement_noise),
      estimate_(initial_estimate),
      error_covariance_(initial_error),
      initial_estimate_(initial_estimate),
      initial_error_(initial_error) {
    if (!(process_noise_ >= T(0))) {
        throw std::invalid_argument("KalmanFilter1D: process_noise must be non-negative");
    }
    if (!(measurement_noise_ > T(0))) {
        throw std::invalid_argument("KalmanFilter1D: measurement_noise must be positive");
    }
    if (!(initial_error_ > T(0))) {
        throw std::invalid_argument("KalmanFilter1D: initial_error must be positive");
    }
}

template <typename T>
T KalmanFilter1D<T>::update(T value) {
    // 1) 预测：状态不变（常值模型），协方差增大
    error_covariance_ += process_noise_;

    // 2) 更新：用观测修正状态
    const T gain = error_covariance_ / (error_covariance_ + measurement_noise_);
    estimate_ += gain * (value - estimate_);
    error_covariance_ = (T(1) - gain) * error_covariance_;

    return estimate_;
}

template <typename T>
std::vector<T> KalmanFilter1D<T>::apply(const std::vector<T>& data) {
    reset();

    std::vector<T> output;
    output.reserve(data.size());
    for (std::size_t i = 0; i < data.size(); ++i) {
        output.push_back(update(data[i]));
    }
    return output;
}

template <typename T>
void KalmanFilter1D<T>::reset() {
    estimate_         = initial_estimate_;
    error_covariance_ = initial_error_;
}

// ----------------------------------------------------------------------------
// 显式实例化
//
// 头文件里只有声明，模板实现都在本文件，必须为每个受支持的标量类型各实例化
// 一次，否则使用方会在链接阶段报 "undefined reference"。
// 需要支持新类型（例如 int、long double）时，在下面照着补一组即可。
// ----------------------------------------------------------------------------
template class SlidingWindowFilter<double>;
template class MedianFilter<double>;
template class FirstOrderLowPassFilter<double>;
template class KalmanFilter1D<double>;

template class SlidingWindowFilter<float>;
template class MedianFilter<float>;
template class FirstOrderLowPassFilter<float>;
template class KalmanFilter1D<float>;

} // namespace common
} // namespace pure
