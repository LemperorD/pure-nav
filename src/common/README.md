# common

通用模块，存放共享内存（shared memory）、通用库（common_libs）、类型别名（type_alias）等

## common_libs/filters

模板化数字滤波器。头文件 `common_libs/include/filters.hpp` 只放类模板**声明**，
实现与显式实例化（explicit instantiation）在 `common_libs/src/filters.cpp`，
当前支持 `float` / `double` 两种标量类型。

| 滤波器 | 说明 |
|---|---|
| `SlidingWindowFilter<T>`（别名 `MovingAverageFilter<T>`） | 滑动窗口均值滤波，窗口未满时按已有样本求平均 |
| `MedianFilter<T>` | 滑动窗口中值滤波，对脉冲野值不敏感 |
| `FirstOrderLowPassFilter<T>` | 一阶低通 / 指数加权移动平均（EMA） |
| `KalmanFilter1D<T>` | 一维卡尔曼滤波（常值状态 + 随机游走过程模型） |

统一接口（继承自 `Filter<T>`）：

- `update(v)`：在线逐点滤波，喂入一个样本返回当前输出；
- `apply(data)`：离线批量滤波，先 `reset()` 再逐点处理，返回等长结果；
- `reset()`：清空历史状态。

构造参数非法（窗口为 0、`alpha` 越界、噪声为负等）时抛 `std::invalid_argument`。

```cpp
pure::common::SlidingWindowFilter<double> filter(5);
double smoothed = filter.update(3.14);              // 在线
std::vector<double> out = filter.apply(samples);    // 离线
```

## 测试

`src/test/test_common/` 下三个测试目标共用 `test_check.hpp` 迷你框架，不依赖第三方库：

| 目标 | 内容 |
|---|---|
| `test_common` | 手工构造小数据的单元测试（边界、异常、多态、float 实例化） |
| `test_filters_data` | 读取 `src/test/test_data/filters/*.csv` 的数据驱动测试：与 Python 参考实现的 golden 逐点比对 + 降噪/稳态指标 |
| `test_filters_consumer` | 模拟下游模块引用：只链接 `pure_nav_common_libs`、不加 `-I`，验证 PUBLIC include 传递与显式实例化符号可链接 |

```bash
cmake -S . -B build -DPURE_NAV_BUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

数据文件的格式与重新生成方式见 `src/test/test_data/README.md`。
