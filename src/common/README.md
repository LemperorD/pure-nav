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

## shm

基于 iceoryx 的进程间共享内存发布 / 订阅封装：`ShmRuntime` 负责进程级运行时
初始化，`ShmPublisher<T>` / `ShmSubscriber<T>` 负责零拷贝收发，`FixedVector<T,N>`
解决点云 / 图像这类"长度运行时才知道"的定容变长数据。雷达点云、IMU、相机图像
都通过这里进共享内存，供各模块跨进程读取。

```cpp
pure::common::ShmRuntime::init("lidar_driver");                 // 多进程：连外部 iox-roudi
pure::common::ShmPublisher<PointCloudMsg> pub({"sensor/lidar/points"});
pub.publish([](PointCloudMsg& cloud) { /* 在共享内存里原地填充 */ });
```

详细的 API、消息类型约束、内存池配置与注意事项见
[`shm/README.md`](shm/README.md)。

## 测试

`src/test/test_common/` 下的测试目标共用 `test_check.hpp` 迷你框架：

| 目标 | 内容 |
|---|---|
| `test_common` | 手工构造小数据的单元测试（边界、异常、多态、float 实例化） |
| `test_filters_data` | 读取 `src/test/test_data/filters/*.csv` 的数据驱动测试：与 Python 参考实现的 golden 逐点比对 + 降噪/稳态指标 |
| `test_filters_consumer` | 模拟下游模块引用：只链接 `pure_nav_common_libs`、不加 `-I`，验证 PUBLIC include 传递与显式实例化符号可链接 |
| `test_shm` | 共享内存封装单元测试：进程内 RouDi，覆盖主题解析、IMU 往返、2 万点云零拷贝、队列语义 |
| `test_shm_cross_process` | 跨进程验证：fork 出真正的 `iox-roudi` 与订阅者子进程，验证多进程共享内存 |

```bash
cmake -S . -B build -DPURE_NAV_BUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

数据文件的格式与重新生成方式见 `src/test/test_data/README.md`。
