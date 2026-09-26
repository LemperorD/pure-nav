# livox_driver —— 览沃(Livox)雷达驱动

封装 Livox-SDK2，向上提供一个 C++ 驱动类。核心目标：**SDK 回调线程零负担、点云与
IMU 互不干扰、丢包可观测**。

```
include/livox_driver/
    livox_types.hpp      公共数据类型、解码、组帧、配置校验（不依赖 SDK 头）
    blocking_queue.hpp   预分配环形缓冲的单生产者/单消费者阻塞队列
    livox_driver.hpp     驱动类 LivoxDriver
src/
    livox_types.cpp      线格式解码 + FrameAssembler + 配置校验
    livox_driver.cpp     SDK 生命周期、全局回调路由、两个工作线程
```

## 快速使用

```cpp
#include "livox_driver/livox_driver.hpp"

pure::driver::LivoxDriver lidar;

lidar.setPointCloudCallback([](const pure::driver::PointCloudFrame& frame) {
    // 运行在点云线程；frame.points 单位 m，可直接喂感知/建图
    (void)frame;
});
lidar.setImuCallback([](const pure::driver::ImuSample& imu) {
    // 运行在 IMU 线程；acc 单位 m/s^2，gyro 单位 rad/s
    (void)imu;
});

pure::driver::LivoxConfig config;
config.config_path = "config/livox_mid360.json"; // 见仓库根目录 config/
config.sn = "";                                  // 空 = 绑定第一个发现的雷达
config.enable_imu = true;
config.verbose = true;

if (!lidar.start(config)) { /* 配置非法或 SDK 初始化失败，看 stderr */ }

// 雷达可能上电比程序晚，可在这里阻塞等待（返回 false 表示超时）
lidar.waitForLidar(5000);

// ... 主循环 ...

pure::driver::LivoxStats stats = lidar.stats();
lidar.stop();
```

`config/livox_mid360.json` 里的 `host_ip` 必须改成**本机连接雷达那块网卡的 IP**，
否则 SDK 收不到任何数据（典型症状：`waitForLidar` 超时）。

## 线程模型

```
[SDK data_io_thread]  ← Livox-SDK2 内部唯一的数据收包线程
     |  DataHandler::Handle() 同时分发点云与 IMU（两者本来就在同一个线程）
     v
 pointCloudTrampoline / imuTrampoline          ← 本驱动
     |  只做：按 handle 查表 → memcpy 到栈上 RawPacket → 入队 → 立即返回
     v
 point_queue_ (RawPacket, 2KB)   imu_queue_ (ImuSample, 固定缓冲)
     |                                |
     v                                v
 pointThreadMain()                imuThreadMain()
     |  组帧 + 坐标解码（ms 级批处理）  |  单位换算 + 逐点分发（us 级）
     v                                v
 点云回调 / latestFrame()          IMU 回调 / latestImu()
```

## 点云与 IMU 要不要分成两个取流线程？

**结论：要分开。** 下面是依据，而不是拍脑袋。

### 1. SDK 内部本来就是串行的

读 Livox-SDK2 源码可以确认：

- `sdk_core/data_handler/data_handler.cpp` 的 `DataHandler::Handle()` 在**同一个函数**
  里按 `data_type` 分发点云与 IMU；
- 它的调用点是 `device_manager.cpp` 的 `OnData()`（`IOLoopDelegate::OnData`）；
- 点云口和 IMU 口的两个 socket 都注册在**同一个** `data_io_thread_`
  （`device_manager.cpp` 中 `data_io_thread_->GetLoop()->AddDelegate(...)`）。

也就是说，点云和 IMU 的回调**必然串行**在同一条 SDK 收包线程上。这带来两条硬约束：

1. **回调里绝不能做重活**。否则点云解码会把 IMU 回调一起卡住，而且收包线程一慢，
   内核 UDP 缓冲就会溢出丢包。所以回调只 `memcpy` + 入队。
2. **真正的并行只能在消费侧获得**。既然 SDK 侧无法并行，就要靠我们自己的消费线程
   把两条流拆开。

> 常见误区：以为给点云和 IMU 各注册一个回调就有了两条"取流线程"。实际上回调是
> SDK 在自己那条线程上调用的，注册几个回调都改变不了这一点。

### 2. 两条流的特征差异很大

| | 点云 | IMU |
|---|---|---|
| 速率 | ~10Hz 成帧，单帧上万点 | ~200Hz，每包几十字节 |
| 处理方式 | 攒够一帧再批处理 | 逐点、极短 |
| 单次耗时 | 组帧 + 乘除换算，可达数 ms | 几次浮点运算，us 级 |
| 时延敏感度 | 有几十 ms 的帧缓冲，宽松 | 直接喂 LIO/里程计，毫秒级抖动即恶化姿态 |

如果共用一个消费线程，每次整帧组装（数 ms）都会让紧随其后的 IMU 排队，形成
**周期性头阻塞(head-of-line blocking)**。IMU 恰恰是最怕这种抖动的流。

### 3. 分开之后还多了两个好处

- **背压隔离**：点云消费端变慢（下游感知卡顿、队列打满）时，IMU 依然按时分发，
  不会被点云的丢弃策略牵连。
- **独立启停**：`LivoxConfig::enable_imu = false` 时就完全不启动 IMU 线程；
  反过来只收 IMU 也一样，不必为一个空跑的线程付代价。

### 4. 什么时候不需要分开

如果 IMU 只是拿来打日志、不参与实时状态估计，那么单线程也够用，直接
`enable_imu = false` 即可。**是否拆分的判据是"IMU 是否参与实时估计"，而不是
"数据量大小"**——IMU 数据量很小，但时延价值很高。

### 5. 为什么不是三个线程

SDK 自己已经有一条收包线程，我们**不需要**再复制一条"SDK 取流线程"去轮询——SDK2
是回调推送模型，不是拉取模型。本驱动的"取流线程"就是上面两条消费者线程。

## 数据约定

| 字段 | 单位 / 取值 |
|---|---|
| `PointXYZI::x/y/z` | 米（SDK 的 mm/cm 在解码时换算） |
| `PointXYZI::intensity` | 0~255（原始 reflectivity） |
| `PointXYZI::tag` | 雷达原始标签位，原样透传 |
| `PointCloudFrame::timestamp_ns` | 帧首包时间戳(ns)，时基取决于授时方式 |
| `ImuSample::acc_*` | m/s²（由原始 g 值 × `kLivoxGravity`） |
| `ImuSample::gyro_*` | rad/s |
| `ImuSample::raw_*` | SDK 原始浮点值，便于排查与二次标定 |

原始单位由 `LivoxConfig::imu_acc_is_g` / `imu_gyro_is_rad_s` 控制，默认按
Mid360/HAP 的惯例（g 与 rad/s）。若换机型，先确认单位再改这两个开关。

## 组帧与丢包判定

- 以 SDK 的 `frame_cnt` 变化为帧边界；新 `frame_cnt` 到来时交出上一帧。
- `expected_packets` 由帧内观察到的最大 `udp_cnt + 1` 推断，
  `received >= expected` 时 `complete = true`。
- 流中断（雷达停转/拔网线）由点云线程的超时（`frame_timeout_ms`）触发半帧冲刷。
- 队列满时默认 `kDropOldest`（保最新），并把丢弃计入 `stats().packets_dropped`。

## 缓冲与实时性

两个队列都是**预分配环形缓冲**：`reset()` 时一次性分配，运行期 `push/pop` 不产生
堆分配。这是为了满足"SDK 回调线程不能因为 malloc 抖动变慢"的要求。默认容量：

- 点云 `512` 包（1000 包/s 量级 → 约 0.5s）
- IMU `1024` 采样（200Hz → 约 5s）

## 排障

| 症状 | 可能原因 |
|---|---|
| `LivoxLidarSdkInit 失败` | json 路径错误 / `host_ip` 与本机网卡不符 |
| `waitForLidar` 超时 | 雷达未上电、网线/网段不对、防火墙拦 UDP |
| `packets_dropped` 持续增长 | 点云消费端太慢，调大 `point_queue_capacity` 或优化下游 |
| `frames_incomplete` 偏高 | 网络丢包（检查网线与交换机），或主机负载过高 |
| `控制命令失败 ... kLivoxLidarStatusNotSupported` | 该机型不支持某条配置命令，可关掉 `configure_on_connect` |

## 测试

`src/test/test_driver/test_livox_driver.cpp` 覆盖解码、组帧、队列、配置等纯逻辑，
**不接硬件**：

```bash
cmake -S . -B build && cmake --build build -j
ctest --test-dir build --output-on-failure -R test_livox_driver
```

`livox_types.cpp` 在 SDK 可用时会用 `static_assert` 校验复刻的线格式结构体与
`livox_lidar_def.h` 逐字节一致；SDK 不可用时驱动退化为占位实现（`start()` 直接
失败并提示），但上述纯逻辑测试仍可运行。
