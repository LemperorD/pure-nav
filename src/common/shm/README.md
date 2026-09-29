# shm —— 基于 iceoryx 的共享内存发布 / 订阅

`shm.hpp` 把 [iceoryx](https://iceoryx.io/)（v2.95.8，`src/thirdparty/iceoryx`）
的运行时、发布者、订阅者三件套收敛成项目里"一眼就会用"的几个类，用于把雷达
点云、IMU、相机图像等传感器数据放进共享内存，供各模块零拷贝读取。

```
┌──────────────┐   publish    ┌──────────────────────────┐   take    ┌──────────────┐
│ 传感器驱动进程 │ ───────────▶ │  共享内存 chunk（零拷贝）  │ ────────▶ │ 感知/规划进程 │
│ ShmPublisher │              │  /dev/shm (RouDi 预分配)  │           │ ShmSubscriber│
└──────────────┘              └──────────────────────────┘           └──────────────┘
                                        ▲
                                        │ 端口注册 / 订阅握手
                                  ┌───────────┐
                                  │ iox-roudi │  （后台守护进程）
                                  └───────────┘
```

## 快速上手

### 多进程（生产环境）

1. 后台启动一次 RouDi 守护进程（整机只跑一个）：

   ```bash
   build/thirdparty/install/bin/iox-roudi -c config/iceoryx_roudi.toml
   ```

2. 每个进程在创建任何发布 / 订阅之前初始化一次运行时：

   ```cpp
   #include "shm.hpp"
   #include "shm_payload.hpp"

   struct ImuSample {
       uint64_t timestamp_ns = 0;
       float acc[3] = {};
       float gyro[3] = {};
   };

   struct PointXYZI { float x, y, z, intensity; };
   using PointCloudMsg = pure::common::FixedVector<PointXYZI, 20000>;

   // 驱动进程：写
   pure::common::ShmRuntime::init("lidar_driver");
   pure::common::ShmPublisher<PointCloudMsg> cloud_pub({"sensor/lidar/points"});
   cloud_pub.publish([&](PointCloudMsg& cloud) {   // 在共享内存里原地填充
       cloud.resize(points.size());
       std::copy(points.begin(), points.end(), cloud.begin());
   });

   // 感知进程：读
   pure::common::ShmRuntime::init("perception");
   pure::common::ShmSubscriber<PointCloudMsg> cloud_sub({"sensor/lidar/points"});
   cloud_sub.waitForData(std::chrono::milliseconds(100));
   cloud_sub.take([](const PointCloudMsg& cloud) { /* 零拷贝只读访问 */ });
   ```

### 单进程 / 单元测试

不启动外部守护进程，在进程内拉起一个 RouDi（需要链接 `pure_nav_shm_inproc`）：

```cpp
pure::common::ShmRuntime::initSingleProcess("my_test");
```

还可以自定义内存池：

```cpp
pure::common::ShmRuntimeConfig config;
config.mempools = {{4096, 1000}, {1024 * 1024, 20}};
pure::common::ShmRuntime::initSingleProcess("my_test", config);
```

## API 速查

| 类型 | 作用 |
|---|---|
| `ShmRuntime::init(name)` | 连接外部 `iox-roudi`，进程名整机唯一 |
| `ShmRuntime::initSingleProcess(name, config)` | 进程内 RouDi（测试 / 单进程） |
| `ShmRuntime::initialized()` / `defaultAppName()` | 状态查询 / 默认进程名 |
| `ShmTopic(service, instance, event)` | 主题；也支持 `ShmTopic::fromString("a/b/c")` |
| `ShmPublisher<T>::publish(fill)` | **零拷贝**发布：`fill` 拿 `T&`（或 `T*`）原地写 |
| `ShmPublisher<T>::publishCopy(value)` | 拷贝发布，适合小消息 |
| `ShmPublisher<T>::hasSubscribers()` | 是否有订阅者 |
| `ShmSubscriber<T>::take(handler)` | 取一个样本交给 `handler(const T&)`，零拷贝 |
| `ShmSubscriber<T>::takeCopy(value)` | 取一个样本并拷贝出来 |
| `ShmSubscriber<T>::drain(handler)` | 清空队列，返回取到的样本数 |
| `ShmSubscriber<T>::waitForData(timeout)` | 轮询等待数据（超时返回 false） |
| `ShmSubscriber<T>::subscribed()` | 订阅端口是否已就绪（不保证已有发布者） |
| `FixedVector<T, N>` | 定容变长容器，用于点云 / 图像这类"长度运行时才知道"的数据 |

返回码：

- `ShmPublishResult`：`OK` / `NO_MEMPOOL`（内存池不够大）/ `OUT_OF_CHUNKS`（池被占满）
  / `TOO_MANY_LOANS` / `INVALID_PARAMETER` / `ERROR`，可用 `toString()` 打印。
- `ShmTakeResult`：`OK` / `EMPTY` / `TOO_MANY_HELD` / `ERROR`。

## 消息类型约束（重要）

iceoryx 的共享内存 chunk 会被多个进程**直接按 C++ 内存布局解释**，因此 `T` 必须：

1. **可平凡复制**（`std::is_trivially_copyable`）——`shm.hpp` 里有 `static_assert` 兜底；
2. **可默认构造**——iceoryx 借出 chunk 时会先 `new (p) T`；
3. **不含进程内地址**——不要用 `std::string` / `std::vector` / 裸指针 / 虚函数 /
   `Eigen::MatrixXf` 等动态类型，改用 `FixedVector<T, N>` 或定长数组。

```cpp
// ✅ 可以
struct ImuSample { uint64_t t; float acc[3]; float gyro[3]; };
pure::common::FixedVector<PointXYZI, 20000>;

// ❌ 不可以（跨进程必崩）
struct Bad { std::vector<float> points; std::string frame_id; };
```

## 内存池（mempool）怎么配

发布消息时 iceoryx 从"能装下 `sizeof(T)` 的最小档位"取 chunk。默认内存池最大
4 MB（见 `iceoryx_posh` 的 `MePooConfig::setDefaults`），足够小点云和 IMU；
**彩色图像或高线束点云需要自定义**：

- 多进程：编辑 `config/iceoryx_roudi.toml`，保证有一档 `size >= sizeof(T) + 约 80 B`，
  然后带着 `-c` 启动 RouDi；
- 单进程：通过 `ShmRuntimeConfig::mempools` 传入。

配小了的表现是 `publish()` 返回 `NO_MEMPOOL`，用 `toString()` 能直接看出原因。

## 测试

```bash
cmake -S . -B build -DPURE_NAV_BUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure -R test_shm
```

- `test_shm`：进程内 RouDi，覆盖主题解析、IMU 小消息往返、2 万点云零拷贝、
  队列顺序、空队列 / 无订阅者等语义。
- `test_shm_cross_process`：启动外部 `iox-roudi`，父进程发布、fork 出的子进程
  订阅，验证真正的跨进程共享内存。

## 注意事项

- **一个进程只能初始化一次运行时**；`ShmRuntime::init` 重复调用是安全的空操作。
- 进程名（runtime name）在整机内必须唯一，否则 RouDi 会拒绝注册。
- `ShmPublisher` / `ShmSubscriber` **不是线程安全的**，每个实例只由一个线程使用。
- 订阅是异步握手：发布前若想确认订阅者已连上，请轮询 `publisher.hasSubscribers()`；
  `subscriber.subscribed()` 只表示订阅端口就绪，并不保证已有发布者。否则在
  `history_capacity = 0` 时首帧可能丢失（`test_shm_cross_process` 就是这么同步的）。
- 不要做成全局对象：静态析构顺序无法保证运行时还活着。
- 无订阅者时发布不会阻塞，数据被直接丢弃（返回 `OK`）。
