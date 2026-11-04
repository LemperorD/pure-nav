# 环形缓冲区：接口与跨进程设计

## 当前实现

`ring_buffer.hpp` 提供 `pure::common::RingBuffer<T>`，兼容原来的全局名称
`ringbuffer<T>`。模板只做类型检查与转发；非模板存储、容量检查、回绕读写、
移动操作均在 `ring_buffer.cpp`，无需为每一种消息类型显式实例化。

```cmake
target_link_libraries(your_target PRIVATE pure_nav_ring_buffer)
```

```cpp
#include "ring_buffer.hpp"

ringbuffer<int> queue(1024);
const int input[] = {1, 2, 3};
const auto written = queue.write(input, 3);
int output[3];
const auto received = queue.read(output, 3);
```

- 容量和返回值按**元素个数**计，不是字节数。容量不必是 2 的幂。
- FIFO；容量全部可用；满时只写入空闲部分，保留旧数据，空时读取 0 个。
  调用方必须处理部分读写。原子消息应以一个消息结构体作为 T，不能忽略截断。
- `getOccupied()` / `getFree()` 可用于 const 对象；`clear()` 清空逻辑状态，
  不释放内存、不擦除底层字节。构造之后普通读写无分配。
- 零容量抛出 `std::invalid_argument`，字节数乘法溢出抛出 `std::length_error`；
  内存分配仍可能抛出标准分配异常。请求数非零时空指针一律抛出
  `std::invalid_argument`；请求数为零可传空指针。
- 仅接受可平凡复制且无 const/volatile 限定的 T，不支持 `std::string`、
  `std::vector` 等拥有资源的类型。调用方提供足够长且对象生命周期有效的输入/输出数组。
  可平凡复制不等于可以跨进程使用：裸指针、含指针的结构体仍能通过此检查。
- 拷贝构造/赋值执行深拷贝并保留队列状态；移动后源对象容量为零，读写返回零
  （仍检查空指针），可查询、清空、重新赋值。
- **非线程安全**，即使一个线程写、另一个线程读也必须加同一把外部锁；查询和
  clear 也需同步。读写、复制、移动期间不能有并发访问。

原代码中的问题：拷贝构造的 `this(rb.size)` 不合法，且未复制 wrap；默认赋值
浅拷贝裸指针导致重复释放；memcpy 对任意 T 使用没有限制；hpp/cpp 类名与
成员不匹配。当前实现用 RAII 字节存储和 occupied 计数统一状态，避免裸指针所有权。

## 项目内优先采用的跨进程路径

建议沿用 `common/shm` 的 iceoryx 发布/订阅：

```text
生产进程 → ShmPublisher<Msg> → 共享内存 → ShmSubscriber<Msg>
                                               ↓ 回调内复制
                                      本地 RingBuffer<Msg>
                                               ↓ 外部锁同步
                                            算法线程
```

`ShmSubscriberConfig::queue_capacity` 控制传输队列；本地环用于算法排队。
`take(handler)` 的 msg 仅在回调内有效，不能把 `&msg` 放入本地环后异步读取。
小消息可在回调内复制整个 Msg；大型点云/图像优先复制到预分配本地帧池，环中
只传递本地槽位句柄，并通过租约/引用计数保证帧尚在使用时不复用槽位。
本地帧池和租约不属于本次实现。

一个 FIFO 的 read 是消费操作：多个读取者会瓜分数据，**不是广播**。若定位、
感知都需要同一条数据，应各自使用独立订阅队列；IMU 时间历史查询还需要单独
设计按时间查找、多读游标及覆盖缺口检测，不能直接依赖当前 read 接口。

## 若需要自行实现共享内存环

应新增独立的 `SharedRingBuffer`，而不是把本类对象 memcpy 到共享内存。
本类的 vector 数据在进程堆上；两个进程即使看到相同指针值也不代表同一数据。

### 布局与生命周期

Linux 可使用 `shm_open` + `ftruncate` + `mmap(MAP_SHARED)`。共享区按以下方式布局：

```text
Header { magic, abi_version, header_size, element_size, capacity,
         payload_offset, mapping_size, session_id, ... }
同步元数据（按同步方案确定，并满足对齐要求）
固定长度槽位数组
```

共享区只保存固定布局数据、索引和相对偏移，不放普通堆指针、vector、string、
虚函数对象。消息类型需要固定字段宽度、明确对齐和 ABI 版本；进程间必须使用
兼容的架构与 ABI。不能用 static_assert(trivially_copyable) 代替逐字段布局审查。

创建端用 `O_CREAT | O_EXCL` 选出唯一初始化者，只在首次创建时设置长度和初始化
同步对象。已有对象的打开端不能无条件 ftruncate 或 placement new。建议由现有
进程管理层/Unix socket 在初始化完成后才通知其他进程 attach，避免在未构造完成
的同步对象上轮询。attach 前后校验文件长度、Header、偏移、容量乘法、对齐和版本。
若使用 READY 原子标志，其自身的构造与发布也必须有明确同步协议。

各进程独立 RAII 管理 fd、映射与本地视图；共享区中不用保存映射基址。
明确唯一的 unlink 管理方、关闭唤醒、旧会话失效和重启协议；不能在仍有访问者时
原地重建同步对象或清空环。创建者中途崩溃时由外部管理者隔离旧会话后重建。

### 第一版同步：进程共享锁

若吞吐量尚无实测瓶颈，先使用位于共享区内的 `pthread_mutex_t`，配置
`PTHREAD_PROCESS_SHARED`，在锁内完成完整消息复制和索引更新。需要阻塞等待时，
条件变量也配置为进程共享，使用谓词循环、超时和关闭标志；普通 `std::mutex`
没有可移植的跨进程保证。

配置 robust mutex 后，持锁进程死亡可能返回 `EOWNERDEAD`。这只说明拿到了
需要恢复的锁，不代表消息/索引完整。应在锁内丢弃不完整数据、重置队列并变更
session/报告数据缺口，然后调用 `pthread_mutex_consistent`；无法恢复时将会话
标记失效，由外部管理者协调所有访问者退出后重建。还需处理 `ENOTRECOVERABLE`。
跨进程、不同 pthread ABI 的兼容性不能默认成立。

### 优化版：单生产者、单消费者（SPSC）

仅在确定一个写者、一个读者时考虑无锁环。不要沿用当前 shared occupied/wrap
字段：它们会被双方同时修改。可使用两个分离并按目标平台缓存行对齐的索引，
写者独占 head，读者独占 tail。为避免满空歧义，物理槽位数为 N+1，逻辑容量为 N：

```text
producer:
    next = head + 1 == physical_count ? 0 : head + 1
    if next == tail.load(acquire): return FULL
    copy payload into slots[head]
    head.store(next, release)

consumer:
    if tail == head.load(acquire): return EMPTY
    copy payload from slots[tail] to local object
    tail.store(next(tail), release)
```

发布 head 前数据必须写完，发布 tail 前数据必须读完。消费者不得提前推进 tail
后继续持有槽位引用；复制出本地对象是最简单的生命周期约束。需要零拷贝时另设
claim/commit 与 acquire/release 租约，限制持有时长并处理进程死亡。

原子类型必须在目标 OS/架构/编译工具链上明确支持进程间、不同映射地址的操作；
仅看 `std::atomic` 或一次 `is_lock_free()` 不足以给出可移植的 IPC 保证。可评估
Boost IPC atomics 等明确提供 address-free 保证的实现。`volatile` 不能替代同步。

SPSC 满时先返回 FULL/丢新数据；生产者不能擅自修改 tail 丢旧数据，否则可能覆盖
消费者正在读取的槽位。多生产者/多消费者不能只给索引加 atomic：第一版使用共享锁，
确有需求再采用经过验证的逐槽序号算法。广播则需每订阅者独立队列/游标及独立的
槽位生命周期管理。

### 自研 IPC 的验收项（当前尚未实现）

- 使用独立 exec 进程和不同映射地址收发；校验顺序、校验和、满空和多次回绕。
- 竞争创建、初始化中途死亡、版本不匹配、非法容量与截短映射能明确失败。
- 写入中/读取中 kill 进程、超时、正常关闭、重启后旧会话失效，无无限阻塞。
- 慢消费者策略明确，未释放槽位不被复用；多订阅者数据与丢包统计彼此独立。
- 在实际部署 CPU 上压测吞吐、延迟与资源占用，验证弱内存序平台上的发布顺序。

## 参考

- [Linux shm_open](https://man7.org/linux/man-pages/man3/shm_open.3.html)
- [POSIX 进程共享 mutex](https://man7.org/linux/man-pages/man3/pthread_mutexattr_setpshared.3.html)
- [Robust mutex 与恢复](https://man7.org/linux/man-pages/man3/pthread_mutexattr_setrobust.3.html)
- [Boost IPC 原子操作与 address-free 保证](https://www.boost.org/doc/libs/1_87_0/libs/atomic/doc/html/atomic/interface.html)
