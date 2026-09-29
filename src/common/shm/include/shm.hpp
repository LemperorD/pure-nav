#pragma once

// ============================================================================
// shm —— 基于 iceoryx 的进程间共享内存发布 / 订阅封装
//
// 设计目标：把 iceoryx 的 PoshRuntime / Publisher / Subscriber 这套偏底层、
// 模板与回调混杂的 API，收敛成项目里"一眼就会用"的几个 class：
//
//   1. 进程启动时调用一次 ShmRuntime::init("进程名")，连接后台的 iox-roudi
//      守护进程；（单元测试 / 单进程可用 ShmRuntime::initSingleProcess）
//   2. 写数据的一方构造 ShmPublisher<T>，调用 publish(...) 发布；
//   3. 读数据的一方构造 ShmSubscriber<T>，调用 take(...) 领取。
//
// 通信走的是真正的共享内存零拷贝：publish 在共享内存里原地构造 T，订阅方
// take 到的是同一块内存的只读视图，不做序列化、不发网络包。
//
// ------------------------------- 重要约束 ---------------------------------
// * T 必须是 **可平凡复制（trivially copyable）** 的 POD，且不能包含指针 /
//   虚函数 / std::string / std::vector 等进程内地址相关成员——因为同一块内存
//   会被多个进程直接解释。变长数据请用 shm_payload.hpp 的 FixedVector<T,N>。
// * T 必须可默认构造：iceoryx 借出 chunk 时会先 new (p) T，再交给用户填充。
// * iceoryx 的 Publisher / Subscriber 本身**不是线程安全**的，每个对象只由
//   一个线程使用；跨线程请各持一个实例。
// * Publisher / Subscriber 必须**晚于** ShmRuntime 初始化创建，且不要做成全局
//   对象（静态析构顺序无法保证），建议在 main 内构造。
// * 一个进程只能初始化一次运行时，进程名（runtime name）在整机内必须唯一。
//
// ------------------------------- 最小示例 ---------------------------------
//   // 发送端进程
//   pure::common::ShmRuntime::init("lidar_driver");
//   pure::common::ShmPublisher<PointCloudMsg> pub({"sensor/lidar/points"});
//   pub.publish([&](PointCloudMsg& msg) { fill(msg); });   // 零拷贝原地填充
//
//   // 接收端进程
//   pure::common::ShmRuntime::init("planner");
//   pure::common::ShmSubscriber<PointCloudMsg> sub({"sensor/lidar/points"});
//   sub.take([](const PointCloudMsg& msg) { use(msg); });   // 零拷贝只读访问
// ============================================================================

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <iceoryx_posh/capro/service_description.hpp>
#include <iceoryx_posh/iceoryx_posh_types.hpp>
#include <iceoryx_posh/internal/popo/building_blocks/chunk_receiver.hpp>
#include <iceoryx_posh/internal/popo/building_blocks/chunk_sender.hpp>
#include <iceoryx_posh/popo/publisher.hpp>
#include <iceoryx_posh/popo/publisher_options.hpp>
#include <iceoryx_posh/popo/subscriber.hpp>
#include <iceoryx_posh/popo/subscriber_options.hpp>

namespace pure {
namespace common {

// ============================================================================
// ShmTopic —— iceoryx 主题（service / instance / event 三元组）
//
// iceoryx 用三个字符串唯一标识一条数据流；这里额外支持 "a/b/c" 这种简写，
// 方便在代码里像话题名一样直接写。
// ============================================================================
class ShmTopic {
public:
    ShmTopic() = default;

    ShmTopic(std::string service, std::string instance, std::string event);

    // 解析 "service/instance/event"。段数不对或为空时抛 std::invalid_argument。
    static ShmTopic fromString(const std::string& name);

    const std::string& service() const noexcept { return service_; }
    const std::string& instance() const noexcept { return instance_; }
    const std::string& event() const noexcept { return event_; }

    // 还原成 "service/instance/event"，便于日志输出。
    std::string name() const;

    bool valid() const noexcept;

    bool operator==(const ShmTopic& rhs) const noexcept;
    bool operator!=(const ShmTopic& rhs) const noexcept { return !(*this == rhs); }
    bool operator<(const ShmTopic& rhs) const noexcept;

private:
    std::string service_;
    std::string instance_;
    std::string event_;
};

// ============================================================================
// 运行时配置
// ============================================================================

// 一块内存池（mempool）描述。iceoryx 按"chunk 大小"分档预分配共享内存，
// 发布时从能装下 sizeof(消息) 的最小档位取一块。
// 例：点云消息约 300 KB -> 至少需要一个 chunk_size >= 300 KB 的池。
struct ShmMemPool {
    uint64_t chunk_size = 0;   // 单个 chunk 的字节数
    uint32_t chunk_count = 0;  // 该档位的 chunk 个数（决定队列深度与并发上限）
};

// 仅 initSingleProcess 使用：为空表示沿用 iceoryx 默认内存池
// （128B ~ 4MB 共 7 档）。跨进程部署时内存池由 iox-roudi 的 TOML 配置决定，
// 这里的配置不生效（见 config/iceoryx_roudi.toml）。
struct ShmRuntimeConfig {
    std::vector<ShmMemPool> mempools;
};

// ============================================================================
// ShmRuntime —— 进程级 iceoryx 运行时
//
// 必须在创建任何 Publisher / Subscriber 之前调用且只调用一次。
// ============================================================================
class ShmRuntime {
public:
    // 连接外部 iox-roudi（生产环境多进程用）。进程名需整机唯一。
    static void init(const std::string& app_name);

    // 进程内启动一个 RouDi（单进程内多线程通信 / 单元测试用），无需外部守护进程。
    // 定义在 shm_inproc.cpp，需要链接 pure_nav_shm_inproc。
    static void initSingleProcess(const std::string& app_name, const ShmRuntimeConfig& config = ShmRuntimeConfig());

    static bool initialized() noexcept;

    // 取当前可执行文件名作为默认进程名（/proc/self/comm）。
    static std::string defaultAppName();
};

// ============================================================================
// 发布 / 订阅返回值
// ============================================================================
enum class ShmPublishResult {
    OK = 0,
    NO_MEMPOOL,           // 没有能装下该消息的内存池：需调大 RouDi 的 chunk_size
    OUT_OF_CHUNKS,        // 内存池被占满：发布过快或 chunk_count 太小
    TOO_MANY_LOANS,       // 同一 publisher 同时借出的 chunk 超过上限
    INVALID_PARAMETER,    // 消息过大 / 参数非法
    ERROR,                // 其它未分类错误
};

enum class ShmTakeResult {
    OK = 0,
    EMPTY,                // 当前没有新数据
    TOO_MANY_HELD,        // 订阅者同时持有的样本数超过上限（忘了释放 Sample）
    ERROR,
};

const char* toString(ShmPublishResult result) noexcept;
const char* toString(ShmTakeResult result) noexcept;

// ============================================================================
// Publisher / Subscriber 选项
// ============================================================================
struct ShmPublisherConfig {
    // 历史样本数：>0 时新订阅者可以回溯最近若干帧
    uint64_t history_capacity = 0;
    // 创建后立即对外 offer（订阅者才能连上）
    bool offer_on_create = true;
};

struct ShmSubscriberConfig {
    // 接收队列长度；0 表示用 iceoryx 默认值（256）
    uint64_t queue_capacity = 0;
    // 订阅时请求的历史样本数（需要 publisher 的 history_capacity > 0）
    uint64_t history_request = 0;
    // 创建后立即订阅
    bool subscribe_on_create = true;
};

namespace detail {

iox::capro::ServiceDescription makeServiceDescription(const ShmTopic& topic);
iox::popo::PublisherOptions makePublisherOptions(const ShmPublisherConfig& config) noexcept;
iox::popo::SubscriberOptions makeSubscriberOptions(const ShmSubscriberConfig& config) noexcept;

inline ShmPublishResult toPublishResult(iox::popo::AllocationError error) noexcept {
    switch (error) {
    case iox::popo::AllocationError::NO_MEMPOOLS_AVAILABLE:
        return ShmPublishResult::NO_MEMPOOL;
    case iox::popo::AllocationError::RUNNING_OUT_OF_CHUNKS:
        return ShmPublishResult::OUT_OF_CHUNKS;
    case iox::popo::AllocationError::TOO_MANY_CHUNKS_ALLOCATED_IN_PARALLEL:
        return ShmPublishResult::TOO_MANY_LOANS;
    case iox::popo::AllocationError::INVALID_PARAMETER_FOR_USER_PAYLOAD_OR_USER_HEADER:
    case iox::popo::AllocationError::INVALID_PARAMETER_FOR_REQUEST_HEADER:
        return ShmPublishResult::INVALID_PARAMETER;
    case iox::popo::AllocationError::UNDEFINED_ERROR:
    default:
        return ShmPublishResult::ERROR;
    }
}

inline ShmTakeResult toTakeResult(iox::popo::ChunkReceiveResult result) noexcept {
    switch (result) {
    case iox::popo::ChunkReceiveResult::NO_CHUNK_AVAILABLE:
        return ShmTakeResult::EMPTY;
    case iox::popo::ChunkReceiveResult::TOO_MANY_CHUNKS_HELD_IN_PARALLEL:
        return ShmTakeResult::TOO_MANY_HELD;
    default:
        return ShmTakeResult::ERROR;
    }
}

} // namespace detail

// ============================================================================
// ShmPublisher<T> —— 共享内存发布者
//
// 只允许一个线程使用同一个实例。创建即 offer。
// ============================================================================
template <typename T>
class ShmPublisher {
public:
    static_assert(!std::is_const<T>::value, "共享内存消息类型不能是 const");
    static_assert(!std::is_pointer<T>::value, "共享内存消息类型不能是指针");
    static_assert(std::is_trivially_copyable<T>::value,
                  "共享内存消息类型必须是可平凡复制的 POD（不要用 std::string/std::vector/指针/虚函数）");
    static_assert(std::is_default_constructible<T>::value,
                  "共享内存消息类型必须可默认构造（iceoryx 借出 chunk 时会先构造再交给发布方填充）");

    explicit ShmPublisher(const ShmTopic& topic, const ShmPublisherConfig& config = ShmPublisherConfig())
        : topic_(topic)
        , publisher_(std::make_unique<iox::popo::Publisher<T>>(detail::makeServiceDescription(topic),
                                                              detail::makePublisherOptions(config))) {
    }

    ShmPublisher(const ShmPublisher&) = delete;
    ShmPublisher& operator=(const ShmPublisher&) = delete;

    // 拷贝发布：把 value 拷贝进共享内存。适合小消息。
    ShmPublishResult publishCopy(const T& value) noexcept {
        const auto result = publisher_->publishCopyOf(value);
        return toPublishResult(result);
    }

    // 零拷贝发布：在共享内存中原地填充。
    // fill 可以是 void(T&) 或 void(T*) 两种签名，避免多一次大块内存拷贝。
    template <typename Fill>
    ShmPublishResult publish(Fill&& fill) noexcept {
        const auto result = publisher_->publishResultOf([&fill](T* slot) {
            if constexpr (std::is_invocable_v<Fill, T&>) {
                fill(*slot);
            } else {
                fill(slot);
            }
        });
        return toPublishResult(result);
    }

    // 当前是否有订阅者；无订阅者时发布的数据会被直接丢弃（非阻塞语义）。
    bool hasSubscribers() const noexcept {
        return publisher_->hasSubscribers();
    }

    const ShmTopic& topic() const noexcept {
        return topic_;
    }

private:
    static ShmPublishResult toPublishResult(const iox::expected<void, iox::popo::AllocationError>& result) noexcept {
        if (!result.has_error()) {
            return ShmPublishResult::OK;
        }
        return detail::toPublishResult(result.error());
    }

    ShmTopic topic_;
    std::unique_ptr<iox::popo::Publisher<T>> publisher_;
};

// ============================================================================
// ShmSubscriber<T> —— 共享内存订阅者
//
// 只允许一个线程使用同一个实例。取到的数据是共享内存中的只读视图，
// 回调返回后即失效，需要留存请自行拷贝。
// ============================================================================
template <typename T>
class ShmSubscriber {
public:
    static_assert(!std::is_const<T>::value, "共享内存消息类型不能是 const");
    static_assert(!std::is_pointer<T>::value, "共享内存消息类型不能是指针");
    static_assert(std::is_trivially_copyable<T>::value,
                  "共享内存消息类型必须是可平凡复制的 POD（不要用 std::string/std::vector/指针/虚函数）");

    explicit ShmSubscriber(const ShmTopic& topic, const ShmSubscriberConfig& config = ShmSubscriberConfig())
        : topic_(topic)
        , subscriber_(std::make_unique<iox::popo::Subscriber<T>>(detail::makeServiceDescription(topic),
                                                                detail::makeSubscriberOptions(config))) {
    }

    ShmSubscriber(const ShmSubscriber&) = delete;
    ShmSubscriber& operator=(const ShmSubscriber&) = delete;

    // 领取一个样本并交给 handler（handler 签名为 void(const T&)）。
    // 成功返回 OK，队列为空返回 EMPTY。
    template <typename Handler>
    ShmTakeResult take(Handler&& handler) noexcept {
        auto result = subscriber_->take();
        if (result.has_error()) {
            return detail::toTakeResult(result.error());
        }
        handler(static_cast<const T&>(*result.value()));
        return ShmTakeResult::OK;
    }

    // 领取一个样本并拷贝到 out。
    ShmTakeResult takeCopy(T& out) noexcept {
        static_assert(std::is_copy_assignable<T>::value, "takeCopy 需要消息类型可拷贝赋值");
        return take([&out](const T& value) { out = value; });
    }

    // 取空当前队列，返回取到的样本数。
    template <typename Handler>
    std::size_t drain(Handler&& handler) noexcept {
        std::size_t count = 0;
        while (take(handler) == ShmTakeResult::OK) {
            ++count;
        }
        return count;
    }

    bool hasData() const noexcept {
        return subscriber_->hasData();
    }

    // 自上次调用以来是否有数据因队列溢出被丢弃。
    bool hasMissedData() noexcept {
        return subscriber_->hasMissedData();
    }

    // 是否已完成订阅握手（刚创建时需要等一小会儿）。
    bool subscribed() const noexcept {
        return subscriber_->getSubscriptionState() == iox::SubscribeState::SUBSCRIBED;
    }

    // 阻塞等待订阅握手完成。
    bool waitForSubscription(std::chrono::milliseconds timeout) const noexcept {
        return waitUntil(timeout, [this] { return subscribed(); });
    }

    // 阻塞等待第一个样本到来。内部为轮询实现，超时返回 false。
    bool waitForData(std::chrono::milliseconds timeout) const noexcept {
        return waitUntil(timeout, [this] { return this->hasData(); });
    }

    const ShmTopic& topic() const noexcept {
        return topic_;
    }

private:
    template <typename Predicate>
    static bool waitUntil(std::chrono::milliseconds timeout, Predicate predicate) noexcept {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (true) {
            if (predicate()) {
                return true;
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                return predicate();
            }
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
    }

    ShmTopic topic_;
    std::unique_ptr<iox::popo::Subscriber<T>> subscriber_;
};

// 兼容 shm.hpp 早期版本的命名
template <typename T>
using ShMWriter = ShmPublisher<T>;
template <typename T>
using ShMReader = ShmSubscriber<T>;

} // namespace common
} // namespace pure
