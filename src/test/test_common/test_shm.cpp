// ============================================================================
// test_shm.cpp —— src/common/shm 共享内存封装的单元测试
//
// 覆盖内容：
//   1. 主题解析 / 校验（ShmTopic）
//   2. 进程内 RouDi 运行时初始化（ShmRuntime::initSingleProcess）
//   3. IMU 级别小消息的发布 / 订阅往返
//   4. 点云级别大消息的零拷贝发布（FixedVector 定容变长容器）
//   5. 队列语义：空队列、多帧顺序、无订阅者发布
//
// 本测试在同进程内启动一个 RouDi（见 shm_inproc.cpp），因此不依赖外部
// iox-roudi 守护进程，ctest 可以直接跑。真正的跨进程验证见
// test_shm_cross_process.cpp。
// ============================================================================

#include "shm.hpp"
#include "shm_payload.hpp"
#include "test_check.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {

using pure::common::ShmPublishResult;
using pure::common::ShmPublisher;
using pure::common::ShmRuntime;
using pure::common::ShmSubscriber;
using pure::common::ShmTakeResult;
using pure::common::ShmTopic;

// --- 测试用消息类型（必须是可平凡复制的 POD） -------------------------------

struct ImuSample {
    uint64_t timestamp_ns = 0;
    float acc[3] = {0.0F, 0.0F, 0.0F};
    float gyro[3] = {0.0F, 0.0F, 0.0F};
};

struct PointXYZI {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
};

// 单帧点云：2 万个点，约 320 KB，落在 iceoryx 默认的 512 KB 内存池里
constexpr std::size_t kPointCount = 20000;
using PointCloudMsg = pure::common::FixedVector<PointXYZI, kPointCount>;

// --- 辅助函数 ---------------------------------------------------------------

template <typename T>
bool waitConnected(ShmPublisher<T>& publisher, ShmSubscriber<T>& subscriber) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
        if (subscriber.subscribed() && publisher.hasSubscribers()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return subscriber.subscribed() && publisher.hasSubscribers();
}

// --- 用例 -------------------------------------------------------------------

void test_fixed_vector_layout() {
    pure::common::FixedVector<uint32_t, 4> values;
    CHECK(values.empty());
    CHECK(values.capacity() == 4);

    CHECK(values.push_back(10));
    CHECK(values.push_back(20));
    CHECK(values.length() == 2);
    CHECK(values[0] == 10);
    CHECK(values.back() == 20);

    values.clear();
    CHECK(values.empty());

    values.resize(100);  // 超容量应被截断
    CHECK(values.length() == 4);
    CHECK(values.full());
    CHECK(!values.push_back(1));

    // FixedVector 必须保持可平凡复制，才能安全地放进共享内存
    static_assert(std::is_trivially_copyable<pure::common::FixedVector<uint32_t, 4>>::value,
                  "FixedVector 必须是可平凡复制的");
}

void test_topic_parse() {
    const ShmTopic topic("sensor", "lidar", "points");
    CHECK(topic.valid());
    CHECK(topic.name() == "sensor/lidar/points");
    CHECK(topic.service() == "sensor");
    CHECK(topic.instance() == "lidar");
    CHECK(topic.event() == "points");

    const ShmTopic parsed = ShmTopic::fromString("sensor/imu/sample");
    CHECK(parsed == ShmTopic("sensor", "imu", "sample"));
    CHECK(parsed.name() == "sensor/imu/sample");
    CHECK(parsed != topic);

    bool threw = false;
    try {
        ShmTopic::fromString("only-two/segments");
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);

    threw = false;
    try {
        ShmTopic::fromString("too/many/segments/here");
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);

    threw = false;
    try {
        ShmTopic("sensor", "", "points");
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);

    // 超过 iceoryx 单段上限（100 字符）应被拒绝
    threw = false;
    try {
        ShmTopic(std::string(200, 'x'), "instance", "event");
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
}

void test_runtime_state() {
    CHECK(ShmRuntime::initialized());
    CHECK(!ShmRuntime::defaultAppName().empty());

    // 重复初始化必须是安全的空操作（iceoryx 一个进程只能初始化一次）
    ShmRuntime::initSingleProcess("shm_test_again");
    ShmRuntime::init("shm_test_external");
    CHECK(ShmRuntime::initialized());
}

void test_imu_roundtrip() {
    const ShmTopic topic = ShmTopic::fromString("test/imu/sample");
    ShmPublisher<ImuSample> publisher(topic);
    ShmSubscriber<ImuSample> subscriber(topic);

    CHECK(waitConnected(publisher, subscriber));

    ImuSample sent;
    sent.timestamp_ns = 123456789ULL;
    sent.acc[0] = 1.5F;
    sent.acc[1] = -2.25F;
    sent.acc[2] = 9.81F;
    sent.gyro[0] = 0.125F;
    sent.gyro[1] = -0.5F;
    sent.gyro[2] = 0.0F;

    CHECK(publisher.publishCopy(sent) == ShmPublishResult::OK);
    CHECK(subscriber.waitForData(std::chrono::milliseconds(1000)));

    ImuSample received;
    CHECK(subscriber.takeCopy(received) == ShmTakeResult::OK);
    CHECK(received.timestamp_ns == sent.timestamp_ns);
    CHECK_NEAR(received.acc[0], sent.acc[0], 1e-6);
    CHECK_NEAR(received.acc[1], sent.acc[1], 1e-6);
    CHECK_NEAR(received.gyro[1], sent.gyro[1], 1e-6);

    // 取空后队列应为空
    ImuSample extra;
    CHECK(subscriber.takeCopy(extra) == ShmTakeResult::EMPTY);
}

void test_pointcloud_zero_copy() {
    const ShmTopic topic = ShmTopic::fromString("test/lidar/points");
    ShmPublisher<PointCloudMsg> publisher(topic);
    ShmSubscriber<PointCloudMsg> subscriber(topic);

    CHECK(waitConnected(publisher, subscriber));

    // 原地填充共享内存，避免一次 320 KB 的拷贝
    const ShmPublishResult published = publisher.publish([&](PointCloudMsg& cloud) {
        cloud.resize(kPointCount);
        for (std::size_t i = 0; i < kPointCount; ++i) {
            cloud[i].x = static_cast<float>(i);
            cloud[i].y = static_cast<float>(i) * 2.0F;
            cloud[i].z = static_cast<float>(i) * 3.0F;
            cloud[i].intensity = static_cast<float>(i % 100);
        }
    });
    CHECK(published == ShmPublishResult::OK);

    CHECK(subscriber.waitForData(std::chrono::milliseconds(1000)));

    std::size_t received_count = 0;
    double x_sum = 0.0;
    float first_intensity = -1.0F;
    const ShmTakeResult result = subscriber.take([&](const PointCloudMsg& cloud) {
        received_count = cloud.length();
        first_intensity = cloud[0].intensity;
        for (const PointXYZI& point : cloud) {
            x_sum += point.x;
        }
    });

    CHECK(result == ShmTakeResult::OK);
    CHECK(received_count == kPointCount);
    CHECK_NEAR(first_intensity, 0.0F, 1e-6);

    // 0 + 1 + ... + (kPointCount - 1)
    const double expected_sum = static_cast<double>(kPointCount - 1) * static_cast<double>(kPointCount) / 2.0;
    CHECK_NEAR(x_sum, expected_sum, 1.0);

    // 消息大小应落在 512 KB 内存池以内
    CHECK(sizeof(PointCloudMsg) > 300 * 1024);
    CHECK(sizeof(PointCloudMsg) < 512 * 1024);
}

void test_multiple_samples_in_order() {
    struct Counter {
        uint32_t value = 0;
    };

    const ShmTopic topic = ShmTopic::fromString("test/counter/sequence");
    ShmPublisher<Counter> publisher(topic);
    ShmSubscriber<Counter> subscriber(topic);

    CHECK(waitConnected(publisher, subscriber));

    constexpr uint32_t kCount = 5;
    for (uint32_t i = 0; i < kCount; ++i) {
        CHECK(publisher.publishCopy(Counter{i}) == ShmPublishResult::OK);
    }

    // RouDi 投递是异步的，收集到达 kCount 个或超时
    std::vector<uint32_t> received;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (received.size() < kCount && std::chrono::steady_clock::now() < deadline) {
        subscriber.drain([&received](const Counter& counter) { received.push_back(counter.value); });
        if (received.size() < kCount) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    CHECK(received.size() == kCount);
    bool ordered = received.size() == kCount;
    for (std::size_t i = 0; i < received.size() && ordered; ++i) {
        ordered = (received[i] == static_cast<uint32_t>(i));
    }
    CHECK(ordered);
    CHECK(!subscriber.hasMissedData());
}

void test_take_without_publisher_is_empty() {
    const ShmTopic topic = ShmTopic::fromString("test/orphan/reader");
    ShmSubscriber<ImuSample> subscriber(topic);

    ImuSample received;
    CHECK(subscriber.takeCopy(received) == ShmTakeResult::EMPTY);
    CHECK(!subscriber.hasData());
    // 没有发布者时，等待也不会拿到数据
    CHECK(!subscriber.waitForData(std::chrono::milliseconds(100)));
}

void test_publish_without_subscriber_is_dropped() {
    const ShmTopic topic = ShmTopic::fromString("test/orphan/writer");
    ShmPublisher<ImuSample> publisher(topic);

    CHECK(!publisher.hasSubscribers());

    ImuSample sample;
    sample.timestamp_ns = 42;
    // 无订阅者时数据被直接丢弃，但发布本身仍然是成功的（非阻塞语义）
    CHECK(publisher.publishCopy(sample) == ShmPublishResult::OK);
}

} // namespace

int main() {
    try {
        // 同进程内启动 RouDi，无需外部 iox-roudi
        ShmRuntime::initSingleProcess("pure_nav_test_shm");
    } catch (const std::exception& error) {
        std::cerr << "[FATAL] 初始化进程内共享内存运行时失败: " << error.what() << std::endl;
        return 1;
    }

    pure_nav_test::run("fixed_vector_layout", test_fixed_vector_layout);
    pure_nav_test::run("topic_parse", test_topic_parse);
    pure_nav_test::run("runtime_state", test_runtime_state);
    pure_nav_test::run("imu_roundtrip", test_imu_roundtrip);
    pure_nav_test::run("pointcloud_zero_copy", test_pointcloud_zero_copy);
    pure_nav_test::run("multiple_samples_in_order", test_multiple_samples_in_order);
    pure_nav_test::run("take_without_publisher_is_empty", test_take_without_publisher_is_empty);
    pure_nav_test::run("publish_without_subscriber_is_dropped", test_publish_without_subscriber_is_dropped);

    return pure_nav_test::summary();
}
