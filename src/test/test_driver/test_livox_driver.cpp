// test_livox_driver.cpp —— Livox 驱动的离线单元测试（不接硬件、不依赖 SDK）
//
// 覆盖的是驱动里"纯逻辑"的部分，这些正是最容易写错、又最难靠实机复现的地方：
//   * 线格式解码（mm/cm/球坐标 → m，双回波取哪一回波）
//   * 组帧与丢包判定（frame_cnt 边界、udp_cnt 推断期望包数）
//   * 有界队列的丢弃策略与 close 唤醒语义
//   * IMU 单位换算与时间戳顺延
//   * 配置校验
// SDK 生命周期、真实取流时延等需要实机验证，不在本测试范围内。

#include "livox_driver/blocking_queue.hpp"
#include "livox_driver/livox_driver.hpp"
#include "livox_driver/livox_types.hpp"

#include "test_check.hpp"

#include <cstring>
#include <string>
#include <vector>

namespace {

#pragma pack(push, 1)
struct TestHighPoint {
    int32_t x;
    int32_t y;
    int32_t z;
    uint8_t reflectivity;
    uint8_t tag;
};

struct TestLowPoint {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t reflectivity;
    uint8_t tag;
};

struct TestSphericalPoint {
    uint32_t depth;
    uint16_t theta;
    uint16_t phi;
    uint8_t reflectivity;
    uint8_t tag;
};

struct TestDoubleEchoPoint {
    int32_t x1;
    int32_t y1;
    int32_t z1;
    uint8_t reflectivity1;
    uint8_t tag1;
    int32_t x2;
    int32_t y2;
    int32_t z2;
    uint8_t reflectivity2;
    uint8_t tag2;
};

struct TestImuPoint {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
};
#pragma pack(pop)

using pure::driver::FrameAssembler;
using pure::driver::ImuSample;
using pure::driver::LivoxConfig;
using pure::driver::LidarDataType;
using pure::driver::PointCloudFrame;
using pure::driver::PointXYZI;
using pure::driver::RawPacket;

RawPacket makeHighPacket(uint32_t handle,
                         uint8_t frame_cnt,
                         uint16_t udp_cnt,
                         const std::vector<TestHighPoint>& points,
                         uint64_t timestamp_ns) {
    RawPacket packet;
    packet.handle = handle;
    packet.dev_type = 9; // Mid360
    packet.data_type = pure::driver::kDataTypeCartesianHigh;
    packet.frame_cnt = frame_cnt;
    packet.udp_cnt = udp_cnt;
    packet.timestamp_ns = timestamp_ns;
    packet.dot_num = static_cast<uint16_t>(points.size());
    packet.data_len = static_cast<uint16_t>(points.size() * sizeof(TestHighPoint));
    if (packet.data_len > 0) {
        std::memcpy(packet.payload, &points[0], packet.data_len);
    }
    return packet;
}

TestHighPoint makeHigh(int32_t x, int32_t y, int32_t z, uint8_t reflectivity, uint8_t tag) {
    TestHighPoint point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.reflectivity = reflectivity;
    point.tag = tag;
    return point;
}

// ----------------------------------------------------------------------------
// 解码
// ----------------------------------------------------------------------------

void test_point_size_and_name() {
    CHECK(pure::driver::livoxPointSize(pure::driver::kDataTypeCartesianHigh) == 14u);
    CHECK(pure::driver::livoxPointSize(pure::driver::kDataTypeCartesianLow) == 8u);
    CHECK(pure::driver::livoxPointSize(pure::driver::kDataTypeSpherical) == 10u);
    CHECK(pure::driver::livoxPointSize(pure::driver::kDataTypeDoubleEcho) == 28u);
    CHECK(pure::driver::livoxPointSize(pure::driver::kDataTypeImu) == 24u);
    CHECK(pure::driver::livoxPointSize(0x7Fu) == 0u);

    CHECK(std::string(pure::driver::livoxDataTypeName(pure::driver::kDataTypeCartesianHigh)) == "cartesian_high");
    CHECK(std::string(pure::driver::livoxDataTypeName(0x7Fu)) == "unknown");
}

void test_decode_cartesian_high() {
    std::vector<TestHighPoint> raw;
    raw.push_back(makeHigh(1000, -2000, 3000, 128, 5));

    RawPacket packet = makeHighPacket(0x01020304u, 1, 0, raw, 123456789ull);

    PointCloudFrame frame;
    frame.handle = packet.handle;
    const std::size_t count = pure::driver::decodePointPacket(packet, frame);

    CHECK(count == 1u);
    CHECK(frame.points.size() == 1u);
    CHECK_NEAR(frame.points[0].x, 1.0f, 1e-6);
    CHECK_NEAR(frame.points[0].y, -2.0f, 1e-6);
    CHECK_NEAR(frame.points[0].z, 3.0f, 1e-6);
    CHECK_NEAR(frame.points[0].intensity, 128.0f, 1e-6);
    CHECK(frame.points[0].tag == 5u);
}

void test_decode_cartesian_high_length_guard() {
    // dot_num 谎报为 4，但 data_len 只有 1 个点：必须按 data_len 截断，不能越界
    std::vector<TestHighPoint> raw;
    raw.push_back(makeHigh(1000, 0, 0, 10, 0));
    RawPacket packet = makeHighPacket(1, 0, 0, raw, 0);
    packet.dot_num = 4;

    PointCloudFrame frame;
    const std::size_t count = pure::driver::decodePointPacket(packet, frame);

    CHECK(count == 1u);
    CHECK(frame.points.size() == 1u);
}

void test_decode_cartesian_low() {
    TestLowPoint raw;
    raw.x = 100; // cm
    raw.y = -250;
    raw.z = 50;
    raw.reflectivity = 200;
    raw.tag = 1;

    RawPacket packet;
    packet.data_type = pure::driver::kDataTypeCartesianLow;
    packet.dot_num = 1;
    packet.data_len = sizeof(TestLowPoint);
    std::memcpy(packet.payload, &raw, sizeof(raw));

    PointCloudFrame frame;
    CHECK(pure::driver::decodePointPacket(packet, frame) == 1u);
    CHECK_NEAR(frame.points[0].x, 1.0f, 1e-6);
    CHECK_NEAR(frame.points[0].y, -2.5f, 1e-6);
    CHECK_NEAR(frame.points[0].z, 0.5f, 1e-6);
    CHECK_NEAR(frame.points[0].intensity, 200.0f, 1e-6);
}

void test_decode_spherical() {
    TestSphericalPoint raw;
    raw.depth = 1000; // mm
    raw.theta = 0;    // 俯仰 0°
    raw.phi = 0;      // 方位 0°
    raw.reflectivity = 7;
    raw.tag = 0;

    RawPacket packet;
    packet.data_type = pure::driver::kDataTypeSpherical;
    packet.dot_num = 1;
    packet.data_len = sizeof(TestSphericalPoint);
    std::memcpy(packet.payload, &raw, sizeof(raw));

    PointCloudFrame frame;
    CHECK(pure::driver::decodePointPacket(packet, frame) == 1u);
    CHECK_NEAR(frame.points[0].x, 1.0f, 1e-4);
    CHECK_NEAR(frame.points[0].y, 0.0f, 1e-6);
    CHECK_NEAR(frame.points[0].z, 0.0f, 1e-6);
}

void test_decode_double_echo() {
    TestDoubleEchoPoint raw;
    raw.x1 = 1000;
    raw.y1 = 0;
    raw.z1 = 0;
    raw.reflectivity1 = 11;
    raw.tag1 = 1;
    raw.x2 = 2000;
    raw.y2 = 0;
    raw.z2 = 0;
    raw.reflectivity2 = 22;
    raw.tag2 = 2;

    RawPacket packet;
    packet.data_type = pure::driver::kDataTypeDoubleEcho;
    packet.dot_num = 1;
    packet.data_len = sizeof(TestDoubleEchoPoint);
    std::memcpy(packet.payload, &raw, sizeof(raw));

    PointCloudFrame first;
    CHECK(pure::driver::decodePointPacket(packet, first, false) == 1u);
    CHECK_NEAR(first.points[0].x, 1.0f, 1e-6);
    CHECK_NEAR(first.points[0].intensity, 11.0f, 1e-6);

    PointCloudFrame second;
    CHECK(pure::driver::decodePointPacket(packet, second, true) == 1u);
    CHECK_NEAR(second.points[0].x, 2.0f, 1e-6);
    CHECK_NEAR(second.points[0].intensity, 22.0f, 1e-6);
}

// ----------------------------------------------------------------------------
// 组帧
// ----------------------------------------------------------------------------

void test_frame_assembler_basic() {
    FrameAssembler assembler;
    PointCloudFrame frame;

    std::vector<TestHighPoint> two;
    two.push_back(makeHigh(1000, 0, 0, 1, 0));
    two.push_back(makeHigh(2000, 0, 0, 2, 0));

    CHECK(!assembler.push(makeHighPacket(7, 1, 0, two, 111), frame));
    CHECK(assembler.assembling());
    CHECK(assembler.currentFrameIndex() == 1u);
    CHECK(assembler.currentPointNum() == 2u);

    CHECK(!assembler.push(makeHighPacket(7, 1, 1, two, 222), frame));

    std::vector<TestHighPoint> one;
    one.push_back(makeHigh(3000, 0, 0, 3, 0));

    // 新 frame_cnt 到来 → 交出第 1 帧
    CHECK(assembler.push(makeHighPacket(7, 2, 0, one, 333), frame));
    CHECK(frame.frame_index == 1u);
    CHECK(frame.timestamp_ns == 111ull);
    CHECK(frame.points.size() == 4u);
    CHECK(frame.received_packets == 2u);
    CHECK(frame.expected_packets == 2u);
    CHECK(frame.complete);
    CHECK(frame.handle == 7u);

    // flush 交出第 2 帧
    CHECK(assembler.flush(frame));
    CHECK(frame.frame_index == 2u);
    CHECK(frame.points.size() == 1u);
    CHECK(!assembler.assembling());
    CHECK(!assembler.flush(frame));
}

void test_frame_assembler_detects_loss() {
    FrameAssembler assembler;
    PointCloudFrame frame;

    std::vector<TestHighPoint> two;
    two.push_back(makeHigh(1000, 0, 0, 1, 0));
    two.push_back(makeHigh(2000, 0, 0, 2, 0));

    std::vector<TestHighPoint> one;
    one.push_back(makeHigh(3000, 0, 0, 3, 0));

    // frame 5 收到 udp_cnt 0 与 2 → 说明 udp_cnt 1 丢了
    CHECK(!assembler.push(makeHighPacket(1, 5, 0, two, 1), frame));
    CHECK(!assembler.push(makeHighPacket(1, 5, 2, two, 2), frame));
    CHECK(assembler.push(makeHighPacket(1, 6, 0, one, 3), frame));

    CHECK(frame.frame_index == 5u);
    CHECK(frame.received_packets == 2u);
    CHECK(frame.expected_packets == 3u);
    CHECK(!frame.complete);
    CHECK(frame.points.size() == 4u);

    CHECK(assembler.flush(frame));
    CHECK(frame.frame_index == 6u);
}

// ----------------------------------------------------------------------------
// 队列
// ----------------------------------------------------------------------------

void test_queue_drop_oldest() {
    pure::driver::BlockingQueue<int> queue(2, pure::driver::OverflowPolicy::kDropOldest);

    CHECK(queue.push(1));
    CHECK(queue.push(2));
    CHECK(queue.push(3)); // 挤掉 1
    CHECK(queue.dropped() == 1u);

    int value = 0;
    CHECK(queue.pop(value, 0) && value == 2);
    CHECK(queue.pop(value, 0) && value == 3);
    CHECK(!queue.tryPop(value));
}

void test_queue_drop_newest() {
    pure::driver::BlockingQueue<int> queue(2, pure::driver::OverflowPolicy::kDropNewest);

    CHECK(queue.push(1));
    CHECK(queue.push(2));
    CHECK(!queue.push(3)); // 丢弃新样本
    CHECK(queue.dropped() == 1u);

    int value = 0;
    CHECK(queue.pop(value, 0) && value == 1);
    CHECK(queue.pop(value, 0) && value == 2);
}

void test_queue_close_wakes_waiter() {
    pure::driver::BlockingQueue<int> queue(2);
    queue.close();

    CHECK(queue.closed());
    int value = 0;
    CHECK(!queue.pop(value, 0));
    CHECK(!queue.pop(value, 5));
    CHECK(!queue.push(1)); // 关闭后拒绝入队
}

void test_queue_reset_reopens() {
    pure::driver::BlockingQueue<int> queue(2);
    queue.close();
    queue.reset(4);
    CHECK(!queue.closed());
    CHECK(queue.capacity() == 4u);
    CHECK(queue.push(9));
    int value = 0;
    CHECK(queue.pop(value, 0) && value == 9);
}

void test_queue_rejects_zero_capacity() {
    bool thrown = false;
    try {
        pure::driver::BlockingQueue<int> queue(0);
    } catch (const std::invalid_argument&) {
        thrown = true;
    }
    CHECK(thrown);
}

// ----------------------------------------------------------------------------
// IMU
// ----------------------------------------------------------------------------

void test_decode_imu_and_time() {
    TestImuPoint raw;
    raw.gyro_x = 0.1f;
    raw.gyro_y = -0.2f;
    raw.gyro_z = 0.3f;
    raw.acc_x = 0.0f;
    raw.acc_y = 0.0f;
    raw.acc_z = 1.0f;

    RawPacket packet;
    packet.data_type = pure::driver::kDataTypeImu;
    packet.dot_num = 1;
    packet.data_len = sizeof(TestImuPoint);
    packet.timestamp_ns = 5000000000ull;
    packet.time_interval = 10; // 10 * 0.1us = 1us
    std::memcpy(packet.payload, &raw, sizeof(raw));

    std::vector<ImuSample> samples;
    CHECK(pure::driver::decodeImuPacket(packet, true, true, samples) == 1u);
    CHECK(samples.size() == 1u);
    CHECK_NEAR(samples[0].acc_z, pure::driver::kLivoxGravity, 1e-4);
    CHECK_NEAR(samples[0].gyro_x, 0.1f, 1e-6);
    CHECK_NEAR(samples[0].raw_acc_z, 1.0f, 1e-6);
    CHECK(samples[0].timestamp_ns == 5000000000ull);

    // 关闭 g→m/s^2 换算后应等于原始值
    std::vector<ImuSample> raw_samples;
    CHECK(pure::driver::decodeImuPacket(packet, false, true, raw_samples) == 1u);
    CHECK_NEAR(raw_samples[0].acc_z, 1.0f, 1e-6);
}

void test_decode_imu_stack_buffer_capacity() {
    TestImuPoint raw;
    raw.gyro_x = 0.0f;
    raw.gyro_y = 0.0f;
    raw.gyro_z = 0.0f;
    raw.acc_x = 0.0f;
    raw.acc_y = 0.0f;
    raw.acc_z = 0.0f;

    RawPacket packet;
    packet.data_type = pure::driver::kDataTypeImu;
    packet.dot_num = 3;
    packet.data_len = static_cast<uint16_t>(3 * sizeof(TestImuPoint));
    for (int i = 0; i < 3; ++i) {
        std::memcpy(packet.payload + i * sizeof(TestImuPoint), &raw, sizeof(raw));
    }

    ImuSample buffer[2];
    CHECK(pure::driver::decodeImuPacket(packet, true, true, buffer, 2) == 2u);
    CHECK(pure::driver::decodeImuPacket(packet, true, true, buffer, 0) == 0u);
}

// ----------------------------------------------------------------------------
// 配置
// ----------------------------------------------------------------------------

void test_config_validation() {
    LivoxConfig config;
    std::string error;
    CHECK(!config.valid(&error));
    CHECK(!error.empty());

    config.config_path = "config/livox_mid360.json";
    CHECK(config.valid(&error));

    config.enable_point_cloud = false;
    config.enable_imu = false;
    CHECK(!config.valid(&error));

    config.enable_imu = true;
    config.imu_queue_capacity = 0;
    CHECK(!config.valid(&error));

    config.imu_queue_capacity = 16;
    config.point_queue_capacity = 0;
    config.enable_point_cloud = true;
    CHECK(!config.valid(&error));

    config.point_queue_capacity = 16;
    config.frame_timeout_ms = 0;
    CHECK(!config.valid(&error));

    config.frame_timeout_ms = 100;
    config.sn = "0123456789ABCDEF"; // 16 字符，超过 SDK 16 字节缓冲
    CHECK(!config.valid(&error));
}

void test_model_mapping() {
    CHECK(pure::driver::lidarModelFromSdkType(9) == pure::driver::LidarModel::kMid360);
    CHECK(pure::driver::lidarModelFromSdkType(41) == pure::driver::LidarModel::kMid360l);
    CHECK(pure::driver::lidarModelFromSdkType(250) == pure::driver::LidarModel::kUnknown);
    CHECK(std::string(pure::driver::lidarModelName(pure::driver::LidarModel::kMid360)) == "Mid360");
}

// ----------------------------------------------------------------------------
// 驱动类公共接口（不接硬件、不初始化 SDK）
// ----------------------------------------------------------------------------

void test_driver_public_api_without_hardware() {
    pure::driver::LivoxDriver driver;

    CHECK(!driver.running());
    CHECK(!driver.connected());
    CHECK(driver.handle() == 0u);
    // 未连接时等待必须立即/按时超时返回 false
    CHECK(!driver.waitForLidar(0));
    CHECK(!driver.waitForLidar(10));

    // 没数据时两个 latest* 都应返回 false
    PointCloudFrame frame;
    CHECK(!driver.latestFrame(frame));
    ImuSample sample;
    CHECK(!driver.latestImu(sample));

    const pure::driver::LivoxStats stats = driver.stats();
    CHECK(stats.packets_received == 0u);
    CHECK(stats.packets_dropped == 0u);
    CHECK(stats.frames_published == 0u);
    CHECK(stats.imu_samples_published == 0u);

    // 非法配置必须在触碰 SDK 之前就被拒绝
    LivoxConfig invalid; // config_path 为空
    CHECK(!driver.start(invalid));
    CHECK(!driver.running());

    // stop() 必须幂等
    driver.stop();
    driver.stop();
    CHECK(!driver.running());
}

} // namespace

int main() {
    pure_nav_test::run("point_size_and_name", test_point_size_and_name);
    pure_nav_test::run("decode_cartesian_high", test_decode_cartesian_high);
    pure_nav_test::run("decode_cartesian_high_length_guard", test_decode_cartesian_high_length_guard);
    pure_nav_test::run("decode_cartesian_low", test_decode_cartesian_low);
    pure_nav_test::run("decode_spherical", test_decode_spherical);
    pure_nav_test::run("decode_double_echo", test_decode_double_echo);
    pure_nav_test::run("frame_assembler_basic", test_frame_assembler_basic);
    pure_nav_test::run("frame_assembler_detects_loss", test_frame_assembler_detects_loss);
    pure_nav_test::run("queue_drop_oldest", test_queue_drop_oldest);
    pure_nav_test::run("queue_drop_newest", test_queue_drop_newest);
    pure_nav_test::run("queue_close_wakes_waiter", test_queue_close_wakes_waiter);
    pure_nav_test::run("queue_reset_reopens", test_queue_reset_reopens);
    pure_nav_test::run("queue_rejects_zero_capacity", test_queue_rejects_zero_capacity);
    pure_nav_test::run("decode_imu_and_time", test_decode_imu_and_time);
    pure_nav_test::run("decode_imu_stack_buffer_capacity", test_decode_imu_stack_buffer_capacity);
    pure_nav_test::run("config_validation", test_config_validation);
    pure_nav_test::run("model_mapping", test_model_mapping);
    pure_nav_test::run("driver_public_api_without_hardware", test_driver_public_api_without_hardware);
    return pure_nav_test::summary();
}
