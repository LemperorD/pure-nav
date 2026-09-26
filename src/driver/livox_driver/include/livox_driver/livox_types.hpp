// ============================================================================
// livox_types.hpp —— Livox 雷达数据的公共类型
//
// 设计原则：
//   1. **不依赖 Livox-SDK2 头文件**。SDK 的 LivoxLidarCartesianHighRawPoint 等
//      结构体本质是 UDP 线格式(wire format)，本文件在 .cpp 里用等价布局复刻，
//      并在 SDK 可用时用 static_assert 校验布局一致；因此下游模块 include 本
//      文件不会把第三方 SDK 传染进整个编译单元。
//   2. 对外单位统一：坐标 m、强度 0~255、IMU 为 rad/s 与 m/s^2（SI）。
//      SDK 原始值另外保留在 ImuSample::raw_* 中，便于排查与二次标定。
//   3. 本文件只放"数据 + 无状态工具函数"，与线程/SDK 生命周期无关，
//      因此可以脱离硬件单独做单元测试（见 src/test/test_driver）。
// ============================================================================

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace pure {
namespace driver {

// ----------------------------------------------------------------------------
// 常量
// ----------------------------------------------------------------------------

// 单包最大有效载荷。Livox 单包 < 1500B（以太网 MTU），2048 足够覆盖全部机型。
constexpr std::size_t kLivoxMaxUdpPayload = 2048;

constexpr float kLivoxGravity = 9.80665f;
constexpr double kLivoxPi = 3.14159265358979323846;

// 数据包类型。数值刻意与 SDK 的 LivoxLidarPointDataType 保持一致，
// 这样 .cpp 里做映射时一眼能对上（见 livox_types.cpp 的 static_assert）。
enum LidarDataType : uint8_t {
    kDataTypeImu = 0x00,
    kDataTypeCartesianHigh = 0x01,
    kDataTypeCartesianLow = 0x02,
    kDataTypeSpherical = 0x03,
    kDataTypeDoubleEcho = 0x11,
};

// 雷达型号（只用于日志与配置校验，不参与取流逻辑）
enum class LidarModel {
    kUnknown = 0,
    kMid40,
    kTele,
    kHorizon,
    kMid70,
    kAvia,
    kMid360,
    kIndustrialHap,
    kHap,
    kAvia2,
    kMid360s,
    kMid360l,
};

// 期望 SDK 下发的点云格式。Mid360 默认用 kCartesianHigh（mm 整数，14B/点）。
enum class PointDataType {
    kCartesianHigh,
    kCartesianLow,
    kSpherical,
    kDoubleEcho,
};

// ----------------------------------------------------------------------------
// 点 / 帧 / IMU
// ----------------------------------------------------------------------------

// 单点。x/y/z 单位 m，intensity 0~255，tag 为雷达原始标签位（原样透传）。
struct PointXYZI {
    float x;
    float y;
    float z;
    float intensity;
    uint8_t tag;

    PointXYZI() : x(0.0f), y(0.0f), z(0.0f), intensity(0.0f), tag(0) {}
    PointXYZI(float px, float py, float pz, float pi, uint8_t pt)
        : x(px), y(py), z(pz), intensity(pi), tag(pt) {}
};

// 一帧点云。由 FrameAssembler 按 SDK 的 frame_cnt 组装。
struct PointCloudFrame {
    uint32_t handle;           // 雷达句柄（SDK 里等于雷达 IP 的整型表示）
    uint32_t frame_index;      // SDK frame_cnt（0~255 循环）
    uint64_t timestamp_ns;     // 帧首包时间戳，ns（时基由 time_type/授时方式决定）
    uint32_t received_packets; // 本帧实际收到的包数
    uint32_t expected_packets; // 依据包内 udp_cnt 推断的应有包数（0 = 未知）
    bool complete;             // received >= expected 时为 true（无丢包）
    uint8_t dev_type;          // SDK 设备类型
    uint8_t data_type;         // SDK 数据类型
    std::vector<PointXYZI> points;

    PointCloudFrame() { clear(); }

    void clear() {
        handle = 0;
        frame_index = 0;
        timestamp_ns = 0;
        received_packets = 0;
        expected_packets = 0;
        complete = true;
        dev_type = 0;
        data_type = 0;
        points.clear();
    }

    bool empty() const { return points.empty(); }
    std::size_t size() const { return points.size(); }
};

// 单个 IMU 采样。
//   * acc  —— m/s^2（由 SDK 原始 g 值乘以 kLivoxGravity 得到）
//   * gyro —— rad/s
//   * raw_*—— SDK 直接给出的原始浮点数（Mid360 为 g 与 rad/s），保留用于排查
struct ImuSample {
    uint64_t timestamp_ns;

    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;

    float raw_acc_x;
    float raw_acc_y;
    float raw_acc_z;
    float raw_gyro_x;
    float raw_gyro_y;
    float raw_gyro_z;

    ImuSample()
        : timestamp_ns(0),
          acc_x(0.0f), acc_y(0.0f), acc_z(0.0f),
          gyro_x(0.0f), gyro_y(0.0f), gyro_z(0.0f),
          raw_acc_x(0.0f), raw_acc_y(0.0f), raw_acc_z(0.0f),
          raw_gyro_x(0.0f), raw_gyro_y(0.0f), raw_gyro_z(0.0f) {}
};

// ----------------------------------------------------------------------------
// 原始 UDP 包（SDK 回调 → 工作线程之间的传递单元）
//
// SDK 回调里的 LivoxLidarEthernetPacket* 只在回调期间有效，必须拷贝。
// 这里用固定大小数组而不是 std::vector，是为了让 BlockingQueue 可以预分配
// 环形缓冲，回调路径上不再发生堆分配。
// ----------------------------------------------------------------------------
struct RawPacket {
    uint32_t handle;
    uint64_t timestamp_ns;
    uint16_t time_interval; // SDK 单位 0.1us
    uint16_t dot_num;       // 本包点数（IMU 包为采样数）
    uint16_t udp_cnt;       // 帧内包序号
    uint16_t data_len;      // payload 有效字节数
    uint8_t dev_type;
    uint8_t data_type;
    uint8_t frame_cnt;
    uint8_t time_type;
    uint8_t payload[kLivoxMaxUdpPayload];

    RawPacket()
        : handle(0),
          timestamp_ns(0),
          time_interval(0),
          dot_num(0),
          udp_cnt(0),
          data_len(0),
          dev_type(0),
          data_type(0),
          frame_cnt(0),
          time_type(0) {}
};

// ----------------------------------------------------------------------------
// 无状态工具函数
// ----------------------------------------------------------------------------

// 单点字节数；未知类型返回 0
std::size_t livoxPointSize(uint8_t data_type);

// 日志用的类型名
const char* livoxDataTypeName(uint8_t data_type);

// 从 SDK 包头里的 8 字节字段读取时间戳（小端）
uint64_t livoxReadTimestamp(const uint8_t timestamp[8]);

// 把一包原始点云解码并**追加**到 frame.points，返回本包追加的点数。
// use_second_echo 仅在 kDoubleEcho 时生效。
std::size_t decodePointPacket(const RawPacket& packet, PointCloudFrame& frame, bool use_second_echo = false);

// 把一包原始 IMU 数据解码到调用方提供的缓冲（**不分配内存**，供 SDK 回调线程使用）。
//   acc_is_g        —— SDK 原始加速度是否为 g（是则乘以 kLivoxGravity）
//   gyro_is_rad_s   —— SDK 原始角速度是否为 rad/s（否则按 0.01deg/s 处理）
// 返回写入的采样数，最多 max_samples 个。
std::size_t decodeImuPacket(const RawPacket& packet,
                            bool acc_is_g,
                            bool gyro_is_rad_s,
                            ImuSample* out,
                            std::size_t max_samples);

// 同上，但追加到 vector（便于测试/离线回放）
std::size_t decodeImuPacket(const RawPacket& packet,
                            bool acc_is_g,
                            bool gyro_is_rad_s,
                            std::vector<ImuSample>& out);

// ----------------------------------------------------------------------------
// FrameAssembler —— 把连续 UDP 包组装成帧（纯逻辑，无锁、无线程）
//
// 组装规则：
//   * 以 SDK 的 frame_cnt 变化作为帧边界：收到属于新 frame_cnt 的包时，
//     把上一帧交出（push 返回 true 并填充 out），再以当前包开启新帧；
//   * expected_packets 由帧内观察到的最大 udp_cnt + 1 推断，
//     received >= expected 视为无丢包；
//   * 流中断（雷达停转/拔网线）时由调用方用 flush() 冲刷未完成帧。
// ----------------------------------------------------------------------------
class FrameAssembler {
public:
    FrameAssembler();

    void reset();

    void setUseSecondEcho(bool enabled) { use_second_echo_ = enabled; }
    bool useSecondEcho() const { return use_second_echo_; }

    bool assembling() const { return has_frame_; }
    uint32_t currentFrameIndex() const { return current_.frame_index; }
    std::size_t currentPointNum() const { return current_.points.size(); }

    // 喂入一包。若该包导致上一帧结束，返回 true 并把整帧写入 out。
    bool push(const RawPacket& packet, PointCloudFrame& out);

    // 冲刷当前未完成帧；没有在组装的帧时返回 false。
    bool flush(PointCloudFrame& out);

private:
    void startFrame(const RawPacket& packet);
    void finalize(PointCloudFrame& out);

    PointCloudFrame current_;
    bool has_frame_;
    bool use_second_echo_;
    uint32_t max_udp_cnt_;
};

// ----------------------------------------------------------------------------
// 配置 / 统计
// ----------------------------------------------------------------------------

struct LivoxConfig {
    // --- 必填 ---
    std::string config_path; // Livox SDK 的 json 配置文件路径（见 config/livox_mid360.json）

    // --- 目标雷达选择：都为空/0 时绑定第一个发现的雷达 ---
    std::string sn;    // 目标 SN，例如 "47MDL3C0020913"
    uint32_t handle = 0; // 目标 handle（雷达 IP 的整型值）
    LidarModel model = LidarModel::kUnknown; // 仅日志/校验

    // --- 数据格式 ---
    PointDataType point_data_type = PointDataType::kCartesianHigh;
    bool use_second_echo = false; // kDoubleEcho 时取第二回波

    // --- 流开关 ---
    bool enable_point_cloud = true;
    bool enable_imu = true;

    // --- 发现雷达后是否自动下发工作模式/数据格式/IMU 使能 ---
    bool configure_on_connect = true;

    // --- IMU 原始单位（决定如何换算到 SI）---
    bool imu_acc_is_g = true;        // Mid360/HAP 为 true
    bool imu_gyro_is_rad_s = true;   // Mid360/HAP 为 true

    // --- 取流缓冲 ---
    std::size_t point_queue_capacity = 512; // 约 0.5s（1000 包/s 量级）
    std::size_t imu_queue_capacity = 1024;  // 约 5s（200Hz）

    // --- 点云线程在多久没有新包时冲刷半帧（ms）---
    int frame_timeout_ms = 200;

    // --- 是否打印 info 级日志 ---
    bool verbose = false;

    // 检查配置自洽性；不合法时返回 false 并写入 error
    bool valid(std::string* error = nullptr) const;
};

struct LivoxStats {
    uint64_t packets_received;      // SDK 回调收到的点云包
    uint64_t packets_dropped;       // 因队列满被丢弃的点云包
    uint64_t frames_published;      // 发布出去的帧
    uint64_t frames_incomplete;     // 其中有丢包的帧
    uint64_t points_published;      // 累计发布点数
    uint64_t imu_packets_received;  // SDK 回调收到的 IMU 包
    uint64_t imu_samples_published; // 发布出去的 IMU 采样
    uint64_t imu_dropped;           // 因队列满被丢弃的 IMU 采样

    LivoxStats()
        : packets_received(0),
          packets_dropped(0),
          frames_published(0),
          frames_incomplete(0),
          points_published(0),
          imu_packets_received(0),
          imu_samples_published(0),
          imu_dropped(0) {}
};

// 设备类型（SDK 数值）→ 型号
LidarModel lidarModelFromSdkType(uint8_t dev_type);

// 型号 → 便于阅读的名字
const char* lidarModelName(LidarModel model);

} // namespace driver
} // namespace pure
