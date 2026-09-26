// ============================================================================
// livox_types.cpp —— 数据解码 / 组帧 / 配置校验
//
// 本文件**只**依赖 "线格式"（UDP 包布局），不依赖 Livox-SDK2 的符号，
// 因此可以脱离 SDK 编译与单元测试。当 SDK 头可用时，用 static_assert 校验
// 复刻的结构体与 SDK 定义逐字节一致，避免 SDK 升级后悄悄错位。
// ============================================================================

#include "livox_driver/livox_types.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

// SDK 头必须放在**全局作用域**引入：它定义的是全局的线格式类型与枚举，
// 放进 namespace 里会造成污染，也会让系统头的包含发生在命名空间内。
#ifdef PURE_NAV_HAS_LIVOX_SDK2
#include "livox_lidar_def.h"
#endif

namespace pure {
namespace driver {
namespace wire {

// SDK 头里这些结构体整体处于 #pragma pack(1)，这里必须一致。
#pragma pack(push, 1)

// 对应 LivoxLidarCartesianHighRawPoint（x/y/z 单位 mm）
struct CartesianHigh {
    int32_t x;
    int32_t y;
    int32_t z;
    uint8_t reflectivity;
    uint8_t tag;
};

// 对应 LivoxLidarCartesianLowRawPoint（x/y/z 单位 cm）
struct CartesianLow {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t reflectivity;
    uint8_t tag;
};

// 对应 LivoxLidarSpherPoint（depth 单位 mm，theta/phi 单位 0.01°）
struct Spherical {
    uint32_t depth;
    uint16_t theta;
    uint16_t phi;
    uint8_t reflectivity;
    uint8_t tag;
};

// 对应 LivoxLidarDoubleEchoRawPoint
struct DoubleEcho {
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

// 对应 LivoxLidarImuRawPoint
struct Imu {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
};

#pragma pack(pop)

} // namespace wire

// ----------------------------------------------------------------------------
// 与 SDK 定义的一致性校验（仅在 SDK 可用时）
// ----------------------------------------------------------------------------
#ifdef PURE_NAV_HAS_LIVOX_SDK2

// 转成 int 比较，避免不同枚举类型之间比较触发 -Wenum-compare
static_assert(static_cast<int>(kDataTypeImu) == static_cast<int>(::kLivoxLidarImuData), "IMU 类型码与 SDK 不一致");
static_assert(static_cast<int>(kDataTypeCartesianHigh) == static_cast<int>(::kLivoxLidarCartesianCoordinateHighData),
              "点类型码与 SDK 不一致");
static_assert(static_cast<int>(kDataTypeCartesianLow) == static_cast<int>(::kLivoxLidarCartesianCoordinateLowData),
              "点类型码与 SDK 不一致");
static_assert(static_cast<int>(kDataTypeSpherical) == static_cast<int>(::kLivoxLidarSphericalCoordinateData),
              "点类型码与 SDK 不一致");
static_assert(static_cast<int>(kDataTypeDoubleEcho) == static_cast<int>(::kLivoxLidarDoubleEchoData),
              "点类型码与 SDK 不一致");

static_assert(sizeof(wire::CartesianHigh) == sizeof(::LivoxLidarCartesianHighRawPoint),
              "CartesianHigh 布局与 SDK 不一致");
static_assert(sizeof(wire::CartesianLow) == sizeof(::LivoxLidarCartesianLowRawPoint),
              "CartesianLow 布局与 SDK 不一致");
static_assert(sizeof(wire::Spherical) == sizeof(::LivoxLidarSpherPoint), "Spherical 布局与 SDK 不一致");
static_assert(sizeof(wire::DoubleEcho) == sizeof(::LivoxLidarDoubleEchoRawPoint),
              "DoubleEcho 布局与 SDK 不一致");
static_assert(sizeof(wire::Imu) == sizeof(::LivoxLidarImuRawPoint), "IMU 布局与 SDK 不一致");
#endif

// ----------------------------------------------------------------------------
// 工具函数
// ----------------------------------------------------------------------------

std::size_t livoxPointSize(uint8_t data_type) {
    switch (data_type) {
        case kDataTypeCartesianHigh:
            return sizeof(wire::CartesianHigh);
        case kDataTypeCartesianLow:
            return sizeof(wire::CartesianLow);
        case kDataTypeSpherical:
            return sizeof(wire::Spherical);
        case kDataTypeDoubleEcho:
            return sizeof(wire::DoubleEcho);
        case kDataTypeImu:
            return sizeof(wire::Imu);
        default:
            return 0;
    }
}

const char* livoxDataTypeName(uint8_t data_type) {
    switch (data_type) {
        case kDataTypeImu:
            return "imu";
        case kDataTypeCartesianHigh:
            return "cartesian_high";
        case kDataTypeCartesianLow:
            return "cartesian_low";
        case kDataTypeSpherical:
            return "spherical";
        case kDataTypeDoubleEcho:
            return "double_echo";
        default:
            return "unknown";
    }
}

uint64_t livoxReadTimestamp(const uint8_t timestamp[8]) {
    // Livox 包内时间戳按小端存放（单位 ns，时基由 time_type / 授时方式决定）
    uint64_t value = 0;
    std::memcpy(&value, timestamp, sizeof(value));
    return value;
}

std::size_t decodePointPacket(const RawPacket& packet, PointCloudFrame& frame, bool use_second_echo) {
    const std::size_t point_size = livoxPointSize(packet.data_type);
    if (point_size == 0 || packet.data_type == kDataTypeImu) {
        return 0;
    }

    // 用 dot_num 与 data_len 双重限幅，任何一方异常都不会越界读 payload
    std::size_t count = packet.dot_num;
    const std::size_t max_by_length = packet.data_len / point_size;
    if (count > max_by_length) {
        count = max_by_length;
    }
    if (count == 0) {
        return 0;
    }

    frame.points.reserve(frame.points.size() + count);
    const uint8_t* payload = packet.payload;

    switch (packet.data_type) {
        case kDataTypeCartesianHigh: {
            const wire::CartesianHigh* raw = reinterpret_cast<const wire::CartesianHigh*>(payload);
            for (std::size_t i = 0; i < count; ++i) {
                frame.points.push_back(PointXYZI(static_cast<float>(raw[i].x) * 0.001f,
                                                 static_cast<float>(raw[i].y) * 0.001f,
                                                 static_cast<float>(raw[i].z) * 0.001f,
                                                 static_cast<float>(raw[i].reflectivity),
                                                 raw[i].tag));
            }
            break;
        }
        case kDataTypeCartesianLow: {
            const wire::CartesianLow* raw = reinterpret_cast<const wire::CartesianLow*>(payload);
            for (std::size_t i = 0; i < count; ++i) {
                frame.points.push_back(PointXYZI(static_cast<float>(raw[i].x) * 0.01f,
                                                 static_cast<float>(raw[i].y) * 0.01f,
                                                 static_cast<float>(raw[i].z) * 0.01f,
                                                 static_cast<float>(raw[i].reflectivity),
                                                 raw[i].tag));
            }
            break;
        }
        case kDataTypeSpherical: {
            // 注意：球坐标约定的机型差异较大，Mid360 请使用 kCartesianHigh。
            // 这里采用 Livox 常见约定：depth(mm)、theta 俯仰角、phi 方位角，单位 0.01°。
            const wire::Spherical* raw = reinterpret_cast<const wire::Spherical*>(payload);
            const double degree_to_rad = kLivoxPi / 18000.0; // 0.01° -> rad
            for (std::size_t i = 0; i < count; ++i) {
                const double depth = static_cast<double>(raw[i].depth) * 0.001;
                const double theta = static_cast<double>(raw[i].theta) * degree_to_rad;
                const double phi = static_cast<double>(raw[i].phi) * degree_to_rad;
                const double cos_theta = std::cos(theta);
                frame.points.push_back(PointXYZI(static_cast<float>(depth * cos_theta * std::cos(phi)),
                                                 static_cast<float>(depth * cos_theta * std::sin(phi)),
                                                 static_cast<float>(depth * std::sin(theta)),
                                                 static_cast<float>(raw[i].reflectivity),
                                                 raw[i].tag));
            }
            break;
        }
        case kDataTypeDoubleEcho: {
            const wire::DoubleEcho* raw = reinterpret_cast<const wire::DoubleEcho*>(payload);
            for (std::size_t i = 0; i < count; ++i) {
                if (use_second_echo) {
                    frame.points.push_back(PointXYZI(static_cast<float>(raw[i].x2) * 0.001f,
                                                     static_cast<float>(raw[i].y2) * 0.001f,
                                                     static_cast<float>(raw[i].z2) * 0.001f,
                                                     static_cast<float>(raw[i].reflectivity2),
                                                     raw[i].tag2));
                } else {
                    frame.points.push_back(PointXYZI(static_cast<float>(raw[i].x1) * 0.001f,
                                                     static_cast<float>(raw[i].y1) * 0.001f,
                                                     static_cast<float>(raw[i].z1) * 0.001f,
                                                     static_cast<float>(raw[i].reflectivity1),
                                                     raw[i].tag1));
                }
            }
            break;
        }
        default:
            return 0;
    }

    return count;
}

std::size_t decodeImuPacket(const RawPacket& packet,
                            bool acc_is_g,
                            bool gyro_is_rad_s,
                            ImuSample* out,
                            std::size_t max_samples) {
    if (packet.data_type != kDataTypeImu || out == nullptr || max_samples == 0) {
        return 0;
    }

    const std::size_t point_size = sizeof(wire::Imu);
    std::size_t count = packet.dot_num;
    const std::size_t max_by_length = packet.data_len / point_size;
    if (count > max_by_length) {
        count = max_by_length;
    }
    if (count > max_samples) {
        count = max_samples;
    }
    if (count == 0) {
        return 0;
    }

    const float acc_scale = acc_is_g ? kLivoxGravity : 1.0f;
    // 0.01deg/s -> rad/s
    const float gyro_scale = gyro_is_rad_s ? 1.0f : static_cast<float>(kLivoxPi / 18000.0);

    const wire::Imu* raw = reinterpret_cast<const wire::Imu*>(packet.payload);
    for (std::size_t i = 0; i < count; ++i) {
        ImuSample& sample = out[i];
        sample.raw_acc_x = raw[i].acc_x;
        sample.raw_acc_y = raw[i].acc_y;
        sample.raw_acc_z = raw[i].acc_z;
        sample.raw_gyro_x = raw[i].gyro_x;
        sample.raw_gyro_y = raw[i].gyro_y;
        sample.raw_gyro_z = raw[i].gyro_z;

        sample.acc_x = raw[i].acc_x * acc_scale;
        sample.acc_y = raw[i].acc_y * acc_scale;
        sample.acc_z = raw[i].acc_z * acc_scale;
        sample.gyro_x = raw[i].gyro_x * gyro_scale;
        sample.gyro_y = raw[i].gyro_y * gyro_scale;
        sample.gyro_z = raw[i].gyro_z * gyro_scale;

        // time_interval 单位 0.1us = 100ns，同包内后续采样按此顺延
        sample.timestamp_ns =
            packet.timestamp_ns + static_cast<uint64_t>(i) * static_cast<uint64_t>(packet.time_interval) * 100ull;
    }
    return count;
}

std::size_t decodeImuPacket(const RawPacket& packet,
                            bool acc_is_g,
                            bool gyro_is_rad_s,
                            std::vector<ImuSample>& out) {
    if (packet.data_type != kDataTypeImu) {
        return 0;
    }
    const std::size_t count = static_cast<std::size_t>(packet.dot_num);
    if (count == 0) {
        return 0;
    }
    out.resize(out.size() + count);
    const std::size_t written = decodeImuPacket(
        packet, acc_is_g, gyro_is_rad_s, &out[out.size() - count], count);
    out.resize(out.size() - count + written);
    return written;
}

// ----------------------------------------------------------------------------
// FrameAssembler
// ----------------------------------------------------------------------------

FrameAssembler::FrameAssembler() : has_frame_(false), use_second_echo_(false), max_udp_cnt_(0) {}

void FrameAssembler::reset() {
    current_.clear();
    has_frame_ = false;
    max_udp_cnt_ = 0;
}

void FrameAssembler::startFrame(const RawPacket& packet) {
    current_.clear();
    current_.handle = packet.handle;
    current_.frame_index = packet.frame_cnt;
    current_.timestamp_ns = packet.timestamp_ns;
    current_.dev_type = packet.dev_type;
    current_.data_type = packet.data_type;
    current_.received_packets = 0;
    current_.expected_packets = 0;
    current_.complete = true;
    max_udp_cnt_ = 0;
    has_frame_ = true;
}

void FrameAssembler::finalize(PointCloudFrame& out) {
    out.clear();
    out.points.swap(current_.points);
    out.handle = current_.handle;
    out.frame_index = current_.frame_index;
    out.timestamp_ns = current_.timestamp_ns;
    out.dev_type = current_.dev_type;
    out.data_type = current_.data_type;
    out.received_packets = current_.received_packets;
    out.expected_packets = current_.expected_packets;
    out.complete = current_.complete;
    has_frame_ = false;
    max_udp_cnt_ = 0;
}

bool FrameAssembler::push(const RawPacket& packet, PointCloudFrame& out) {
    bool emitted = false;

    // frame_cnt 变化 = 上一帧结束
    if (has_frame_ && packet.frame_cnt != current_.frame_index) {
        finalize(out);
        emitted = true;
    }
    if (!has_frame_) {
        startFrame(packet);
    }

    decodePointPacket(packet, current_, use_second_echo_);
    ++current_.received_packets;
    const uint32_t expected = static_cast<uint32_t>(packet.udp_cnt) + 1u;
    if (expected > max_udp_cnt_) {
        max_udp_cnt_ = expected;
    }
    current_.expected_packets = max_udp_cnt_;
    current_.complete = (current_.received_packets >= max_udp_cnt_);

    return emitted;
}

bool FrameAssembler::flush(PointCloudFrame& out) {
    if (!has_frame_) {
        return false;
    }
    finalize(out);
    return true;
}

// ----------------------------------------------------------------------------
// 配置校验
// ----------------------------------------------------------------------------

namespace {

bool configError(std::string* error, const char* message) {
    if (error != nullptr) {
        *error = message;
    }
    return false;
}

} // namespace

bool LivoxConfig::valid(std::string* error) const {
    if (config_path.empty()) {
        return configError(error, "config_path 为空：需要 Livox SDK 的 json 配置文件路径");
    }
    if (!enable_point_cloud && !enable_imu) {
        return configError(error, "enable_point_cloud 与 enable_imu 不能同时为 false");
    }
    if (enable_point_cloud && point_queue_capacity == 0) {
        return configError(error, "point_queue_capacity 必须大于 0");
    }
    if (enable_imu && imu_queue_capacity == 0) {
        return configError(error, "imu_queue_capacity 必须大于 0");
    }
    if (enable_point_cloud && frame_timeout_ms <= 0) {
        return configError(error, "frame_timeout_ms 必须大于 0");
    }
    if (sn.size() > 15) {
        return configError(error, "sn 长度超过 SDK 的 16 字节缓冲（最多 15 个字符）");
    }
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

// ----------------------------------------------------------------------------
// 型号映射
// ----------------------------------------------------------------------------

LidarModel lidarModelFromSdkType(uint8_t dev_type) {
    // 数值取自 SDK 的 LivoxLidarDeviceType
    switch (dev_type) {
        case 1:
            return LidarModel::kMid40;
        case 2:
            return LidarModel::kTele;
        case 3:
            return LidarModel::kHorizon;
        case 6:
            return LidarModel::kMid70;
        case 7:
            return LidarModel::kAvia;
        case 9:
            return LidarModel::kMid360;
        case 10:
            return LidarModel::kIndustrialHap;
        case 15:
            return LidarModel::kHap;
        case 35:
            return LidarModel::kMid360s;
        case 40:
            return LidarModel::kAvia2;
        case 41:
            return LidarModel::kMid360l;
        default:
            return LidarModel::kUnknown;
    }
}

const char* lidarModelName(LidarModel model) {
    switch (model) {
        case LidarModel::kMid40:
            return "Mid40";
        case LidarModel::kTele:
            return "Tele";
        case LidarModel::kHorizon:
            return "Horizon";
        case LidarModel::kMid70:
            return "Mid70";
        case LidarModel::kAvia:
            return "Avia";
        case LidarModel::kMid360:
            return "Mid360";
        case LidarModel::kIndustrialHap:
            return "IndustrialHAP";
        case LidarModel::kHap:
            return "HAP";
        case LidarModel::kMid360s:
            return "Mid360s";
        case LidarModel::kAvia2:
            return "Avia2";
        case LidarModel::kMid360l:
            return "Mid360l";
        default:
            return "Unknown";
    }
}

} // namespace driver
} // namespace pure
