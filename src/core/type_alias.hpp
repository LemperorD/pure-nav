#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>

namespace pure::core {
  using Mat3f = Eigen::Matrix3f;           ///< 三阶浮点矩阵。
  using Vec3f = Eigen::Vector3f;           ///< 三维浮点向量。
  using Vec3d = Eigen::Vector3d;           ///< 三维双精度向量。
  using Quaterniond = Eigen::Quaterniond;  ///< 姿态四元数，使用前须归一化。
  using Timestamp = std::int64_t;          ///< 本机单调时钟时间，单位 ns。

  /// @brief 数据坐标系；变换由应用层提供，禁止仅改标识冒充变换。
  enum class Frame : std::uint8_t {
    kUnknown,
    kMap,
    kOdom,
    kBase,
    kLidar,
    kCamera,
    kGimbal
  };
  /// @brief 一次观测的身份；时间 0 合法，业务字段显式表达有效性。
  struct DataHeader {
    Timestamp stamp_ns{0};         ///< 采样时刻，不是处理完成时刻。
    std::uint64_t sequence{0};     ///< 会话内递增序号。
    std::uint64_t session{0};      ///< 来源重启后更换的会话号。
    Frame frame{Frame::kUnknown};  ///< 数据坐标系。
  };
  /// @brief 三维位姿；orientation 将局部坐标旋转到消息头参考系。
  struct Pose3D {
    Vec3d position{Vec3d::Zero()};                     ///< 位置，m。
    Quaterniond orientation{Quaterniond::Identity()};  ///< 单位四元数。
  };
  /// @brief 数据健康状态；有效期限由调度方按单调时钟检查。
  enum class DataStatus : std::uint8_t {
    kUnavailable,
    kValid,
    kStale,
    kInvalid
  };
}  // namespace pure::core
