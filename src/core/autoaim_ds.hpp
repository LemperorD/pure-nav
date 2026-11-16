#pragma once

#include <vector>

#include "type_alias.hpp"

namespace pure::core {
  /// @brief 相机像素布局；应用层负责厂商缓冲转换。
  enum class PixelFormat : std::uint8_t { kUnknown, kMono8, kBgr8, kRgb8 };
  /// @brief 拥有像素数据的图像；不借用已经归还的相机缓冲。
  struct Image {
    DataHeader header;  ///< 曝光中点时刻，坐标系为相机。
    std::uint32_t width{0}, height{0}, stride_bytes{0};  ///< 图像尺寸及行跨度。
    PixelFormat format{PixelFormat::kUnknown};           ///< 像素格式。
    std::vector<std::uint8_t> pixels;  ///< 至少 stride_bytes * height 字节。
  };
  /// @brief 图像中的矩形区域，以左上角为原点。
  struct ImageRect {
    double x{0}, y{0}, width{0}, height{0};  ///< 像素单位。
  };
  /// @brief 图像关键点，用于目标几何与位姿估计。
  struct ImagePoint {
    double x{0}, y{0};  ///< 像素坐标，顺序由检测模型契约定义。
  };
  /// @brief 一次比赛目标检测，尚不代表已经获得三维位置。
  struct DetectResult {
    ImageRect bounding_box;  ///< 图像框。
    double confidence{0};    ///< [0,1]。
    int class_id{-1};        ///< 模型类别，-1 表示未知。
    std::vector<ImagePoint>
        keypoints;  ///< 可选几何关键点，缺失时估计器应降级。
  };
  using DetectResults = std::vector<DetectResult>;  ///< 单帧检测集合。
  /// @brief 带来源与状态的单帧检测，空结果可表示有效的未检出。
  struct DetectionFrame {
    DataHeader header;  ///< 原图时刻、序号和相机坐标系。
    DataStatus status{DataStatus::kUnavailable};  ///< 检测是否成功执行。
    DetectResults detections;                     ///< 检测集合。
  };
  /// @brief 云台反馈，角度方向为右手系，零位与外参由标定定义。
  struct GimbalState {
    DataHeader header;                            ///< 反馈时刻，参考系为车体。
    double pitch{0}, yaw{0}, roll{0};             ///< rad。
    double pitch_rate_rps{0}, yaw_rate_rps{0};    ///< rad/s。
    DataStatus status{DataStatus::kUnavailable};  ///< 是否有效。
  };
  /// @brief 已标定的相机参数，矩阵和畸变模型由估计器解释。
  struct CameraModel {
    bool calibrated{false};  ///< 是否完成本相机标定。
    Eigen::Matrix3d intrinsics{Eigen::Matrix3d::Identity()};  ///< 像素内参。
    std::vector<double> distortion;  ///< 模型约定的畸变系数。
    Pose3D camera_in_gimbal;         ///< 相机到云台的外参。
  };
  /// @brief 目标跟踪状态。
  enum class TrackingState : std::uint8_t {
    kLost,
    kAcquiring,
    kTracking,
    kPredicting
  };
  /// @brief 同步后的观察者状态，防止历史图像配上最新位姿。
  struct ObserverState {
    DataHeader header;   ///< 参考系与曝光时刻。
    Pose3D base_pose;    ///< 车体在参考系中的姿态，含坡面俯仰横滚。
    GimbalState gimbal;  ///< 插值到曝光时刻的云台反馈。
    DataStatus status{DataStatus::kUnavailable};  ///< 同步是否有效。
  };
  /// @brief 三维目标状态，用于比赛目标观测和云台跟随。
  struct TargetState {
    DataHeader header;                             ///< 估计时间和位置参考系。
    std::uint64_t track_id{0};                     ///< 会话内跟踪标识。
    Pose3D pose;                                   ///< 目标位姿。
    Vec3d velocity{Vec3d::Zero()};                 ///< 同一参考系速度，m/s。
    TrackingState tracking{TrackingState::kLost};  ///< 跟踪状态。
    double confidence{0};                          ///< [0,1]。
    Timestamp valid_until_ns{0};                   ///< 预测有效期限。
  };
  /// @brief 单帧自瞄输入快照，标定与同步由应用层提供。
  struct AutoAimInput {
    Image image;             ///< 当前图像。
    CameraModel camera;      ///< 对应相机标定。
    ObserverState observer;  ///< 曝光时刻的姿态和云台反馈。
  };
  /// @brief 云台跟随解算结果，不代表驱动已经执行。
  struct AimSolution {
    DataHeader header;                ///< 生成时刻，相对车体的云台角度。
    std::uint64_t track_id{0};        ///< 对应目标。
    double yaw_rad{0}, pitch_rad{0};  ///< 标定零位下的目标角度。
    Timestamp valid_until_ns{0};      ///< 解算有效期限。
    DataStatus status{DataStatus::kUnavailable};  ///< 有效性。
  };
}  // namespace pure::core
