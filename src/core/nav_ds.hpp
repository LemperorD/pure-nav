#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "type_alias.hpp"

namespace pure::core {
  /// @brief 平面位置，用于区域边界与网格原点。
  struct Point2D {
    double x{0};  ///< m。
    double y{0};  ///< m。
  };
  /// @brief 带逐点时间的雷达点，坐标系来自点云消息头。
  struct LidarPoint {
    Vec3f position{Vec3f::Zero()};  ///< m。
    float intensity{0};             ///< 反射强度。
    std::uint32_t offset_ns{0};     ///< 相对扫描开始的时间。
  };
  /// @brief 一次扫描；不完整帧由算法明确决定是否接收。
  struct PointCloudMap {
    DataHeader header;               ///< 扫描开始时间和点坐标系。
    Timestamp end_ns{0};             ///< 扫描结束时间。
    bool complete{false};            ///< 是否确认扫描完整。
    std::vector<LidarPoint> points;  ///< 本进程拥有的点数组。
  };
  /// @brief 惯性测量，序列按时间严格递增。
  struct ImuSample {
    DataHeader header;                      ///< 测量时间和传感器坐标系。
    Vec3d acceleration{Vec3d::Zero()};      ///< 比力，m/s²，包含重力影响。
    Vec3d angular_velocity{Vec3d::Zero()};  ///< rad/s。
  };
  /// @brief 定位输入；惯性历史应覆盖扫描区间和前次积分边界。
  struct LocalizationInput {
    PointCloudMap scan;          ///< 原始扫描及逐点时间。
    std::vector<ImuSample> imu;  ///< 同步后的惯性测量序列。
  };
  /// @brief 定位健康状态，失定位时不能继续使用旧位姿行驶。
  enum class LocalizationState : std::uint8_t {
    kInitializing,
    kTracking,
    kDegraded,
    kLost
  };
  /// @brief 三维状态估计，保留坡面上的俯仰与横滚。
  struct NavigationState {
    DataHeader header;  ///< 位姿参考系和估计时刻。
    std::uint64_t scan_sequence{0}, scan_session{0};  ///< 关联输入扫描身份。
    Pose3D pose;                            ///< 车体在参考系中的位姿。
    Vec3d linear_velocity{Vec3d::Zero()};   ///< 参考系速度，m/s。
    Vec3d angular_velocity{Vec3d::Zero()};  ///< 车体系角速度，rad/s。
    LocalizationState state{LocalizationState::kInitializing};  ///< 定位状态。
    Timestamp valid_until_ns{0};  ///< 位姿有效期限。
  };
  /// @brief 地形语义；洞口是通道，坑洞是地面支撑缺失。
  enum class TerrainType : std::uint8_t {
    kUnknown,
    kFlat,
    kSlope,
    kStep,
    kDropOff,
    kHole,
    kPassage,
    kOverhang,
    kObstacle
  };
  /// @brief 支撑证据；缺少回波只表示未知，不能直接判定坑洞或平地。
  enum class GroundSupport : std::uint8_t { kUnknown, kSupported, kMissing };
  /// @brief 几何证据到通行决策的结果，默认未知且不放行。
  enum class Traversability : std::uint8_t {
    kUnknown,
    kBlocked,
    kRestricted,
    kTraversable
  };
  /// @brief 一个地表层的几何与语义证据；未测量数值为 NaN。
  struct TerrainCell {
    TerrainType type{TerrainType::kUnknown};             ///< 地形类别。
    GroundSupport support{GroundSupport::kUnknown};      ///< 地面支撑证据。
    Traversability traversal{Traversability::kUnknown};  ///< 本车通行结论。
    double ground_z{
        std::numeric_limits<double>::quiet_NaN()};  ///< 地面高程，m。
    Vec3d normal{Vec3d::Zero()};  ///< 向上单位法向，零表示未知。
    double slope_rad{std::numeric_limits<double>::quiet_NaN()};  ///< 最大坡角。
    double roughness_m{
        std::numeric_limits<double>::quiet_NaN()};  ///< 地面拟合残差。
    double step_up_m{
        std::numeric_limits<double>::quiet_NaN()};  ///< 最大上台阶。
    double drop_down_m{
        std::numeric_limits<double>::quiet_NaN()};  ///< 最大下降落差。
    double clearance_m{
        std::numeric_limits<double>::quiet_NaN()};  ///< 到顶棚净空。
    double confidence{0};  ///< 证据置信度，[0,1]，不等于可通行概率。
    double traversal_cost{
        std::numeric_limits<double>::infinity()};  ///< 非负代价。
    Timestamp observed_ns{0};                      ///< 本格最新观测时刻。
  };
  /// @brief 带高程的平面地形网格，同一平面位置只表达一层地表。
  struct GridMap2D {
    DataHeader header;               ///< 网格参考系与生成时刻。
    std::size_t width{0};            ///< 列数。
    std::size_t height{0};           ///< 行数。
    double resolution_m{0};          ///< 每格边长，m。
    Point2D origin;                  ///< 左下角，网格轴与参考系平行。
    Timestamp valid_until_ns{0};     ///< 整图期限，仍需检查格子观测年龄。
    std::vector<TerrainCell> cells;  ///< 行主序，y * width + x。
    /// @brief 检查尺寸、分辨率、原点和存储长度是否一致。
    bool valid() const noexcept {
      return width > 0 && height > 0 && std::isfinite(resolution_m)
          && resolution_m > 0 && std::isfinite(origin.x)
          && std::isfinite(origin.y)
          && width <= std::numeric_limits<std::size_t>::max() / height
          && cells.size() == width * height;
    }
    /// @brief 按格坐标读取地形；非法布局或越界抛 out_of_range。
    /// @param x 列号。
    /// @param y 行号。
    const TerrainCell& at(std::size_t x, std::size_t y) const {
      if (!valid() || x >= width || y >= height) {
        throw std::out_of_range("invalid terrain grid or cell index");
      }
      return cells[y * width + x];
    }
  };
  /// @brief 三维体素占据证据，未知与空闲分开。
  enum class Occupancy : std::uint8_t { kUnknown, kFree, kOccupied };
  /// @brief 三维占据网格，用于多层结构、顶棚和通道体积查询。
  struct GridMap3D {
    DataHeader header;                          ///< 网格坐标系和生成时间。
    std::size_t width{0}, height{0}, depth{0};  ///< 三轴格数。
    double resolution_m{0};                     ///< 体素边长，m。
    Vec3d origin{Vec3d::Zero()};                ///< 最小角点，轴与参考系平行。
    std::vector<Occupancy> cells;  ///< 下标 (z * height + y) * width + x。
  };
  /// @brief 车体几何与实测通行能力，默认未配置，不能直接规划。
  struct RobotCapability {
    bool configured{false};                       ///< 尺寸和能力已按本车标定。
    double length_m{0}, width_m{0}, height_m{0};  ///< 完整碰撞包络尺寸。
    double safety_margin_m{0};                    ///< 障碍物与洞口边界余量。
    double max_uphill_rad{0}, max_downhill_rad{0};  ///< 纵向上下坡限制。
    double max_cross_slope_rad{0};                  ///< 横坡抗侧翻限制。
    double max_step_up_m{0}, max_drop_down_m{0};    ///< 台阶与下降限制。
    double max_gap_m{0};  ///< 实测可越过的地面间隙，0 表示禁止跨越。
    double max_speed_mps{0}, max_yaw_rate_rps{0};  ///< 速度上限。
  };
  /// @brief 可穿行洞口或窄通道，还需地面支撑和扫掠体碰撞检查。
  struct Passage {
    std::uint64_t id{0};  ///< 当前地图内标识。
    Pose3D entrance;      ///< 洞口中心，局部 x 穿行、y 宽度、z 高度。
    Pose3D exit;          ///< 出口中心，与入口同一地图参考系。
    double width_m{0}, height_m{0}, length_m{0};  ///< 最小净宽、净高及长度。
    Traversability traversal{Traversability::kUnknown};  ///< 通行结论。
    GroundSupport support{GroundSupport::kUnknown};      ///< 通道内支撑证据。
    Timestamp observed_ns{0};                            ///< 最近一次确认时刻。
  };
  /// @brief 地形处理结果，区分地面坑洞与可穿行洞口。
  struct TerrainMap {
    GridMap2D surface;  ///< 地面、坡面、台阶、坑洞和落差。
    std::shared_ptr<const GridMap3D>
        volume;                     ///< 可选的只读三维快照，空表示无证据。
    std::vector<Passage> passages;  ///< 通道及洞口几何。
    DataStatus status{DataStatus::kUnavailable};  ///< 数据健康状态。
  };
  /// @brief 哨兵自主导航任务类别。
  enum class NavigationTask : std::uint8_t {
    kHold,
    kGoTo,
    kPatrol,
    kReturnToSupply,
    kTraversePassage
  };
  /// @brief 上层任务，目标与巡逻点位于消息头参考系。
  struct NavigationGoal {
    DataHeader header;                           ///< 任务生成时间和坐标系。
    std::uint64_t task_id{0};                    ///< 防止旧路径用于新任务。
    NavigationTask task{NavigationTask::kHold};  ///< 默认原地保持。
    Pose3D target;                               ///< 目标位姿。
    std::vector<Pose3D> patrol_points;  ///< 有序巡逻点，重复策略由决策决定。
    std::uint64_t passage_id{0};        ///< 穿行任务的洞口标识。
    double position_tolerance_m{0.1};   ///< 位置容差。
    double heading_tolerance_rad{0.1};  ///< 航向容差。
    Timestamp valid_until_ns{0};        ///< 任务有效期限。
  };
  /// @brief 带坡面姿态、速度和相对时间的轨迹点。
  struct TrajectoryPoint {
    Pose3D pose;                           ///< 期望车体位姿。
    Vec3d linear_velocity{Vec3d::Zero()};  ///< 路径参考系速度，m/s。
    double yaw_rate_rps{0};                ///< 绕局部竖轴角速度。
    double time_from_start_s{0};           ///< 相对轨迹头时刻，非负递增秒数。
    TerrainType terrain{TerrainType::kUnknown};  ///< 对应地形。
  };
  /// @brief 规划状态，空路径不能表示规划成功。
  enum class PlanStatus : std::uint8_t {
    kUnavailable,
    kReady,
    kArrived,
    kUnreachable,
    kUnsafe
  };
  /// @brief 全局或局部轨迹，关联任务与地形地图版本。
  struct Trajectory {
    DataHeader header;                              ///< 路径参考系和起始时间。
    std::uint64_t task_id{0}, terrain_sequence{0};  ///< 输入任务与地图序号。
    std::uint64_t terrain_session{0};               ///< 地图重启后旧序号失效。
    Timestamp valid_until_ns{0};                    ///< 路径有效期限。
    PlanStatus status{PlanStatus::kUnavailable};    ///< 规划结论。
    std::vector<TrajectoryPoint> waypoints;         ///< 有序轨迹点。
  };
  /// @brief 地形输入，扫描已去畸变并变换到位姿参考系。
  struct TerrainInput {
    PointCloudMap registered_scan;  ///< 与状态对应同一扫描。
    Pose3D sensor_pose;  ///< 雷达在参考系中的位姿，提供射线原点和可见性依据。
    NavigationState state;  ///< 对应扫描时刻的车体姿态。
    RobotCapability robot;  ///< 通行约束。
  };
  /// @brief 规划快照，调用方保证时间、坐标系、会话一致。
  struct PlanningInput {
    NavigationState state;  ///< 当前状态。
    NavigationGoal goal;    ///< 有效任务。
    TerrainMap terrain;     ///< 有效地形。
    RobotCapability robot;  ///< 通行能力。
  };
  /// @brief 控制结果，默认零速，只有有效命令可以被驱动执行。
  struct ChassisCommand {
    DataHeader header;                     ///< 生成时刻，坐标系必须为车体系。
    Vec3d linear_velocity{Vec3d::Zero()};  ///< 车体系速度，m/s。
    double yaw_rate_rps{0};                ///< rad/s。
    Timestamp valid_until_ns{0};           ///< 命令有效期限。
    DataStatus status{DataStatus::kUnavailable};  ///< 无效或过期必须停车。
  };
}  // namespace pure::core
