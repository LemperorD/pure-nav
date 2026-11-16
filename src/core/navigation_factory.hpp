#pragma once

#include "abstract.hpp"
#include "nav_ds.hpp"

namespace pure::core {

/// @brief 导航算法模块类别。
enum class NavigationModule : std::uint8_t {
  kSlam,
  kTerrain,
  kPlanner,
  kController
};

/// @brief 导航产品公共接口
class INavigationProduct : public IProduct {
public:
  /// @brief 返回模块类别，便于装配和诊断。
  virtual NavigationModule module() const noexcept = 0;
};

  /// @brief 导航工厂接口，只接受导航产品。
  class INavigationFactory : public ITypedFactory<INavigationProduct> {};
  /// @brief 用具体导航算法创建工厂，也支持构造回调注入配置。
  /// @tparam Product 具体导航产品类。
  template <typename Product>
  using NavigationFactory = ConcreteFactory<Product, INavigationFactory>;

  /// @brief 同步定位与建图接口。
  class ISlam : public INavigationProduct {
  public:
    /// @brief 建图或使用已有地图定位。
    enum class SlamMode : std::uint8_t { kMapping, kLocalization };
    /// @brief 返回定位模块类别。
    NavigationModule module() const noexcept final {
      return NavigationModule::kSlam;
    }
    /// @brief 返回当前工作模式，由具体算法保存模式状态。
    virtual SlamMode getMode() const noexcept = 0;
    /// @brief 切换模式；失败抛异常，成功后应重置相关地图和历史状态。
    /// @param mode 请求的工作模式。
    virtual void setMode(SlamMode mode) = 0;
    /// @brief 安装已知地图，失败抛异常；地图坐标系和外参须与定位配置一致。
    /// @param prior 由调用方拥有的先验地图快照，算法须拷贝需要长期保存的数据。
    virtual void setPriorMap(const PointCloudMap& prior) = 0;
    /// @brief 输入同步扫描及惯性历史，输出带健康状态的三维位姿。
    /// @param input 时间递增且时钟、外参明确的输入。
    virtual NavigationState localize(const LocalizationInput& input) = 0;
    /// @brief 在建图模式融合一次观测，返回地图快照；其他模式应报错。
    /// @param input 同步观测。
    virtual PointCloudMap map(const LocalizationInput& input) = 0;
  };
  /// @brief 提取地面、坡面、负障碍与洞口几何，并评估本车通行性。
  class ITerrain : public INavigationProduct {
  public:
    /// @brief 返回地形模块类别。
    NavigationModule module() const noexcept final {
      return NavigationModule::kTerrain;
    }
    /// @brief 使用完整三维姿态分析扫描；无回波区域保持未知。
    /// @param input 已配准点云、对应位姿和车体约束。
    virtual TerrainMap analyze(const TerrainInput& input) = 0;
  };
  /// @brief 地形约束规划接口，分别表达全局路线与局部轨迹。
  class IPlan : public INavigationProduct {
  public:
    /// @brief 返回规划模块类别。
    NavigationModule module() const noexcept final {
      return NavigationModule::kPlanner;
    }
    /// @brief 规划任务路线；未知、支撑缺失或能力超限区域不得默认放行。
    /// @param input 有效状态、任务、地形及车体约束。
    virtual Trajectory globalPlan(const PlanningInput& input) = 0;
    /// @brief 根据新地形更新局部轨迹，复核洞口净空和方向性坡度约束。
    /// @param input 当前规划快照。
    /// @param global_route 与当前任务一致的全局路线。
    virtual Trajectory localPlan(const PlanningInput& input,
                                 const Trajectory& global_route) = 0;
  };
  /// @brief 控制所用算法类别。
  enum class ControlAlgorithm : std::uint8_t { kPid, kMpc, kMppi, kOther };
  /// @brief 控制输入，控制周期独立于规划周期。
  struct ControlInput {
    Timestamp now_ns{0};    ///< 当前单调时钟时间。
    double dt_s{0};         ///< 本次控制间隔，s，必须有限且为正。
    NavigationState state;  ///< 有效位姿。
    Trajectory trajectory;  ///< 当前任务的有效轨迹。
    TerrainMap terrain;     ///< 碰撞及地形复核依据。
    RobotCapability robot;  ///< 速度及通行限制。
  };
  /// @brief 底盘轨迹跟踪接口，异常输入或预测碰撞时输出无效零速命令。
  class IControl : public INavigationProduct {
  public:
    /// @brief 返回控制模块类别。
    NavigationModule module() const noexcept final {
      return NavigationModule::kController;
    }
    /// @brief 返回控制后端类别。
    virtual ControlAlgorithm getCtrlAlg() const noexcept = 0;
    /// @brief 在本周期执行轨迹跟踪，输出车体系速度及有效期限。
    /// @param input 不依赖规划线程唤醒的最新输入快照。
    virtual ChassisCommand control(const ControlInput& input) = 0;
  };
}  // namespace pure::core
