#pragma once

#include "abstract.hpp"
#include "autoaim_ds.hpp"

namespace pure::core {
  /// @brief 自瞄算法模块类别。
  enum class AutoAimModule : std::uint8_t { kDetector, kTracker, kSolver };
  /// @brief 自瞄产品公共接口，不负责相机采集或串口收发。
  class IAutoAimProduct : public IProduct {
  public:
    /// @brief 返回模块类别。
    virtual AutoAimModule module() const noexcept = 0;
  };
  /// @brief 自瞄工厂接口，只接受自瞄产品。
  class IAutoAimFactory : public ITypedFactory<IAutoAimProduct> {};
  /// @brief 使用具体算法或构造回调创建自瞄产品。
  /// @tparam Product 具体自瞄产品类。
  template <typename Product>
  using AutoAimFactory = ConcreteFactory<Product, IAutoAimFactory>;
  /// @brief 从图像中检测比赛目标，推理结果保留原图身份。
  class IDetector : public IAutoAimProduct {
  public:
    /// @brief 返回检测模块类别。
    AutoAimModule module() const noexcept final {
      return AutoAimModule::kDetector;
    }
    /// @brief 处理单帧图像；有效的空检测与推理失败必须区分。
    /// @param image 拥有数据且像素布局明确的输入。
    virtual DetectionFrame detect(const Image& image) = 0;
  };
  /// @brief 结合标定和同步姿态估计三维目标并保持跟踪身份。
  class ITracker : public IAutoAimProduct {
  public:
    /// @brief 返回跟踪模块类别。
    AutoAimModule module() const noexcept final {
      return AutoAimModule::kTracker;
    }
    /// @brief 更新三维估计；无观测时只能在有效期限内预测。
    /// @param input 原图、标定及曝光时刻的观察者状态。
    /// @param detections 与原图时间、会话、序号一致的检测。
    virtual TargetState track(const AutoAimInput& input,
                              const DetectionFrame& detections) = 0;
  };
  /// @brief 使用三维目标和当前车体姿态解算云台跟随角度。
  class IAimSolver : public IAutoAimProduct {
  public:
    /// @brief 返回解算模块类别。
    AutoAimModule module() const noexcept final {
      return AutoAimModule::kSolver;
    }
    /// @brief 解算跟随角度，过期目标或无效姿态返回无效结果。
    /// @param target 具有明确参考系的目标估计。
    /// @param observer 当前观察者状态，包含坡面姿态。
    /// @param now_ns 当前单调时钟时间。
    virtual AimSolution solve(const TargetState& target,
                              const ObserverState& observer,
                              Timestamp now_ns) = 0;
  };
}  // namespace pure::core
