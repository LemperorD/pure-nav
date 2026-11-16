#pragma once

#include <functional>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace pure::core {

/// @brief 算法生命周期；同一实例由一个调度线程调用。
class IProduct {
public:
  /// @brief 释放算法资源。
  virtual ~IProduct() = default;
  /// @brief 初始化或重置算法；失败抛异常，不启动业务线程。
  virtual void init() = 0;
  /// @brief 建议调用频率，单位 Hz；0 表示输入事件触发。
  virtual double getFreq() const noexcept = 0;
};

/// @brief 创建独立算法实例，所有权交给装配层。
class IFactory {
public:
  /// @brief 释放工厂配置。
  virtual ~IFactory() = default;
  /// @brief 创建非空产品；失败抛异常，调用方随后调用 init。
  virtual std::unique_ptr<IProduct> createProduct() const = 0;
};

/// @brief 保留产品类别的工厂接口。
/// @tparam Product 导航或自瞄产品基类。
template <typename Product>
class ITypedFactory : public IFactory {
public:
  using ProductType = Product;  ///< 允许创建的产品类别。
  /// @brief 创建类型明确的非空实例；失败抛异常。
  virtual std::unique_ptr<Product> createTypedProduct() const = 0;
  /// @brief 向通用装配层返回产品，拒绝空结果。
  std::unique_ptr<IProduct> createProduct() const final {
    auto product = createTypedProduct();
    if (!product) {
      throw std::runtime_error("factory returned an empty product");
    }
    return product;
  }
};

/// @brief 使用默认构造或注入的回调创建具体算法。
/// @tparam Product 非抽象的具体算法类。
/// @tparam Factory 导航或自瞄工厂接口。
template <typename Product, typename Factory>
class ConcreteFactory final : public Factory {
  static_assert(std::is_base_of_v<typename Factory::ProductType, Product>,
                "product does not belong to this factory family");
  static_assert(!std::is_abstract_v<Product>, "product must be concrete");

public:
  using Creator = std::function<std::unique_ptr<Product>()>;  ///< 构造回调。
  /// @brief 使用产品的默认构造函数。
  ConcreteFactory()
      : ConcreteFactory([] { return std::make_unique<Product>(); }) {
  }
  /// @brief 注入配置或依赖，捕获的数据须保持有效。
  /// @param creator 每次创建新实例的回调；空回调抛 invalid_argument。
  explicit ConcreteFactory(Creator creator) : creator_(std::move(creator)) {
    if (!creator_) {
      throw std::invalid_argument("factory creator is empty");
    }
  }
  /// @brief 创建具体类型实例，直接调用其算法接口；空结果抛 runtime_error。
  std::unique_ptr<Product> create() const {
    auto product = creator_();
    if (!product) {
      throw std::runtime_error("factory creator returned an empty product");
    }
    return product;
  }
  /// @brief 向类型化装配层返回未初始化的独立实例。
  std::unique_ptr<typename Factory::ProductType> createTypedProduct()
      const final {
    return create();
  }

private:
  Creator creator_;
};

}  // namespace pure::core
