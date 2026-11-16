#include <limits>
#include <type_traits>

#include "../test_common/test_check.hpp"
#include "autoaim_factory.hpp"
#include "navigation_factory.hpp"

namespace {
  using namespace pure::core;

  class TestTerrain final : public ITerrain {
  public:
    explicit TestTerrain(double frequency) : frequency_(frequency) {
    }
    void init() override {
      initialized = true;
    }
    double getFreq() const noexcept override {
      return frequency_;
    }
    TerrainMap analyze(const TerrainInput&) override {
      return {};
    }
    bool initialized{false};

  private:
    double frequency_;
  };

  class TestDetector final : public IDetector {
  public:
    void init() override {
    }
    double getFreq() const noexcept override {
      return 0;
    }
    DetectionFrame detect(const Image& image) override {
      DetectionFrame result;
      result.header = image.header;
      result.status = DataStatus::kValid;
      return result;
    }
  };

  class EmptyTerrainFactory final : public INavigationFactory {
  public:
    std::unique_ptr<INavigationProduct> createTypedProduct() const override {
      return nullptr;
    }
  };

  void testFactories() {
    NavigationFactory<TestTerrain> factory(
        [] { return std::make_unique<TestTerrain>(20.0); });
    auto first = factory.create();
    auto second = factory.create();
    CHECK(first.get() != second.get());
    CHECK(!first->initialized);
    CHECK_NEAR(first->getFreq(), 20.0, 1e-12);
    first->init();
    CHECK(first->initialized);
    CHECK(!second->initialized);
    CHECK(first->module() == NavigationModule::kTerrain);
    const IFactory& generic = factory;
    auto erased = generic.createProduct();
    CHECK(dynamic_cast<ITerrain*>(erased.get()) != nullptr);

    AutoAimFactory<TestDetector> detector_factory;
    auto detector = detector_factory.create();
    Image image;
    image.header.session = 8;
    image.header.sequence = 42;
    image.header.stamp_ns = 100;
    image.header.frame = Frame::kCamera;
    const auto detections = detector->detect(image);
    CHECK(detections.status == DataStatus::kValid);
    CHECK(detections.detections.empty());
    CHECK(detections.header.sequence == image.header.sequence);
    CHECK(detections.header.session == image.header.session);
    CHECK(detector->module() == AutoAimModule::kDetector);

    bool rejected = false;
    try {
      NavigationFactory<TestTerrain> empty(
          NavigationFactory<TestTerrain>::Creator{});
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    CHECK(rejected);
    NavigationFactory<TestTerrain> null_factory(
        [] { return std::unique_ptr<TestTerrain>{}; });
    rejected = false;
    try {
      (void)null_factory.createTypedProduct();
    } catch (const std::runtime_error&) {
      rejected = true;
    }
    CHECK(rejected);
    EmptyTerrainFactory empty;
    rejected = false;
    try {
      (void)empty.createProduct();
    } catch (const std::runtime_error&) {
      rejected = true;
    }
    CHECK(rejected);
  }

  void testTerrainContracts() {
    TerrainCell unknown;
    CHECK(unknown.type == TerrainType::kUnknown);
    CHECK(unknown.support == GroundSupport::kUnknown);
    CHECK(unknown.traversal == Traversability::kUnknown);
    CHECK(std::isnan(unknown.ground_z));
    CHECK(std::isnan(unknown.clearance_m));
    CHECK(std::isinf(unknown.traversal_cost));
    CHECK(!RobotCapability{}.configured);
    CHECK(ChassisCommand{}.linear_velocity.isZero());
    CHECK(ChassisCommand{}.status == DataStatus::kUnavailable);
    CHECK(TargetState{}.tracking == TrackingState::kLost);
    CHECK(NavigationGoal{}.task == NavigationTask::kHold);

    GridMap2D grid;
    CHECK(!grid.valid());
    grid.width = 2;
    grid.height = 3;
    grid.resolution_m = 0.1;
    grid.cells.resize(6);
    grid.cells[5].type = TerrainType::kHole;
    grid.cells[5].support = GroundSupport::kMissing;
    CHECK(grid.valid());
    CHECK(grid.at(1, 2).type == TerrainType::kHole);
    CHECK(grid.at(0, 0).type == TerrainType::kUnknown);
    bool rejected = false;
    try {
      (void)grid.at(2, 0);
    } catch (const std::out_of_range&) {
      rejected = true;
    }
    CHECK(rejected);
    grid.width = std::numeric_limits<std::size_t>::max();
    CHECK(!grid.valid());
    grid.width = 2;
    grid.resolution_m = std::numeric_limits<double>::quiet_NaN();
    CHECK(!grid.valid());
    static_assert(std::is_abstract_v<ISlam> && std::is_abstract_v<IPlan>
                  && std::is_abstract_v<IControl>
                  && std::is_abstract_v<ITracker>
                  && std::is_abstract_v<IAimSolver>);
    static_assert(!std::is_base_of_v<INavigationProduct, TestDetector>);
    static_assert(!std::is_base_of_v<IAutoAimProduct, TestTerrain>);
  }
}  // namespace

int main() {
  pure_nav_test::run("factories", testFactories);
  pure_nav_test::run("terrain contracts", testTerrainContracts);
  return pure_nav_test::summary();
}
