#pragma once
#include "pure_nav/pursuit.hpp"
#include "pure_nav/scan_localizer.hpp"
#include "simple_decision/core/decision.hpp"
#include "simple_decision/core/environment_context.hpp"
#include "terrain_analysis/per_frame_height_map.hpp"
#include "terrain_analysis/persistent_voxel_map.hpp"
namespace pure_nav {
struct NavigationOutput {
    Velocity command;
    Pose estimate;
    Path path;
    bool localized{}, planned{}, arrived{};
    simple_decision::ChassisMode chassis_mode{simple_decision::ChassisMode::CHASSIS_FOLLOWED};
    simple_decision::State state{simple_decision::State::DEFAULT};
};
class Navigation {
  public:
    Navigation(Grid prior, Pose initial, simple_decision::ContextConfig config);
    NavigationOutput tick(const PointCloud &scan, Velocity measured, double stamp,
                          const simple_decision::RobotStatus &status);
    // Single-thread API. Target is in map frame, valid for 0.5 seconds.
    void onTarget(Pose target, double stamp);
    void onGameStatus(simple_decision::GameStatus status, double stamp);

  private:
    Grid prior_;
    ScanLocalizer localizer_;
    Pursuit controller_;
    PerFrameHeightConfig height_config_;
    PersistentVoxelConfig voxel_config_;
    terrain_analysis::PersistentVoxelMap voxels_;
    terrain_analysis::PerFrameHeightMap terrain_;
    simple_decision::EnvironmentContext context_;
    simple_decision::Decision decision_;
    double last_stamp_{-1};
    std::optional<Pose> target_;
    double target_stamp_{-1};
    double combat_range_;
};
Path plan(const Grid &map, Pose start, Pose goal);
} // namespace pure_nav
