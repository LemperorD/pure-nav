#pragma once
#include "pure_nav/point_cloud.hpp"
#include "pure_nav/types.hpp"
namespace pure_nav {
class Simulator {
  public:
    Simulator(Grid world, Pose initial) : world_(std::move(world)), pose_(initial) {}
    PointCloud scan() const;
    bool step(Velocity command, double dt);
    Pose truth() const { return pose_; }
    Velocity measured() const { return measured_; }
    const Grid &world() const { return world_; }

  private:
    Grid world_;
    Pose pose_;
    Velocity measured_;
};
Grid demoWorld(bool blocked = false);
} // namespace pure_nav
