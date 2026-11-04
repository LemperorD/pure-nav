#pragma once
#include "pb_omni_pid_pursuit_controller/core/pid.hpp"
#include "pure_nav/types.hpp"
namespace pure_nav {
class Pursuit {
  public:
    explicit Pursuit(double dt = 0.05);
    Velocity command(Pose pose, const Path &path, const Grid &collision_map);

  private:
    double dt_;
    pb_omni_pid_pursuit_controller::PID translation_, rotation_;
};
} // namespace pure_nav
