#pragma once
#include "pure_nav/point_cloud.hpp"
#include "pure_nav/types.hpp"
namespace pure_nav {
// Planar scan-to-map localization for the standalone simulator; not Point-LIO.
class ScanLocalizer {
  public:
    explicit ScanLocalizer(Pose initial) : pose_(initial) {
        if (!finite(initial))
            throw std::invalid_argument("invalid initial pose");
    }
    void predict(Velocity measured, double dt) { pose_ = integrate(pose_, measured, dt); }
    bool correct(const PointCloud &body_scan, const Grid &map);
    Pose pose() const { return pose_; }
    bool healthy() const { return healthy_; }
    double residual() const { return residual_; }

  private:
    Pose pose_;
    bool healthy_{false};
    double residual_{1e9};
};
} // namespace pure_nav
