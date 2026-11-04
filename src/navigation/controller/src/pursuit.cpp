#include "pure_nav/pursuit.hpp"
namespace pure_nav {
Pursuit::Pursuit(double dt)
    : dt_(dt), translation_(dt, 1.0, 0, 1.8, 0, 0), rotation_(dt, 1.5, -1.5, 2, 0, 0) {
    if (!std::isfinite(dt) || dt <= 0)
        throw std::invalid_argument("invalid controller dt");
}
Velocity Pursuit::command(Pose pose, const Path &path, const Grid &map) {
    if (path.empty() || !finite(pose))
        return {};
    for (const auto &p : path)
        if (!finite(p))
            return {};
    const auto goal = path.back();
    if (distance(pose, goal) < .10)
        return {0, 0, rotation_.calculate(wrap(goal.yaw - pose.yaw), 0)};
    std::size_t nearest = 0;
    for (std::size_t i = 1; i < path.size(); ++i)
        if (distance(pose, path[i]) < distance(pose, path[nearest]))
            nearest = i;
    Pose target = path[nearest];
    for (std::size_t i = nearest; i < path.size(); ++i) {
        if (!map.segment(pose, path[i]))
            break;
        target = path[i];
        if (distance(pose, target) >= .35)
            break;
    }
    const double dx = target.x - pose.x, dy = target.y - pose.y, d = std::hypot(dx, dy);
    if (d < 1e-6)
        return {};
    const double speed = translation_.calculate(std::min(d, distance(pose, goal)), 0);
    Velocity v{speed * (std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy) / d,
               speed * (-std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy) / d, 0};
    // Sweep a short reaction horizon through the inflated footprint map.
    if (!map.segment(pose, integrate(pose, v, std::max(dt_, .3))))
        return {};
    return v;
}
} // namespace pure_nav
