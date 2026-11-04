#include "pure_nav/navigation.hpp"
#include "jps_planner/jps_algorithm.hpp"
namespace pure_nav {
Path plan(const Grid &map, Pose start, Pose goal) {
    if (!finite(start) || !finite(goal) || map.blocked(start.x, start.y) || map.blocked(goal.x, goal.y))
        return {};
    jps_planner::JPSState state;
    state.costmap_data = map.cells.data();
    state.size_x = map.width;
    state.size_y = map.height;
    jps_planner::JPSConfig config;
    std::vector<std::pair<double, double>> points;
    if (!jps_planner::JPSAlgorithm::generatePath(config, state, map.cellX(start.x), map.cellY(start.y),
                                                 map.cellX(goal.x), map.cellY(goal.y), points))
        return {};
    std::vector<std::pair<double, double>> dense;
    if (!points.empty())
        dense.push_back(points.front());
    for (std::size_t i = 1; i < points.size(); ++i) {
        const auto a = points[i - 1], b = points[i];
        const int count = std::max(1, static_cast<int>(std::ceil(std::max(std::abs(b.first - a.first),
                                                                          std::abs(b.second - a.second)))));
        for (int k = 1; k <= count; ++k)
            dense.emplace_back(a.first + (b.first - a.first) * k / count,
                               a.second + (b.second - a.second) * k / count);
    }
    points = std::move(dense);
    points = jps_planner::JPSAlgorithm::detourCornerHuggingDiagonals(points, map.cells.data(), map.width,
                                                                     map.height, false);
    Path out{start};
    for (auto p : points)
        out.push_back(
            {map.origin_x + p.first * map.resolution, map.origin_y + p.second * map.resolution, goal.yaw});
    out.push_back(goal);
    for (std::size_t i = 1; i < out.size(); ++i)
        if (!map.segment(out[i - 1], out[i]))
            return {};
    return out;
}
Navigation::Navigation(Grid prior, Pose initial, simple_decision::ContextConfig config)
    : prior_(std::move(prior)), localizer_(initial), controller_(), voxels_(voxel_config_),
      terrain_(height_config_), context_(config), decision_(config),
      combat_range_(config.combat_max_distance) {
    height_config_.min_block_point_num = 3;
    voxel_config_.decay_time = .5;
}
void Navigation::onTarget(Pose target, double stamp) {
    if (!finite(target) || !std::isfinite(stamp) || stamp < 0 || stamp > 1e9)
        throw std::invalid_argument("invalid target");
    if (stamp < target_stamp_)
        return;
    target_ = target;
    target_stamp_ = stamp;
}
void Navigation::onGameStatus(simple_decision::GameStatus status, double stamp) {
    if (!std::isfinite(stamp) || stamp < 0 || stamp > 1e9)
        throw std::invalid_argument("invalid game timestamp");
    context_.onGameStatus(status, static_cast<int64_t>(stamp * 1e9));
}
NavigationOutput Navigation::tick(const PointCloud &scan, Velocity measured, double stamp,
                                  const simple_decision::RobotStatus &status) {
    NavigationOutput out;
    out.estimate = localizer_.pose();
    if (!std::isfinite(stamp) || stamp < 0 || stamp > 1e9 || (last_stamp_ >= 0 && stamp <= last_stamp_))
        return out;
    const double dt = last_stamp_ < 0 ? .05 : stamp - last_stamp_;
    last_stamp_ = stamp;
    if (dt > .2)
        return out; // stale data: stop; no large prediction step
    localizer_.predict(measured, dt);
    PointCloud obstacle_scan;
    for (auto p : scan.points)
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && p.z > .04F && p.z < .62F)
            obstacle_scan.push_back(p);
    out.localized = localizer_.correct(obstacle_scan, prior_);
    out.estimate = localizer_.pose();
    if (!out.localized)
        return out;
    const auto p = out.estimate;
    PointCloud registered;
    for (auto q : scan.points) {
        if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z))
            continue;
        registered.push_back({static_cast<float>(p.x + std::cos(p.yaw) * q.x - std::sin(p.yaw) * q.y),
                              static_cast<float>(p.y + std::sin(p.yaw) * q.x + std::cos(p.yaw) * q.y), q.z,
                              0});
    }
    const guga_common::Point3d lidar{p.x, p.y, .23};
    voxels_.receiveFrame(registered, lidar, stamp);
    voxels_.update();
    PointCloud accumulated;
    voxels_.collectCloud(accumulated);
    terrain_.compute(accumulated, lidar);
    Grid obstacles = prior_;
    for (auto q : terrain_.obstacleCloud().points) {
        if (q.x >= prior_.origin_x && q.y >= prior_.origin_y &&
            q.x < prior_.origin_x + prior_.width * prior_.resolution &&
            q.y < prior_.origin_y + prior_.height * prior_.resolution)
            obstacles.mark(prior_.cellX(q.x), prior_.cellY(q.y));
    }
    const Grid collision = obstacles.inflated(.22);
    context_.onRobotStatus(status);
    context_.updatePose(p.x, p.y, p.yaw);
    const auto ns = static_cast<int64_t>(stamp * 1e9);
    if (context_.checkReadiness(ns).status != simple_decision::Readiness::Status::READY)
        return out;
    auto snapshot = context_.buildSnapshot(simple_decision::toStamp(ns));
    context_.updateTracking(simple_decision::toStamp(ns), snapshot);
    if (target_ && stamp >= target_stamp_ && stamp - target_stamp_ <= .5 &&
        distance(p, *target_) <= combat_range_) {
        simple_decision::Target target;
        target.position = {target_->x, target_->y, 0};
        target.yaw = target_->yaw;
        target.tracking = true;
        snapshot.target_opt = target;
        snapshot.enemy_recent = true;
    }
    auto action = decision_.computeAction(snapshot);
    context_.changeState(action.next_state);
    out.state = action.next_state;
    out.chassis_mode = action.chassis_mode;
    Pose goal{action.target_x, action.target_y, action.target_yaw};
    out.arrived = distance(p, goal) < .12 && std::abs(wrap(p.yaw - goal.yaw)) < .1;
    if (!action.should_publish_goal || out.arrived)
        return out;
    out.path = plan(collision, p, goal);
    out.planned = !out.path.empty();
    out.command = controller_.command(p, out.path, collision);
    return out;
}
} // namespace pure_nav
