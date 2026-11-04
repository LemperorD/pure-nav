#include "pure_nav/scan_localizer.hpp"
#include <limits>
namespace pure_nav {
bool ScanLocalizer::correct(const PointCloud &scan, const Grid &map) {
    healthy_ = false;
    if (scan.size() < 8)
        return false;
    std::vector<Pose> landmarks;
    for (int y = 0; y < map.height; ++y)
        for (int x = 0; x < map.width; ++x)
            if (map.at(x, y) >= 253)
                landmarks.push_back(map.center(x, y));
    if (landmarks.empty())
        return false;
    Pose estimate = pose_;
    // Point-to-point SE(2) ICP with a bounded association gate.
    for (int iteration = 0; iteration < 8; ++iteration) {
        std::vector<std::pair<Pose, Pose>> pairs;
        Pose a{}, b{};
        double error = 0;
        for (const auto &p : scan.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y))
                continue;
            Pose q{estimate.x + std::cos(estimate.yaw) * p.x - std::sin(estimate.yaw) * p.y,
                   estimate.y + std::sin(estimate.yaw) * p.x + std::cos(estimate.yaw) * p.y, 0};
            double best = .6;
            Pose match{};
            for (auto l : landmarks) {
                const auto d = distance(q, l);
                if (d < best) {
                    best = d;
                    match = l;
                }
            }
            if (best >= .6)
                continue;
            pairs.push_back({q, match});
            a.x += q.x;
            a.y += q.y;
            b.x += match.x;
            b.y += match.y;
            error += best * best;
        }
        if (pairs.size() < 8 || pairs.size() < scan.size() / 2)
            return false;
        const auto n = static_cast<double>(pairs.size());
        a.x /= n;
        a.y /= n;
        b.x /= n;
        b.y /= n;
        double dot = 0, cross = 0;
        for (auto pair : pairs) {
            const auto u = pair.first, v = pair.second;
            dot += (u.x - a.x) * (v.x - b.x) + (u.y - a.y) * (v.y - b.y);
            cross += (u.x - a.x) * (v.y - b.y) - (u.y - a.y) * (v.x - b.x);
        }
        const double angle = std::atan2(cross, dot), c = std::cos(angle), s = std::sin(angle);
        estimate = {b.x + c * (estimate.x - a.x) - s * (estimate.y - a.y),
                    b.y + s * (estimate.x - a.x) + c * (estimate.y - a.y), wrap(estimate.yaw + angle)};
        residual_ = std::sqrt(error / n);
    }
    if (distance(estimate, pose_) > .5 || std::abs(wrap(estimate.yaw - pose_.yaw)) > .3 || residual_ > .20)
        return false;
    pose_ = estimate;
    healthy_ = true;
    return true;
}
} // namespace pure_nav
