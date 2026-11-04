#include "pure_nav/simulator.hpp"
namespace pure_nav {
Grid demoWorld(bool blocked) {
    Grid grid(100, 80, .1);
    for (int y = 0; y < 80; ++y)
        for (int x = 0; x < 100; ++x)
            if (x == 0 || y == 0 || x == 99 || y == 79 || (x == 48 && (blocked || y < 26 || y > 46)))
                grid.mark(x, y);
    for (int y = 51; y < 61; ++y)
        for (int x = 65; x < 75; ++x)
            grid.mark(x, y);
    return grid;
}
PointCloud Simulator::scan() const {
    PointCloud cloud;
    for (int i = 0; i < 180; ++i) {
        const double angle = 2 * pi * i / 180, c = std::cos(angle), s = std::sin(angle);
        const double wc = std::cos(angle + pose_.yaw), ws = std::sin(angle + pose_.yaw);
        for (double r = .1; r < 12; r += .04) {
            const double x = pose_.x + r * wc, y = pose_.y + r * ws;
            if (world_.blocked(x, y)) {
                cloud.push_back({static_cast<float>(r * c), static_cast<float>(r * s), .3F, 0});
                break;
            }
            if (r < 4.8)
                cloud.push_back({static_cast<float>(r * c), static_cast<float>(r * s), 0, 0});
        }
    }
    return cloud;
}
bool Simulator::step(Velocity command, double dt) {
    auto next = integrate(pose_, command, dt);
    if (!world_.inflated(.20).segment(pose_, next)) {
        measured_ = {};
        return false;
    }
    pose_ = next;
    measured_ = {command.x * 1.01, command.y * .99, command.yaw}; // encoder scale error
    return true;
}
} // namespace pure_nav
