#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
namespace pure_nav {
constexpr double pi = 3.14159265358979323846;
inline double wrap(double a) { return std::remainder(a, 2 * pi); }
struct Pose {
    double x{}, y{}, yaw{};
};
struct Velocity {
    double x{}, y{}, yaw{};
}; // body frame, m/s and rad/s
inline bool finite(Pose p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw); }
inline double distance(Pose a, Pose b) { return std::hypot(a.x - b.x, a.y - b.y); }
inline Pose integrate(Pose p, Velocity v, double dt) {
    if (!finite(p) || !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.yaw) ||
        !std::isfinite(dt) || dt <= 0)
        throw std::invalid_argument("invalid motion sample");
    const double h = p.yaw + v.yaw * dt / 2;
    return {p.x + (std::cos(h) * v.x - std::sin(h) * v.y) * dt,
            p.y + (std::sin(h) * v.x + std::cos(h) * v.y) * dt, wrap(p.yaw + v.yaw * dt)};
}
using Path = std::vector<Pose>;
struct Grid {
    int width, height;
    double resolution;
    double origin_x{}, origin_y{};
    std::vector<unsigned char> cells;
    Grid(int w, int h, double r) : width(w), height(h), resolution(r) {
        if (w <= 0 || h <= 0 || w > 10000 || h > 10000 || !std::isfinite(r) || r <= 0)
            throw std::invalid_argument("invalid grid");
        cells.resize(static_cast<std::size_t>(w) * h);
    }
    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < width && y < height; }
    unsigned char at(int x, int y) const {
        return inside(x, y) ? cells[static_cast<std::size_t>(y) * width + x] : 254;
    }
    bool blocked(double x, double y) const {
        if (!std::isfinite(x) || !std::isfinite(y) || x < origin_x || y < origin_y ||
            x >= origin_x + width * resolution || y >= origin_y + height * resolution)
            return true;
        return at(cellX(x), cellY(y)) >= 253;
    }
    int cellX(double x) const { return static_cast<int>(std::floor((x - origin_x) / resolution)); }
    int cellY(double y) const { return static_cast<int>(std::floor((y - origin_y) / resolution)); }
    Pose center(int x, int y) const {
        return {origin_x + (x + .5) * resolution, origin_y + (y + .5) * resolution, 0};
    }
    void mark(int x, int y, unsigned char c = 254) {
        if (inside(x, y))
            cells[static_cast<std::size_t>(y) * width + x] = c;
    }
    bool segment(Pose a, Pose b) const {
        if (!finite(a) || !finite(b))
            return false;
        const int n = std::max(1, static_cast<int>(std::ceil(distance(a, b) / (resolution * .25))));
        for (int i = 0; i <= n; ++i)
            if (blocked(a.x + (b.x - a.x) * i / n, a.y + (b.y - a.y) * i / n))
                return false;
        return true;
    }
    Grid inflated(double radius) const {
        if (!std::isfinite(radius) || radius < 0 || radius > 100)
            throw std::invalid_argument("invalid radius");
        Grid out = *this;
        const int n = static_cast<int>(std::ceil(radius / resolution));
        for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x)
                if (at(x, y) >= 253)
                    for (int dy = -n; dy <= n; ++dy)
                        for (int dx = -n; dx <= n; ++dx)
                            if (std::hypot(dx, dy) * resolution <= radius + resolution * .71)
                                out.mark(x + dx, y + dy);
        return out;
    }
};
} // namespace pure_nav
