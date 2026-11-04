#pragma once
#include <memory>
#include <vector>
namespace pure_nav {
struct PointXYZI {
    float x{}, y{}, z{}, intensity{};
};
struct PointCloud {
    using Ptr = std::shared_ptr<PointCloud>;
    std::vector<PointXYZI> points;
    void clear() { points.clear(); }
    void push_back(const PointXYZI &p) { points.push_back(p); }
    auto size() const { return points.size(); }
    bool empty() const { return points.empty(); }
    PointXYZI &back() { return points.back(); }
    PointXYZI &operator[](std::size_t i) { return points[i]; }
    const PointXYZI &operator[](std::size_t i) const { return points[i]; }
    PointCloud &operator+=(const PointCloud &rhs) {
        points.insert(points.end(), rhs.points.begin(), rhs.points.end());
        return *this;
    }
};
} // namespace pure_nav
