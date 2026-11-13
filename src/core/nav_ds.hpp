#ifndef NAV_DS_HPP
#define NAV_DS_HPP

#include <vector>
#include <pcl/point_types.h>

struct Point2D {
    double x;
    double y;
};

struct Trajectory {
    std::vector<Point2D> waypoints;
};

struct GridMap2D {
    int width;
    int height;
    std::vector<std::vector<int>> grid; // 0 for free space, 1 for obstacles
};

struct GridMap3D {
    int width;
    int height;
    int depth;
    std::vector<std::vector<std::vector<int>>> grid; // 0 for free space, 1 for obstacles
};

struct PointCloudMap {
    std::vector<pcl::PointXYZ> points;
};

#endif // NAV_DS_HPP