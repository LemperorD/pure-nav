#include <cmath>

inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

inline double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

inline double ReLU(double x) {
    return std::max(0.0, x);
}

inline double sigmoid(double x) {
    return 1.0 / (1.0 + std::exp(-x));
}