#ifndef AUTOAIM_DS_HPP
#define AUTOAIM_DS_HPP

#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;
using Image = cv::Mat;

struct DetectResult {
    cv::Rect bounding_box;
    float confidence;
    int class_id;
};
using DetectResults = std::vector<DetectResult>;

struct GimbalState {
    float pitch;
    float yaw;
    float roll;
};

#endif // AUTOAIM_DS_HPP