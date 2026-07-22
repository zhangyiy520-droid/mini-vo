#pragma once

#include <opencv2/core.hpp>

namespace mini_vo {

enum class ProjectionStatus {
    Valid,
    BehindCamera,
    OutsideImage,
    NonFinite
};

struct ProjectionResult {
    ProjectionStatus status = ProjectionStatus::NonFinite;
    cv::Point2d pixel{};
    cv::Point3d camera_point{};
};

class PinholeCamera {
public:
    PinholeCamera(double fx, double fy, double cx, double cy,
                  int width, int height);

    ProjectionResult project(const cv::Point3d& point_world,
                             const cv::Mat& Rcw,
                             const cv::Mat& tcw) const;

private:
    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
    int width_ = 0;
    int height_ = 0;
};

}  // namespace mini_vo
