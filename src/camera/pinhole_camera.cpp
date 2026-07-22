#include "mini_vo/camera/pinhole_camera.h"

#include <cmath>
#include <stdexcept>

namespace mini_vo {

PinholeCamera::PinholeCamera(double fx, double fy, double cx, double cy,
                             int width, int height)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy),
      width_(width), height_(height) {
    if (fx <= 0.0 || fy <= 0.0 || width <= 0 || height <= 0) {
        throw std::invalid_argument("invalid pinhole camera parameters");
    }
}

ProjectionResult PinholeCamera::project(const cv::Point3d& point_world,
                                        const cv::Mat& Rcw,
                                        const cv::Mat& tcw) const {
    ProjectionResult result;
    if (Rcw.rows != 3 || Rcw.cols != 3 ||
        tcw.rows != 3 || tcw.cols != 1) {
        return result;
    }

    const cv::Mat point = (cv::Mat_<double>(3, 1) <<
                           point_world.x, point_world.y, point_world.z);
    const cv::Mat camera = Rcw * point + tcw;
    result.camera_point = {
        camera.at<double>(0),
        camera.at<double>(1),
        camera.at<double>(2)
    };

    const double x = result.camera_point.x;
    const double y = result.camera_point.y;
    const double z = result.camera_point.z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        result.status = ProjectionStatus::NonFinite;
        return result;
    }
    if (z <= 1e-9) {
        result.status = ProjectionStatus::BehindCamera;
        return result;
    }

    result.pixel.x = fx_ * x / z + cx_;
    result.pixel.y = fy_ * y / z + cy_;
    if (!std::isfinite(result.pixel.x) || !std::isfinite(result.pixel.y)) {
        result.status = ProjectionStatus::NonFinite;
        return result;
    }
    if (result.pixel.x < 0.0 || result.pixel.x >= width_ ||
        result.pixel.y < 0.0 || result.pixel.y >= height_) {
        result.status = ProjectionStatus::OutsideImage;
        return result;
    }

    result.status = ProjectionStatus::Valid;
    return result;
}

}  // namespace mini_vo
