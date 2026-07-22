#include "mini_vo/camera/pinhole_camera.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

bool near(double actual, double expected, double epsilon = 1e-9) {
    return std::abs(actual - expected) < epsilon;
}

}  // namespace

int main() {
    const mini_vo::PinholeCamera camera(500.0, 500.0, 320.0, 240.0,
                                        640, 480);
    const cv::Mat Rcw = cv::Mat::eye(3, 3, CV_64F);
    const cv::Mat tcw = cv::Mat::zeros(3, 1, CV_64F);

    const auto center = camera.project({0.0, 0.0, 5.0}, Rcw, tcw);
    assert(center.status == mini_vo::ProjectionStatus::Valid);
    assert(near(center.pixel.x, 320.0));
    assert(near(center.pixel.y, 240.0));
    assert(near(center.camera_point.z, 5.0));

    const auto right = camera.project({1.0, 0.0, 5.0}, Rcw, tcw);
    assert(right.status == mini_vo::ProjectionStatus::Valid);
    assert(near(right.pixel.x, 420.0));
    assert(near(right.pixel.y, 240.0));

    const auto behind = camera.project({0.0, 0.0, -1.0}, Rcw, tcw);
    assert(behind.status == mini_vo::ProjectionStatus::BehindCamera);

    const auto outside = camera.project({10.0, 0.0, 1.0}, Rcw, tcw);
    assert(outside.status == mini_vo::ProjectionStatus::OutsideImage);

    const cv::Mat bad_R = cv::Mat::eye(2, 2, CV_64F);
    const auto invalid_pose = camera.project({0.0, 0.0, 5.0}, bad_R, tcw);
    assert(invalid_pose.status == mini_vo::ProjectionStatus::NonFinite);

    bool threw = false;
    try {
        const mini_vo::PinholeCamera bad_camera(0.0, 500.0, 320.0, 240.0,
                                                640, 480);
        (void)bad_camera;
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);

    std::cout << "[PASS] pinhole projection\n";
    return 0;
}
