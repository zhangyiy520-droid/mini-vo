#include "mini_vo/backend/reprojection_error.h"

#include <cassert>
#include <cmath>
#include <iostream>

int main() {
    const mini_vo::PinholeCamera camera(500.0, 500.0, 320.0, 240.0,
                                        640, 480);

    mini_vo::KeyFrame keyframe;
    keyframe.id = 1;
    keyframe.Rcw = cv::Mat::eye(3, 3, CV_64F);
    keyframe.tcw = cv::Mat::zeros(3, 1, CV_64F);

    mini_vo::MapPoint map_point;
    map_point.id = 10;
    map_point.position_world = {0.0, 0.0, 5.0};

    mini_vo::Observation exact{1, 10, 0, {320.0f, 240.0f}, 0, false};
    auto zero = mini_vo::computeReprojectionError(
        exact, keyframe, map_point, camera);
    assert(zero.valid);
    assert(std::abs(zero.error_px) < 1e-9);

    auto shifted = exact;
    shifted.pixel = {323.0f, 244.0f};
    auto five = mini_vo::computeReprojectionError(
        shifted, keyframe, map_point, camera);
    assert(five.valid);
    assert(std::abs(five.error_px - 5.0) < 1e-9);
    assert(std::abs(five.residual[0] - 3.0) < 1e-9);
    assert(std::abs(five.residual[1] - 4.0) < 1e-9);

    map_point.position_world.z = -1.0;
    auto invalid = mini_vo::computeReprojectionError(
        exact, keyframe, map_point, camera);
    assert(!invalid.valid);
    assert(invalid.status == mini_vo::ProjectionStatus::BehindCamera);

    const auto stats = mini_vo::summarizeReprojectionErrors(
        {zero, five, invalid});
    assert(stats.valid_count == 2);
    assert(stats.invalid_count == 1);
    assert(std::abs(stats.mean - 2.5) < 1e-9);
    assert(std::abs(stats.maximum - 5.0) < 1e-9);

    std::cout << "[PASS] reprojection error mean=" << stats.mean
              << " max=" << stats.maximum << '\n';
    return 0;
}
