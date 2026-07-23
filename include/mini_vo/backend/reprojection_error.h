#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

#include "mini_vo/camera/pinhole_camera.h"
#include "mini_vo/core/map.h"

namespace mini_vo {

struct ReprojectionResult {
    cv::Vec2d residual{0.0, 0.0};
    double error_px = 0.0;
    double depth = 0.0;
    ProjectionStatus status = ProjectionStatus::NonFinite;
    bool valid = false;
};

struct ReprojectionStats {
    std::size_t valid_count = 0;
    std::size_t invalid_count = 0;
    double mean = 0.0;
    double median = 0.0;
    double p95 = 0.0;
    double maximum = 0.0;
};

ReprojectionResult computeReprojectionError(
    const Observation& observation,
    const KeyFrame& keyframe,
    const MapPoint& map_point,
    const PinholeCamera& camera);

ReprojectionStats summarizeReprojectionErrors(
    const std::vector<ReprojectionResult>& results);

}  // namespace mini_vo
