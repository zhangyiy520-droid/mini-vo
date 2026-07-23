#include "mini_vo/backend/reprojection_error.h"

#include <algorithm>
#include <cmath>

namespace mini_vo {

ReprojectionResult computeReprojectionError(
    const Observation& observation,
    const KeyFrame& keyframe,
    const MapPoint& map_point,
    const PinholeCamera& camera) {
    ReprojectionResult result;
    const ProjectionResult projection = camera.project(
        map_point.position_world, keyframe.Rcw, keyframe.tcw);

    result.status = projection.status;
    result.depth = projection.camera_point.z;
    if (projection.status != ProjectionStatus::Valid ||
        observation.outlier || map_point.bad) {
        return result;
    }

    result.residual[0] = static_cast<double>(observation.pixel.x) -
                         projection.pixel.x;
    result.residual[1] = static_cast<double>(observation.pixel.y) -
                         projection.pixel.y;
    result.error_px = std::sqrt(result.residual.dot(result.residual));
    result.valid = std::isfinite(result.error_px);
    return result;
}

ReprojectionStats summarizeReprojectionErrors(
    const std::vector<ReprojectionResult>& results) {
    ReprojectionStats stats;
    std::vector<double> errors;
    for (const auto& result : results) {
        if (result.valid) {
            errors.push_back(result.error_px);
        } else {
            ++stats.invalid_count;
        }
    }

    stats.valid_count = errors.size();
    if (errors.empty()) {
        return stats;
    }

    std::sort(errors.begin(), errors.end());
    for (double error : errors) {
        stats.mean += error;
    }
    stats.mean /= static_cast<double>(errors.size());
    stats.median = errors[errors.size() / 2];
    const std::size_t p95_index = static_cast<std::size_t>(
        std::ceil(0.95 * static_cast<double>(errors.size()))) - 1;
    stats.p95 = errors[p95_index];
    stats.maximum = errors.back();
    return stats;
}

}  // namespace mini_vo
