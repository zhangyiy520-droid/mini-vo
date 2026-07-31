#pragma once

#include <cstddef>
#include <string>

#include "mini_vo/backend/g2o_graph_builder.h"
#include "mini_vo/core/map.h"

namespace mini_vo {

struct PoseOptimizationOptions {
    int max_iterations = 15;
    double chi2_threshold = 5.991;
    std::size_t minimum_observations = 6;
};

struct PoseOptimizationReport {
    bool success = false;
    int iterations = 0;
    std::size_t inliers = 0;
    std::size_t outliers = 0;
    double initial_chi2 = 0.0;
    double final_chi2 = 0.0;
    std::string message;
};

class PoseOptimizer {
public:
    PoseOptimizationReport optimize(
        Map& map,
        KeyFrameId keyframe_id,
        const CameraIntrinsics& camera,
        const PoseOptimizationOptions& options = {}) const;
};

}  // namespace mini_vo
