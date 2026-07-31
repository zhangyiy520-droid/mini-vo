#pragma once

#include <cstddef>
#include <string>

#include "mini_vo/backend/g2o_graph_builder.h"
#include "mini_vo/core/map.h"

namespace mini_vo {

struct PointOptimizationOptions {
    int max_iterations = 15;
    std::size_t minimum_observations = 2;
};

struct PointOptimizationReport {
    bool success = false;
    int iterations = 0;
    double initial_chi2 = 0.0;
    double final_chi2 = 0.0;
    std::string message;
};

class PointOptimizer {
public:
    PointOptimizationReport optimize(
        Map& map,
        MapPointId map_point_id,
        const CameraIntrinsics& camera,
        const PointOptimizationOptions& options = {}) const;
};

}  // namespace mini_vo
