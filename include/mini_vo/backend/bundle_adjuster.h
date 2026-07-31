#pragma once

#include <cstddef>
#include <string>
#include <unordered_set>
#include <vector>

#include "mini_vo/backend/g2o_graph_builder.h"
#include "mini_vo/backend/robust_policy.h"
#include "mini_vo/core/map.h"

namespace mini_vo {

struct BundleAdjustmentOptions {
    int robust_iterations = 3;
    int final_iterations = 10;
    std::size_t minimum_edges = 12;
    bool use_robust_kernel = true;
    RobustPolicyOptions robust_policy;
    std::unordered_set<KeyFrameId> keyframe_ids;
    std::unordered_set<MapPointId> map_point_ids;
    std::unordered_set<KeyFrameId> fixed_keyframe_ids;
};

struct BundleAdjustmentReport {
    bool success = false;
    int iterations = 0;
    std::size_t pose_vertices = 0;
    std::size_t point_vertices = 0;
    std::size_t edges = 0;
    std::size_t inliers = 0;
    std::size_t outliers = 0;
    double initial_chi2 = 0.0;
    double final_chi2 = 0.0;
    std::vector<RejectedObservation> rejected;
    std::string message;
};

class BundleAdjuster {
public:
    BundleAdjustmentReport optimize(
        Map& map,
        const CameraIntrinsics& camera,
        const BundleAdjustmentOptions& options) const;
};

}  // namespace mini_vo
