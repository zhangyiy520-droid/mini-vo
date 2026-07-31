#pragma once

#include <cstddef>
#include <vector>

#include "mini_vo/backend/g2o_graph_builder.h"

namespace mini_vo {

struct RobustPolicyOptions {
    double huber_delta = 2.447651936;
    double chi2_threshold = 5.991;
};

struct RejectedObservation {
    std::size_t observation_index = 0;
    KeyFrameId keyframe_id = 0;
    MapPointId map_point_id = 0;
    double chi2 = 0.0;
    bool positive_depth = true;
};

struct OutlierClassification {
    std::size_t inliers = 0;
    std::size_t outliers = 0;
    std::vector<RejectedObservation> rejected;
};

void attachHuberKernels(G2OGraph& graph, double delta);
void removeRobustKernels(G2OGraph& graph);
OutlierClassification classifyOutliers(
    G2OGraph& graph,
    const RobustPolicyOptions& options,
    bool deactivate_outliers);

}  // namespace mini_vo
