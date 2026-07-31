#include "mini_vo/backend/robust_policy.h"

#include <cmath>
#include <stdexcept>

#include <g2o/core/robust_kernel_impl.h>

namespace mini_vo {

void attachHuberKernels(G2OGraph& graph, double delta) {
    if (!std::isfinite(delta) || delta <= 0.0) {
        throw std::invalid_argument("Huber delta must be positive");
    }
    for (const EdgeBinding& binding : graph.edge_bindings) {
        auto* kernel = new g2o::RobustKernelHuber();
        kernel->setDelta(delta);
        binding.edge->setRobustKernel(kernel);
    }
}

void removeRobustKernels(G2OGraph& graph) {
    for (const EdgeBinding& binding : graph.edge_bindings) {
        binding.edge->setRobustKernel(nullptr);
    }
}

OutlierClassification classifyOutliers(
    G2OGraph& graph,
    const RobustPolicyOptions& options,
    bool deactivate_outliers) {
    OutlierClassification result;
    for (const EdgeBinding& binding : graph.edge_bindings) {
        binding.edge->computeError();
        const bool positive_depth = edgeDepthPositive(binding);
        const double chi2 = binding.edge->chi2();
        const bool inlier = positive_depth && std::isfinite(chi2) &&
                            chi2 <= options.chi2_threshold;
        binding.edge->setLevel(deactivate_outliers && !inlier ? 1 : 0);
        if (inlier) {
            ++result.inliers;
        } else {
            ++result.outliers;
            result.rejected.push_back(
                {binding.observation_index, binding.keyframe_id,
                 binding.map_point_id, chi2, positive_depth});
        }
    }
    return result;
}

}  // namespace mini_vo
