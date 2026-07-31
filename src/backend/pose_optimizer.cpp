#include "mini_vo/backend/pose_optimizer.h"

#include <cmath>

#include <g2o/types/sba/types_six_dof_expmap.h>

namespace mini_vo {

PoseOptimizationReport PoseOptimizer::optimize(
    Map& map,
    KeyFrameId keyframe_id,
    const CameraIntrinsics& camera,
    const PoseOptimizationOptions& options) const {
    PoseOptimizationReport report;
    KeyFrame* keyframe = map.findKeyFrame(keyframe_id);
    if (keyframe == nullptr) {
        report.message = "keyframe not found";
        return report;
    }
    if (options.max_iterations <= 0) {
        report.message = "max_iterations must be positive";
        return report;
    }

    GraphBuildOptions build_options;
    build_options.mode = GraphMode::PoseOnly;
    build_options.keyframe_ids.insert(keyframe_id);
    GraphBuildResult build =
        buildG2OGraph(map, camera, build_options);
    if (!build.report.success || build.graph == nullptr) {
        report.message = build.report.message;
        return report;
    }
    if (build.report.edges < options.minimum_observations) {
        report.message = "not enough observations for pose optimization";
        return report;
    }

    g2o::SparseOptimizer& optimizer = *build.graph->optimizer;
    optimizer.initializeOptimization(0);
    optimizer.computeActiveErrors();
    report.initial_chi2 = optimizer.activeChi2();
    report.iterations = optimizer.optimize(options.max_iterations);
    optimizer.computeActiveErrors();
    report.final_chi2 = optimizer.activeChi2();

    for (const EdgeBinding& binding : build.graph->edge_bindings) {
        binding.edge->computeError();
        if (edgeDepthPositive(binding) &&
            binding.edge->chi2() <= options.chi2_threshold) {
            ++report.inliers;
        } else {
            ++report.outliers;
        }
    }

    const auto pose_id = build.graph->pose_vertex_ids.at(keyframe_id);
    const auto* pose_vertex = dynamic_cast<const g2o::VertexSE3Expmap*>(
        optimizer.vertex(pose_id));
    if (pose_vertex == nullptr || report.iterations <= 0 ||
        !std::isfinite(report.final_chi2) ||
        report.final_chi2 > report.initial_chi2) {
        report.message = "optimizer returned an invalid pose";
        return report;
    }
    const Eigen::Matrix4d transform = pose_vertex->estimate().to_homogeneous_matrix();
    if (!transform.allFinite()) {
        report.message = "optimized pose contains non-finite values";
        return report;
    }

    writeG2OPose(pose_vertex->estimate(), *keyframe);
    report.success = true;
    report.message = "ok";
    return report;
}

}  // namespace mini_vo
