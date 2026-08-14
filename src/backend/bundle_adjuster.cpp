#include "mini_vo/backend/bundle_adjuster.h"

#include <algorithm>
#include <cmath>

#include <g2o/types/sba/types_six_dof_expmap.h>

namespace mini_vo {
namespace {

bool estimateIsFinite(const G2OGraph& graph) {
    for (const auto& item : graph.pose_vertex_ids) {
        const auto* vertex = dynamic_cast<const g2o::VertexSE3Expmap*>(
            graph.optimizer->vertex(item.second));
        if (vertex == nullptr ||
            !vertex->estimate().to_homogeneous_matrix().allFinite()) {
            return false;
        }
    }
    for (const auto& item : graph.point_vertex_ids) {
        const auto* vertex = dynamic_cast<const g2o::VertexPointXYZ*>(
            graph.optimizer->vertex(item.second));
        if (vertex == nullptr || !vertex->estimate().allFinite()) {
            return false;
        }
    }
    return true;
}

}  // namespace

BundleAdjustmentReport BundleAdjuster::optimize(
    Map& map,
    const CameraIntrinsics& camera,
    const BundleAdjustmentOptions& options) const {
    BundleAdjustmentReport report;
    if (options.robust_iterations < 0 || options.final_iterations < 0 ||
        options.robust_iterations + options.final_iterations == 0) {
        report.message = "iteration counts are invalid";
        return report;
    }
    if (!options.robust_policy.valid()) {
        report.message = "robust policy options are invalid";
        return report;
    }

    GraphBuildOptions build_options;
    build_options.mode = GraphMode::FullBundleAdjustment;
    build_options.keyframe_ids = options.keyframe_ids;
    build_options.map_point_ids = options.map_point_ids;
    build_options.fixed_keyframe_ids = options.fixed_keyframe_ids;
    GraphBuildResult build =
        buildG2OGraph(map, camera, build_options);
    report.pose_vertices = build.report.pose_vertices;
    report.point_vertices = build.report.point_vertices;
    report.edges = build.report.edges;
    if (!build.report.success || build.graph == nullptr) {
        report.message = build.report.message;
        return report;
    }
    if (build.report.fixed_vertices < 2U) {
        report.message =
            "monocular BA needs two fixed poses to anchor pose and scale";
        return report;
    }
    if (build.report.edges < options.minimum_edges) {
        report.message = "not enough edges for bundle adjustment";
        return report;
    }

    if (options.use_robust_kernel) {
        attachHuberKernels(*build.graph,
                           options.robust_policy.huber_delta);
    }

    g2o::SparseOptimizer& optimizer = *build.graph->optimizer;
    optimizer.initializeOptimization(0);
    optimizer.computeActiveErrors();
    report.initial_chi2 = optimizer.activeChi2();
    if (options.robust_iterations > 0) {
        report.iterations += optimizer.optimize(options.robust_iterations);
    }

    OutlierClassification classification = classifyOutliers(
        *build.graph, options.robust_policy, true);
    if (options.use_robust_kernel) {
        removeRobustKernels(*build.graph);
    }
    if (options.final_iterations > 0) {
        optimizer.initializeOptimization(0);
        report.iterations += optimizer.optimize(options.final_iterations);
    }
    optimizer.computeActiveErrors();
    report.final_chi2 = optimizer.activeChi2();
    classification = classifyOutliers(
        *build.graph, options.robust_policy, false);
    report.inliers = classification.inliers;
    report.outliers = classification.outliers;
    report.rejected = classification.rejected;

    if (report.iterations <= 0 || !std::isfinite(report.final_chi2) ||
        report.final_chi2 > report.initial_chi2 ||
        !estimateIsFinite(*build.graph)) {
        report.message = "BA result failed validation; map was not modified";
        return report;
    }
    for (const EdgeBinding& binding : build.graph->edge_bindings) {
        if (binding.edge->level() == 0 && !edgeDepthPositive(binding)) {
            report.message =
                "BA produced negative depth; map was not modified";
            return report;
        }
    }

    for (const auto& item : build.graph->pose_vertex_ids) {
        KeyFrame* keyframe = map.findKeyFrame(item.first);
        const auto* vertex = dynamic_cast<const g2o::VertexSE3Expmap*>(
            optimizer.vertex(item.second));
        writeG2OPose(vertex->estimate(), *keyframe);
    }
    for (const auto& item : build.graph->point_vertex_ids) {
        MapPoint* point = map.findMapPoint(item.first);
        const auto* vertex = dynamic_cast<const g2o::VertexPointXYZ*>(
            optimizer.vertex(item.second));
        const Eigen::Vector3d estimate = vertex->estimate();
        point->position_world =
            cv::Point3d(estimate.x(), estimate.y(), estimate.z());
    }

    report.success = true;
    report.message = "ok";
    return report;
}

}  // namespace mini_vo
