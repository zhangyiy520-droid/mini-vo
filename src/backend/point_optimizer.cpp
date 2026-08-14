#include "mini_vo/backend/point_optimizer.h"

#include <cmath>

#include <g2o/types/sba/types_six_dof_expmap.h>

namespace mini_vo {

PointOptimizationReport PointOptimizer::optimize(
    Map& map,
    MapPointId map_point_id,
    const CameraIntrinsics& camera,
    const PointOptimizationOptions& options) const {
    PointOptimizationReport report;
    const MapPoint* map_point = map.findMapPoint(map_point_id);
    if (map_point == nullptr || map_point->bad) {
        report.message = "map point not found or marked bad";
        return report;
    }

    GraphBuildOptions build_options;
    build_options.mode = GraphMode::PointOnly;
    build_options.map_point_ids.insert(map_point_id);
    GraphBuildResult build =
        buildG2OGraph(map, camera, build_options);
    if (!build.report.success || build.graph == nullptr) {
        report.message = build.report.message;
        return report;
    }
    if (build.report.edges < options.minimum_observations) {
        report.message = "point needs at least two observations";
        return report;
    }

    g2o::SparseOptimizer& optimizer = *build.graph->optimizer;
    optimizer.initializeOptimization(0);
    optimizer.computeActiveErrors();
    report.initial_chi2 = optimizer.activeChi2();
    report.iterations = optimizer.optimize(options.max_iterations);
    optimizer.computeActiveErrors();
    report.final_chi2 = optimizer.activeChi2();

    const int vertex_id = build.graph->point_vertex_ids.at(map_point_id);
    const auto* point_vertex = dynamic_cast<const g2o::VertexPointXYZ*>(
        optimizer.vertex(vertex_id));
    if (point_vertex == nullptr || report.iterations <= 0 ||
        !point_vertex->estimate().allFinite() ||
        !std::isfinite(report.final_chi2) ||
        report.final_chi2 > report.initial_chi2) {
        report.message = "optimizer returned an invalid point";
        return report;
    }
    for (const EdgeBinding& binding : build.graph->edge_bindings) {
        if (!edgeDepthPositive(binding)) {
            report.message = "optimized point has negative depth";
            return report;
        }
    }

    const Eigen::Vector3d estimate = point_vertex->estimate();
    if (!map.updateMapPointPosition(
            map_point_id,
            cv::Point3d(estimate.x(), estimate.y(), estimate.z()))) {
        report.message = "optimized point could not be written back";
        return report;
    }
    report.success = true;
    report.message = "ok";
    return report;
}

}  // namespace mini_vo
