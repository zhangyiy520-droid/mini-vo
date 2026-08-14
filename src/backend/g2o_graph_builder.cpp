#include "mini_vo/backend/g2o_graph_builder.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace mini_vo {
namespace {

using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;
using LinearSolver = g2o::LinearSolverDense<BlockSolver::PoseMatrixType>;

std::unique_ptr<g2o::SparseOptimizer> makeOptimizer(GraphMode mode) {
    g2o::OptimizationAlgorithmLevenberg* algorithm = nullptr;
    if (mode == GraphMode::PointOnly) {
        using DynamicBlockSolver = g2o::BlockSolverX;
        using DynamicLinearSolver =
            g2o::LinearSolverDense<DynamicBlockSolver::PoseMatrixType>;
        auto linear_solver = std::make_unique<DynamicLinearSolver>();
        auto block_solver = std::make_unique<DynamicBlockSolver>(
            std::move(linear_solver));
        algorithm = new g2o::OptimizationAlgorithmLevenberg(
            std::move(block_solver));
    } else {
        auto linear_solver = std::make_unique<LinearSolver>();
        auto block_solver =
            std::make_unique<BlockSolver>(std::move(linear_solver));
        algorithm = new g2o::OptimizationAlgorithmLevenberg(
            std::move(block_solver));
    }

    auto optimizer = std::make_unique<g2o::SparseOptimizer>();
    optimizer->setAlgorithm(algorithm);
    optimizer->setVerbose(false);
    return optimizer;
}

bool keyframeSelected(KeyFrameId id,
                      const std::unordered_set<KeyFrameId>& selected_ids) {
    return selected_ids.empty() || selected_ids.count(id) != 0;
}

bool mapPointSelected(MapPointId id,
                      const std::unordered_set<MapPointId>& selected_ids) {
    return selected_ids.empty() || selected_ids.count(id) != 0;
}

Eigen::Vector2d project(const Eigen::Vector3d& point_camera,
                        const CameraIntrinsics& camera) {
    const double inverse_z = 1.0 / point_camera.z();
    return {camera.fx * point_camera.x() * inverse_z + camera.cx,
            camera.fy * point_camera.y() * inverse_z + camera.cy};
}

Eigen::Vector3d toEigen(const cv::Point3d& point) {
    return {point.x, point.y, point.z};
}

std::uint64_t observationKey(KeyFrameId keyframe_id,
                             MapPointId map_point_id) {
    return (static_cast<std::uint64_t>(keyframe_id) << 32U) ^
           static_cast<std::uint64_t>(map_point_id);
}

}  // namespace

EdgeReprojectionPoseOnly::EdgeReprojectionPoseOnly(
    const Eigen::Vector3d& point_world,
    const CameraIntrinsics& camera)
    : point_world_(point_world), camera_(camera) {}

void EdgeReprojectionPoseOnly::computeError() {
    const auto* pose =
        static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    const Eigen::Vector3d point_camera = pose->estimate().map(point_world_);
    if (point_camera.z() <= 1e-9) {
        _error = Eigen::Vector2d::Constant(1e3);
        return;
    }
    _error = _measurement - project(point_camera, camera_);
}

bool EdgeReprojectionPoseOnly::read(std::istream&) {
    return false;
}

bool EdgeReprojectionPoseOnly::write(std::ostream&) const {
    return false;
}

bool EdgeReprojectionPoseOnly::depthPositive() const {
    const auto* pose =
        static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    return pose->estimate().map(point_world_).z() > 1e-9;
}

EdgeReprojectionBA::EdgeReprojectionBA(const CameraIntrinsics& camera)
    : camera_(camera) {}

void EdgeReprojectionBA::computeError() {
    const auto* point = static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);
    const auto* pose =
        static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const Eigen::Vector3d point_camera = pose->estimate().map(point->estimate());
    if (point_camera.z() <= 1e-9) {
        _error = Eigen::Vector2d::Constant(1e3);
        return;
    }
    _error = _measurement - project(point_camera, camera_);
}

bool EdgeReprojectionBA::read(std::istream&) {
    return false;
}

bool EdgeReprojectionBA::write(std::ostream&) const {
    return false;
}

bool EdgeReprojectionBA::depthPositive() const {
    const auto* point = static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);
    const auto* pose =
        static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    return pose->estimate().map(point->estimate()).z() > 1e-9;
}

g2o::SE3Quat toG2OPose(const KeyFrame& keyframe) {
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    for (int row = 0; row < 3; ++row) {
        translation[row] = keyframe.tcw.at<double>(row, 0);
        for (int col = 0; col < 3; ++col) {
            rotation(row, col) = keyframe.Rcw.at<double>(row, col);
        }
    }
    return g2o::SE3Quat(rotation, translation);
}

void writeG2OPose(const g2o::SE3Quat& estimate, KeyFrame& keyframe) {
    const Eigen::Matrix3d rotation = estimate.rotation().toRotationMatrix();
    const Eigen::Vector3d translation = estimate.translation();
    keyframe.Rcw = cv::Mat(3, 3, CV_64F);
    keyframe.tcw = cv::Mat(3, 1, CV_64F);
    for (int row = 0; row < 3; ++row) {
        keyframe.tcw.at<double>(row, 0) = translation[row];
        for (int col = 0; col < 3; ++col) {
            keyframe.Rcw.at<double>(row, col) = rotation(row, col);
        }
    }
}

GraphBuildResult buildG2OGraph(const Map& map,
                               const CameraIntrinsics& camera,
                               const GraphBuildOptions& options) {
    GraphBuildResult result;
    if (!map.validate()) {
        result.report.message = "map validation failed";
        return result;
    }
    if (!camera.valid()) {
        result.report.message = "camera intrinsics are invalid";
        return result;
    }

    auto graph = std::make_unique<G2OGraph>();
    graph->optimizer = makeOptimizer(options.mode);

    const auto& observations = map.observations().all();
    std::unordered_set<KeyFrameId> keyframe_ids;
    std::unordered_set<MapPointId> point_ids;
    std::unordered_map<MapPointId, std::size_t> point_observation_count;
    std::unordered_set<std::uint64_t> pairs;

    for (std::size_t index = 0; index < observations.size(); ++index) {
        const Observation& observation = observations[index];
        if (observation.outlier ||
            !keyframeSelected(observation.keyframe_id,
                              options.keyframe_ids) ||
            !mapPointSelected(observation.map_point_id,
                              options.map_point_ids)) {
            ++result.report.rejected_observations;
            continue;
        }
        const KeyFrame* keyframe = map.findKeyFrame(observation.keyframe_id);
        const MapPoint* point = map.findMapPoint(observation.map_point_id);
        if (keyframe == nullptr || point == nullptr || point->bad) {
            ++result.report.rejected_observations;
            continue;
        }
        const std::uint64_t pair_key =
            observationKey(observation.keyframe_id, observation.map_point_id);
        if (!pairs.insert(pair_key).second) {
            result.report.message = "duplicate observation detected";
            return result;
        }
        keyframe_ids.insert(observation.keyframe_id);
        point_ids.insert(observation.map_point_id);
        ++point_observation_count[observation.map_point_id];
    }

    int next_vertex_id = 0;
    for (KeyFrameId id : keyframe_ids) {
        const KeyFrame* keyframe = map.findKeyFrame(id);
        auto* vertex = new g2o::VertexSE3Expmap();
        vertex->setId(next_vertex_id);
        vertex->setEstimate(toG2OPose(*keyframe));
        const bool fixed = options.mode == GraphMode::PointOnly ||
                           keyframe->fixed ||
                           options.fixed_keyframe_ids.count(id) != 0;
        vertex->setFixed(fixed);
        if (!graph->optimizer->addVertex(vertex)) {
            delete vertex;
            result.report.message = "failed to add pose vertex";
            return result;
        }
        graph->pose_vertex_ids.emplace(id, next_vertex_id++);
        ++result.report.pose_vertices;
        result.report.fixed_vertices += fixed ? 1U : 0U;
    }

    if (options.mode != GraphMode::PoseOnly) {
        for (MapPointId id : point_ids) {
            const MapPoint* point = map.findMapPoint(id);
            auto* vertex = new g2o::VertexPointXYZ();
            vertex->setId(next_vertex_id);
            vertex->setEstimate(toEigen(point->position_world));
            vertex->setMarginalized(options.mode ==
                                    GraphMode::FullBundleAdjustment);
            if (!graph->optimizer->addVertex(vertex)) {
                delete vertex;
                result.report.message = "failed to add point vertex";
                return result;
            }
            graph->point_vertex_ids.emplace(id, next_vertex_id++);
            ++result.report.point_vertices;
            if (point_observation_count[id] < 2U) {
                ++result.report.weak_points;
            }
        }
    }

    pairs.clear();
    for (std::size_t index = 0; index < observations.size(); ++index) {
        const Observation& observation = observations[index];
        if (observation.outlier ||
            graph->pose_vertex_ids.count(observation.keyframe_id) == 0 ||
            point_ids.count(observation.map_point_id) == 0) {
            continue;
        }
        const std::uint64_t pair_key =
            observationKey(observation.keyframe_id, observation.map_point_id);
        if (!pairs.insert(pair_key).second) {
            result.report.message = "duplicate observation detected";
            return result;
        }

        const MapPoint* point = map.findMapPoint(observation.map_point_id);
        const Eigen::Vector2d measurement(observation.pixel.x,
                                          observation.pixel.y);
        g2o::OptimizableGraph::Edge* raw_edge = nullptr;

        if (options.mode == GraphMode::PoseOnly) {
            auto* edge = new EdgeReprojectionPoseOnly(
                toEigen(point->position_world), camera);
            edge->setVertex(0, graph->optimizer->vertex(
                                   graph->pose_vertex_ids.at(
                                       observation.keyframe_id)));
            edge->setMeasurement(measurement);
            edge->setInformation(Eigen::Matrix2d::Identity());
            raw_edge = edge;
        } else {
            auto* edge = new EdgeReprojectionBA(camera);
            edge->setVertex(0, graph->optimizer->vertex(
                                   graph->point_vertex_ids.at(
                                       observation.map_point_id)));
            edge->setVertex(1, graph->optimizer->vertex(
                                   graph->pose_vertex_ids.at(
                                       observation.keyframe_id)));
            edge->setMeasurement(measurement);
            edge->setInformation(Eigen::Matrix2d::Identity());
            raw_edge = edge;
        }

        if (!graph->optimizer->addEdge(raw_edge)) {
            delete raw_edge;
            result.report.message = "failed to add reprojection edge";
            return result;
        }
        graph->edge_bindings.push_back(
            {index, observation.keyframe_id, observation.map_point_id,
             raw_edge});
        ++result.report.edges;
    }

    if (result.report.pose_vertices == 0 || result.report.edges == 0) {
        result.report.message = "graph has no usable pose or edge";
        return result;
    }
    result.report.success = true;
    result.report.message = "ok";
    result.graph = std::move(graph);
    return result;
}

bool edgeDepthPositive(const EdgeBinding& binding) {
    if (const auto* edge =
            dynamic_cast<const EdgeReprojectionPoseOnly*>(binding.edge)) {
        return edge->depthPositive();
    }
    if (const auto* edge =
            dynamic_cast<const EdgeReprojectionBA*>(binding.edge)) {
        return edge->depthPositive();
    }
    return false;
}

}  // namespace mini_vo
