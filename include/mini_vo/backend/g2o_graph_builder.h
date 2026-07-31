#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "mini_vo/core/map.h"

namespace mini_vo {

struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;

    bool valid() const;
};

enum class GraphMode {
    PoseOnly,
    PointOnly,
    FullBundleAdjustment
};

struct GraphBuildOptions {
    GraphMode mode = GraphMode::FullBundleAdjustment;
    std::unordered_set<KeyFrameId> keyframe_ids;
    std::unordered_set<MapPointId> map_point_ids;
    std::unordered_set<KeyFrameId> fixed_keyframe_ids;
};

struct GraphBuildReport {
    std::size_t pose_vertices = 0;
    std::size_t point_vertices = 0;
    std::size_t fixed_vertices = 0;
    std::size_t edges = 0;
    std::size_t rejected_observations = 0;
    std::size_t weak_points = 0;
    bool success = false;
    std::string message;
};

class EdgeReprojectionPoseOnly final
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d,
                                g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeReprojectionPoseOnly(const Eigen::Vector3d& point_world,
                             const CameraIntrinsics& camera);

    void computeError() override;
    bool read(std::istream&) override;
    bool write(std::ostream&) const override;
    bool depthPositive() const;

private:
    Eigen::Vector3d point_world_ = Eigen::Vector3d::Zero();
    CameraIntrinsics camera_;
};

class EdgeReprojectionBA final
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d,
                                 g2o::VertexPointXYZ,
                                 g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit EdgeReprojectionBA(const CameraIntrinsics& camera);

    void computeError() override;
    bool read(std::istream&) override;
    bool write(std::ostream&) const override;
    bool depthPositive() const;

private:
    CameraIntrinsics camera_;
};

struct EdgeBinding {
    std::size_t observation_index = 0;
    KeyFrameId keyframe_id = 0;
    MapPointId map_point_id = 0;
    g2o::OptimizableGraph::Edge* edge = nullptr;
};

struct G2OGraph {
    std::unique_ptr<g2o::SparseOptimizer> optimizer;
    std::unordered_map<KeyFrameId, int> pose_vertex_ids;
    std::unordered_map<MapPointId, int> point_vertex_ids;
    std::vector<EdgeBinding> edge_bindings;
};

struct GraphBuildResult {
    std::unique_ptr<G2OGraph> graph;
    GraphBuildReport report;
};

GraphBuildResult buildG2OGraph(const Map& map,
                               const CameraIntrinsics& camera,
                               const GraphBuildOptions& options);

g2o::SE3Quat toG2OPose(const KeyFrame& keyframe);
void writeG2OPose(const g2o::SE3Quat& estimate, KeyFrame& keyframe);
bool edgeDepthPositive(const EdgeBinding& binding);

}  // namespace mini_vo
