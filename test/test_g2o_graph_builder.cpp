#include "mini_vo/backend/g2o_graph_builder.h"

#include <cmath>
#include <iostream>

namespace {

mini_vo::Map makeMap(const mini_vo::CameraIntrinsics& camera) {
    mini_vo::Map map;
    for (std::uint64_t frame_id = 0; frame_id < 2; ++frame_id) {
        mini_vo::KeyFrame keyframe;
        keyframe.id = frame_id;
        keyframe.Rcw = cv::Mat::eye(3, 3, CV_64F);
        keyframe.tcw = cv::Mat::zeros(3, 1, CV_64F);
        keyframe.tcw.at<double>(0, 0) = -0.2 * frame_id;
        keyframe.fixed = frame_id == 0;
        for (int point_index = 0; point_index < 10; ++point_index) {
            keyframe.keypoints.emplace_back(
                cv::Point2f(static_cast<float>(point_index), 0.0F), 1.0F);
        }
        keyframe.descriptors = cv::Mat::zeros(10, 32, CV_8U);
        if (!map.addKeyFrame(keyframe)) {
            throw std::runtime_error("failed to add keyframe");
        }
    }

    for (std::uint64_t point_id = 0; point_id < 10; ++point_id) {
        mini_vo::MapPoint point;
        point.id = point_id;
        point.position_world =
            cv::Point3d(-0.45 + 0.1 * point_id, 0.05, 4.0);
        point.descriptor = cv::Mat::zeros(1, 32, CV_8U);
        if (!map.addMapPoint(point)) {
            throw std::runtime_error("failed to add point");
        }

        for (std::uint64_t frame_id = 0; frame_id < 2; ++frame_id) {
            const double x_camera = point.position_world.x - 0.2 * frame_id;
            mini_vo::Observation observation;
            observation.keyframe_id = frame_id;
            observation.map_point_id = point_id;
            observation.feature_index = point_id;
            observation.pixel = cv::Point2f(
                static_cast<float>(camera.fx * x_camera /
                                   point.position_world.z + camera.cx),
                static_cast<float>(camera.fy * point.position_world.y /
                                   point.position_world.z + camera.cy));
            if (!map.addObservation(observation)) {
                throw std::runtime_error("failed to add observation");
            }
        }
    }
    return map;
}

}  // namespace

int main() {
    const mini_vo::CameraIntrinsics camera{500.0, 500.0, 320.0, 240.0};
    const mini_vo::Map map = makeMap(camera);

    mini_vo::GraphBuildOptions options;
    options.mode = mini_vo::GraphMode::FullBundleAdjustment;
    options.fixed_keyframe_ids.insert(0);
    const mini_vo::GraphBuildResult result =
        mini_vo::buildG2OGraph(map, camera, options);

    if (!result.report.success || result.report.pose_vertices != 2 ||
        result.report.point_vertices != 10 || result.report.edges != 20 ||
        result.report.fixed_vertices != 1 || result.report.weak_points != 0) {
        std::cerr << "[FAIL] graph counts are incorrect\n";
        return 1;
    }

    std::cout << "[PASS] poses=" << result.report.pose_vertices
              << " points=" << result.report.point_vertices
              << " edges=" << result.report.edges
              << " fixed=" << result.report.fixed_vertices
              << " weak=" << result.report.weak_points << '\n';
    return 0;
}
