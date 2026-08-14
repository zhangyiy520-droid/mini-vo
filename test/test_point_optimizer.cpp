#include "mini_vo/backend/point_optimizer.h"

#include <iostream>

int main() {
    const mini_vo::CameraIntrinsics camera{500.0, 500.0, 320.0, 240.0};
    mini_vo::Map map;
    const cv::Point3d truth(0.25, -0.1, 4.0);

    for (std::uint64_t frame_id = 0; frame_id < 3; ++frame_id) {
        mini_vo::KeyFrame keyframe;
        keyframe.id = frame_id;
        keyframe.Rcw = cv::Mat::eye(3, 3, CV_64F);
        keyframe.tcw = cv::Mat::zeros(3, 1, CV_64F);
        keyframe.tcw.at<double>(0, 0) = -0.2 * frame_id;
        keyframe.keypoints.emplace_back(cv::Point2f(), 1.0F);
        keyframe.descriptors = cv::Mat::zeros(1, 32, CV_8U);
        map.addKeyFrame(keyframe);
    }

    mini_vo::MapPoint point;
    point.id = 9;
    point.position_world = truth + cv::Point3d(0.3, -0.2, 0.4);
    point.descriptor = cv::Mat::zeros(1, 32, CV_8U);
    map.addMapPoint(point);

    for (std::uint64_t frame_id = 0; frame_id < 3; ++frame_id) {
        const double x_camera = truth.x - 0.2 * frame_id;
        mini_vo::Observation observation;
        observation.keyframe_id = frame_id;
        observation.map_point_id = point.id;
        observation.feature_index = 0;
        observation.pixel = cv::Point2f(
            static_cast<float>(camera.fx * x_camera / truth.z + camera.cx),
            static_cast<float>(camera.fy * truth.y / truth.z + camera.cy));
        map.addObservation(observation);
    }

    const mini_vo::PointOptimizationReport report =
        mini_vo::PointOptimizer().optimize(map, point.id, camera);
    const cv::Point3d estimate = map.findMapPoint(point.id)->position_world;
    const double error = cv::norm(estimate - truth);
    if (!report.success || report.final_chi2 >= report.initial_chi2 ||
        error > 1e-5) {
        std::cerr << "[FAIL] point optimization did not recover Pw\n";
        return 1;
    }

    mini_vo::Map single_view_map;
    mini_vo::KeyFrame single_keyframe;
    single_keyframe.id = 0;
    single_keyframe.Rcw = cv::Mat::eye(3, 3, CV_64F);
    single_keyframe.tcw = cv::Mat::zeros(3, 1, CV_64F);
    single_keyframe.keypoints.emplace_back(cv::Point2f(), 1.0F);
    single_keyframe.descriptors = cv::Mat::zeros(1, 32, CV_8U);
    single_view_map.addKeyFrame(single_keyframe);

    mini_vo::MapPoint single_view_point;
    single_view_point.id = 10;
    single_view_point.position_world =
        truth + cv::Point3d(0.3, -0.2, 0.4);
    single_view_point.descriptor = cv::Mat::zeros(1, 32, CV_8U);
    const cv::Point3d original_position = single_view_point.position_world;
    single_view_map.addMapPoint(single_view_point);

    mini_vo::Observation single_observation;
    single_observation.keyframe_id = single_keyframe.id;
    single_observation.map_point_id = single_view_point.id;
    single_observation.feature_index = 0;
    single_observation.pixel = cv::Point2f(
        static_cast<float>(camera.fx * truth.x / truth.z + camera.cx),
        static_cast<float>(camera.fy * truth.y / truth.z + camera.cy));
    single_view_map.addObservation(single_observation);

    const mini_vo::PointOptimizationReport rejected_report =
        mini_vo::PointOptimizer().optimize(
            single_view_map, single_view_point.id, camera);
    const cv::Point3d rejected_position =
        single_view_map.findMapPoint(single_view_point.id)->position_world;
    if (rejected_report.success ||
        rejected_report.message != "point needs at least two observations" ||
        cv::norm(rejected_position - original_position) != 0.0) {
        std::cerr << "[FAIL] single-view point was not rejected cleanly\n";
        return 1;
    }

    std::cout << "[PASS] point chi2 " << report.initial_chi2
              << " -> " << report.final_chi2
              << " position_error=" << error
              << " single_view_rejected=true\n";
    return 0;
}
