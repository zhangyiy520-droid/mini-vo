#include "mini_vo/backend/local_bundle_adjuster.h"

#include <iostream>

namespace {

mini_vo::Map makeScene(const mini_vo::CameraIntrinsics& camera) {
    mini_vo::Map map;
    constexpr int kFrames = 4;
    constexpr int kPoints = 30;
    for (int frame_id = 0; frame_id < kFrames; ++frame_id) {
        mini_vo::KeyFrame keyframe;
        keyframe.id = frame_id;
        keyframe.Rcw = cv::Mat::eye(3, 3, CV_64F);
        keyframe.tcw = cv::Mat::zeros(3, 1, CV_64F);
        keyframe.tcw.at<double>(0, 0) = -0.15 * frame_id;
        if (frame_id == 3) {
            keyframe.tcw.at<double>(1, 0) = 0.05;
        }
        keyframe.keypoints.resize(kPoints);
        keyframe.descriptors = cv::Mat::zeros(kPoints, 32, CV_8U);
        map.addKeyFrame(keyframe);
    }

    for (int point_id = 0; point_id < kPoints; ++point_id) {
        const cv::Point3d truth(-0.8 + 0.06 * point_id,
                                -0.25 + 0.1 * (point_id % 6),
                                3.5 + 0.12 * (point_id % 8));
        mini_vo::MapPoint point;
        point.id = point_id;
        point.position_world = truth + cv::Point3d(0.02, -0.01, 0.03);
        point.descriptor = cv::Mat::zeros(1, 32, CV_8U);
        map.addMapPoint(point);

        for (int frame_id = 0; frame_id < kFrames; ++frame_id) {
            const double x_camera = truth.x - 0.15 * frame_id;
            mini_vo::Observation observation;
            observation.keyframe_id = frame_id;
            observation.map_point_id = point_id;
            observation.feature_index = point_id;
            observation.pixel = cv::Point2f(
                static_cast<float>(camera.fx * x_camera / truth.z +
                                   camera.cx),
                static_cast<float>(camera.fy * truth.y / truth.z +
                                   camera.cy));
            if (frame_id == 3 && point_id == 29) {
                observation.pixel.x += 80.0F;
            }
            map.addObservation(observation);
        }
    }
    return map;
}

}  // namespace

int main() {
    const mini_vo::CameraIntrinsics camera{500.0, 500.0, 320.0, 240.0};
    mini_vo::Map map = makeScene(camera);

    mini_vo::LocalBundleAdjustmentOptions disabled;
    disabled.enabled = false;
    const cv::Mat pose_before = map.findKeyFrame(3)->tcw.clone();
    const auto skipped = mini_vo::LocalBundleAdjuster().optimize(
        map, 3, camera, disabled);
    if (!skipped.success || !skipped.skipped ||
        cv::norm(map.findKeyFrame(3)->tcw - pose_before) != 0.0) {
        std::cerr << "[FAIL] disabled Local BA modified the map\n";
        return 1;
    }

    mini_vo::LocalBundleAdjustmentOptions options;
    options.window.max_local_keyframes = 3;
    options.window.minimum_shared_points = 5;
    options.optimizer.minimum_edges = 20;
    const auto report = mini_vo::LocalBundleAdjuster().optimize(
        map, 3, camera, options);
    const cv::Mat expected =
        (cv::Mat_<double>(3, 1) << -0.45, 0.0, 0.0);
    const double pose_error = cv::norm(map.findKeyFrame(3)->tcw - expected);
    const auto point_observations = map.observations().byMapPoint(29);
    bool outlier_marked = false;
    for (const auto& observation : point_observations) {
        if (observation.keyframe_id == 3 && observation.outlier) {
            outlier_marked = true;
        }
    }

    if (!report.success || pose_error > 1e-3 || !outlier_marked ||
        report.marked_outliers == 0) {
        std::cerr << "[FAIL] Local BA optimize/commit is incorrect"
                  << " success=" << report.success
                  << " message=" << report.message
                  << " pose_error=" << pose_error
                  << " outlier_marked=" << outlier_marked
                  << " marked_outliers=" << report.marked_outliers
                  << " ba_initial=" << report.optimizer.initial_chi2
                  << " ba_final=" << report.optimizer.final_chi2
                  << " tcw=" << map.findKeyFrame(3)->tcw.t()
                  << '\n';
        for (const auto& rejected : report.optimizer.rejected) {
            std::cerr << "  rejected kf=" << rejected.keyframe_id
                      << " mp=" << rejected.map_point_id
                      << " chi2=" << rejected.chi2 << '\n';
        }
        return 1;
    }

    std::cout << "[PASS] LocalBA pose_error=" << pose_error
              << " local_kf=" << report.local_keyframes
              << " fixed_kf=" << report.fixed_keyframes
              << " marked_outliers=" << report.marked_outliers << '\n';
    return 0;
}
