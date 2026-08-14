#include "mini_vo/backend/backend.h"

#include <iostream>

namespace {

mini_vo::Map makeScene(const mini_vo::CameraIntrinsics& camera) {
    mini_vo::Map map;
    constexpr int kFrames = 3;
    constexpr int kPoints = 30;

    for (int frame_id = 0; frame_id < kFrames; ++frame_id) {
        mini_vo::KeyFrame keyframe;
        keyframe.id = frame_id;
        keyframe.Rcw = cv::Mat::eye(3, 3, CV_64F);
        keyframe.tcw = cv::Mat::zeros(3, 1, CV_64F);
        keyframe.tcw.at<double>(0, 0) = -0.2 * frame_id;
        if (frame_id == 2) {
            keyframe.tcw.at<double>(0, 0) += 0.08;
            keyframe.tcw.at<double>(1, 0) -= 0.04;
        }
        keyframe.keypoints.resize(kPoints);
        keyframe.descriptors = cv::Mat::zeros(kPoints, 32, CV_8U);
        map.addKeyFrame(keyframe);
    }

    for (int point_id = 0; point_id < kPoints; ++point_id) {
        const cv::Point3d truth(
            -1.0 + 0.06 * point_id,
            -0.3 + 0.15 * (point_id % 5),
            3.5 + 0.1 * (point_id % 9));
        mini_vo::MapPoint point;
        point.id = point_id;
        point.position_world = truth;
        point.descriptor = cv::Mat::zeros(1, 32, CV_8U);
        map.addMapPoint(point);

        for (int frame_id = 0; frame_id < kFrames; ++frame_id) {
            mini_vo::Observation observation;
            observation.keyframe_id = frame_id;
            observation.map_point_id = point_id;
            observation.feature_index = point_id;
            observation.pixel = cv::Point2f(
                static_cast<float>(
                    camera.fx * (truth.x - 0.2 * frame_id) /
                        truth.z +
                    camera.cx),
                static_cast<float>(
                    camera.fy * truth.y / truth.z + camera.cy));
            map.addObservation(observation);
        }
    }
    return map;
}

}  // namespace

int main() {
    const mini_vo::CameraIntrinsics camera{500.0, 500.0, 320.0, 240.0};
    mini_vo::Map map = makeScene(camera);

    const mini_vo::BackendReport report =
        mini_vo::Backend().processKeyFrame(map, 2, camera);
    const cv::Mat expected =
        (cv::Mat_<double>(3, 1) << -0.4, 0.0, 0.0);
    const double pose_error =
        cv::norm(map.findKeyFrame(2)->tcw - expected);
    if (!report.success || !report.pose_optimized ||
        !report.local_ba_optimized || pose_error > 1e-4) {
        std::cerr << "[FAIL] backend did not process the keyframe\n";
        return 1;
    }

    std::cout << "[PASS] backend pose=" << report.pose_optimized
              << " local_ba=" << report.local_ba_optimized
              << " chi2=" << report.initial_chi2
              << "->" << report.final_chi2
              << " pose_error=" << pose_error << '\n';
    return 0;
}
