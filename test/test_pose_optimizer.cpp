#include "mini_vo/backend/pose_optimizer.h"

#include <cmath>
#include <iostream>

namespace {

mini_vo::Map makePoseScene(const mini_vo::CameraIntrinsics& camera) {
    mini_vo::Map map;
    mini_vo::KeyFrame keyframe;
    keyframe.id = 7;
    keyframe.Rcw = cv::Mat::eye(3, 3, CV_64F);
    keyframe.tcw = cv::Mat::zeros(3, 1, CV_64F);
    keyframe.tcw.at<double>(0, 0) = 0.15;

    constexpr int kPointCount = 30;
    for (int index = 0; index < kPointCount; ++index) {
        keyframe.keypoints.emplace_back(
            cv::Point2f(static_cast<float>(index), 0.0F), 1.0F);
    }
    keyframe.descriptors = cv::Mat::zeros(kPointCount, 32, CV_8U);
    if (!map.addKeyFrame(keyframe)) {
        throw std::runtime_error("failed to add keyframe");
    }

    for (int index = 0; index < kPointCount; ++index) {
        mini_vo::MapPoint point;
        point.id = index;
        point.position_world = cv::Point3d(
            -1.0 + 0.08 * index,
            -0.4 + 0.2 * (index % 5),
            3.5 + 0.15 * (index % 7));
        point.descriptor = cv::Mat::zeros(1, 32, CV_8U);
        map.addMapPoint(point);

        mini_vo::Observation observation;
        observation.keyframe_id = keyframe.id;
        observation.map_point_id = point.id;
        observation.feature_index = index;
        observation.pixel = cv::Point2f(
            static_cast<float>(camera.fx * point.position_world.x /
                               point.position_world.z + camera.cx),
            static_cast<float>(camera.fy * point.position_world.y /
                               point.position_world.z + camera.cy));
        map.addObservation(observation);
    }
    return map;
}

}  // namespace

int main() {
    const mini_vo::CameraIntrinsics camera{520.0, 515.0, 320.0, 240.0};
    mini_vo::Map map = makePoseScene(camera);

    const mini_vo::PoseOptimizationReport report =
        mini_vo::PoseOptimizer().optimize(map, 7, camera);
    const mini_vo::KeyFrame* optimized = map.findKeyFrame(7);
    const double translation_error = cv::norm(optimized->tcw);

    if (!report.success || report.final_chi2 >= report.initial_chi2 ||
        translation_error > 1e-4 || report.inliers != 30) {
        std::cerr << "[FAIL] pose optimization did not recover Tcw\n";
        return 1;
    }

    std::cout << "[PASS] pose chi2 " << report.initial_chi2
              << " -> " << report.final_chi2
              << " translation_error=" << translation_error
              << " inliers=" << report.inliers << '\n';
    return 0;
}
