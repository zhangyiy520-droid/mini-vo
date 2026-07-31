#include "mini_vo/backend/robust_policy.h"

#include <iostream>

int main() {
    const mini_vo::CameraIntrinsics camera{500.0, 500.0, 320.0, 240.0};
    mini_vo::Map map;
    mini_vo::KeyFrame keyframe;
    keyframe.id = 0;
    keyframe.Rcw = cv::Mat::eye(3, 3, CV_64F);
    keyframe.tcw = cv::Mat::zeros(3, 1, CV_64F);
    keyframe.keypoints.resize(6);
    keyframe.descriptors = cv::Mat::zeros(6, 32, CV_8U);
    map.addKeyFrame(keyframe);

    for (int index = 0; index < 6; ++index) {
        mini_vo::MapPoint point;
        point.id = index;
        point.position_world = cv::Point3d(0.05 * index, 0.0, 4.0);
        point.descriptor = cv::Mat::zeros(1, 32, CV_8U);
        map.addMapPoint(point);

        mini_vo::Observation observation;
        observation.keyframe_id = 0;
        observation.map_point_id = index;
        observation.feature_index = index;
        observation.pixel = cv::Point2f(
            static_cast<float>(camera.fx * point.position_world.x /
                               point.position_world.z + camera.cx),
            240.0F);
        if (index == 5) {
            observation.pixel.x += 80.0F;
        }
        map.addObservation(observation);
    }

    mini_vo::GraphBuildOptions options;
    options.mode = mini_vo::GraphMode::PoseOnly;
    mini_vo::GraphBuildResult build =
        mini_vo::buildG2OGraph(map, camera, options);
    mini_vo::attachHuberKernels(*build.graph, 2.447651936);
    const mini_vo::OutlierClassification classification =
        mini_vo::classifyOutliers(
            *build.graph, mini_vo::RobustPolicyOptions{}, true);

    if (classification.inliers != 5 || classification.outliers != 1 ||
        classification.rejected.front().map_point_id != 5) {
        std::cerr << "[FAIL] robust outlier classification is incorrect\n";
        return 1;
    }
    std::cout << "[PASS] robust inliers=" << classification.inliers
              << " outliers=" << classification.outliers
              << " bad_chi2=" << classification.rejected.front().chi2
              << '\n';
    return 0;
}
