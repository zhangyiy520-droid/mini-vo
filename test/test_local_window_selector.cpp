#include "mini_vo/backend/local_window_selector.h"

#include <iostream>
#include <unordered_set>

int main() {
    mini_vo::Map map;
    for (int frame_id = 0; frame_id < 4; ++frame_id) {
        mini_vo::KeyFrame keyframe;
        keyframe.id = frame_id;
        keyframe.Rcw = cv::Mat::eye(3, 3, CV_64F);
        keyframe.tcw = cv::Mat::zeros(3, 1, CV_64F);
        keyframe.keypoints.resize(12);
        keyframe.descriptors = cv::Mat::zeros(12, 32, CV_8U);
        map.addKeyFrame(keyframe);
    }
    for (int point_id = 0; point_id < 12; ++point_id) {
        mini_vo::MapPoint point;
        point.id = point_id;
        point.position_world = cv::Point3d(0.1 * point_id, 0.0, 4.0);
        point.descriptor = cv::Mat::zeros(1, 32, CV_8U);
        map.addMapPoint(point);
    }

    const std::vector<std::vector<int>> visible{
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
        {0, 1, 2, 3, 4, 5, 6, 7},
        {6, 7, 8, 9, 10, 11},
        {10, 11}};
    for (int frame_id = 0; frame_id < 4; ++frame_id) {
        for (int point_id : visible[frame_id]) {
            mini_vo::Observation observation;
            observation.keyframe_id = frame_id;
            observation.map_point_id = point_id;
            observation.feature_index = point_id;
            observation.pixel = cv::Point2f(100.0F + point_id, 120.0F);
            map.addObservation(observation);
        }
    }

    mini_vo::LocalWindowOptions options;
    options.max_local_keyframes = 2;
    options.minimum_shared_points = 5;
    const mini_vo::LocalBAWindow window =
        mini_vo::LocalWindowSelector().select(map, 0, options);

    const std::unordered_set<mini_vo::KeyFrameId> local(
        window.local_keyframes.begin(), window.local_keyframes.end());
    const std::unordered_set<mini_vo::KeyFrameId> expected_local{0, 1};
    if (!window.success || local != expected_local ||
        window.local_map_points.size() != 10 ||
        window.fixed_keyframes != std::vector<mini_vo::KeyFrameId>{2}) {
        std::cerr << "[FAIL] local BA window is incorrect\n";
        return 1;
    }

    std::cout << "[PASS] local_kf=" << window.local_keyframes.size()
              << " points=" << window.local_map_points.size()
              << " fixed_kf=" << window.fixed_keyframes.size() << '\n';
    return 0;
}
