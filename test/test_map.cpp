#include "mini_vo/core/map.h"

#include <iostream>
#include <string>

namespace {

bool require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "[FAIL] " << message << '\n';
    }
    return condition;
}

mini_vo::KeyFrame makeKeyFrame(mini_vo::KeyFrameId id) {
    mini_vo::KeyFrame keyframe;
    keyframe.id = id;
    keyframe.image = cv::Mat::zeros(480, 640, CV_8U);
    keyframe.keypoints.emplace_back(320.0f, 240.0f, 1.0f);
    keyframe.descriptors = cv::Mat::zeros(1, 32, CV_8U);
    keyframe.Rcw = cv::Mat::eye(3, 3, CV_64F);
    keyframe.tcw = cv::Mat::zeros(3, 1, CV_64F);
    return keyframe;
}

mini_vo::MapPoint makeMapPoint(mini_vo::MapPointId id,
                               const cv::Point3d& position,
                               unsigned char descriptor_value) {
    mini_vo::MapPoint point;
    point.id = id;
    point.position_world = position;
    point.descriptor = cv::Mat(1, 32, CV_8U,
                               cv::Scalar(descriptor_value));
    return point;
}

}  // namespace

int main() {
    mini_vo::Map map;
    mini_vo::KeyFrame first_keyframe = makeKeyFrame(1);
    first_keyframe.fixed = true;
    if (!require(map.addKeyFrame(first_keyframe), "add first keyframe") ||
        !require(!map.addKeyFrame(first_keyframe),
                 "reject duplicate keyframe")) {
        return 1;
    }

    if (!require(map.addMapPoint(
                     makeMapPoint(10, {0.0, 0.0, 5.0}, 0)),
                 "add first map point") ||
        !require(map.addMapPoint(
                     makeMapPoint(5, {1.0, 2.0, 6.0}, 1)),
                 "add second map point")) {
        return 1;
    }

    const mini_vo::TrackingMapSnapshot& snapshot = map.trackingSnapshot();
    if (!require(snapshot.valid(), "tracking snapshot is aligned") ||
        !require(snapshot.map_point_ids.size() == 2,
                 "tracking snapshot point count") ||
        !require(snapshot.map_point_ids[0] == 5 &&
                     snapshot.map_point_ids[1] == 10,
                 "tracking snapshot sorted ids") ||
        !require(snapshot.points[0] == cv::Point3f(1.0f, 2.0f, 6.0f),
                 "tracking snapshot position") ||
        !require(cv::countNonZero(snapshot.descriptors.row(0)) == 32 &&
                     cv::countNonZero(snapshot.descriptors.row(1)) == 0,
                 "tracking snapshot descriptor alignment")) {
        return 1;
    }

    const mini_vo::Observation first_observation{
        1, 10, 0, {320.0f, 240.0f}, 0, false};
    if (!require(map.addObservation(first_observation),
                 "add first observation") ||
        !require(map.validate(), "validate initial map") ||
        !require(map.keyFrameCount() == 1 && map.mapPointCount() == 2,
                 "initial map counts")) {
        return 1;
    }

    const mini_vo::KeyFrame second_keyframe = makeKeyFrame(2);
    const mini_vo::MapPoint third_point =
        makeMapPoint(20, {0.5, 0.2, 4.0}, 2);
    const std::vector<mini_vo::Observation> invalid_observations{
        {2, 20, 99, {320.0f, 240.0f}, 0, false}};
    if (!require(!map.insertKeyFrameBatch(
                     second_keyframe, {third_point}, invalid_observations),
                 "reject invalid batch") ||
        !require(map.keyFrameCount() == 1 && map.mapPointCount() == 2 &&
                     map.observations().size() == 1,
                 "invalid batch leaves map unchanged")) {
        return 1;
    }

    const std::vector<mini_vo::Observation> valid_observations{
        {1, 20, 0, {320.0f, 240.0f}, 0, false},
        {2, 20, 0, {320.0f, 240.0f}, 0, false}};
    if (!require(map.insertKeyFrameBatch(
                     second_keyframe, {third_point}, valid_observations),
                 "commit valid batch") ||
        !require(map.keyFrameCount() == 2 && map.mapPointCount() == 3 &&
                     map.observations().size() == 3,
                 "valid batch counts") ||
        !require(map.validate(), "validate committed batch")) {
        return 1;
    }

    const mini_vo::TrackingMapSnapshot& updated_snapshot =
        map.trackingSnapshot();
    if (!require(updated_snapshot.map_point_ids.size() == 3 &&
                     updated_snapshot.map_point_ids.back() == 20,
                 "tracking cache updated incrementally") ||
        !require(map.updateMapPointPosition(20, {2.0, 3.0, 7.0}),
                 "update map point through controlled writeback") ||
        !require(updated_snapshot.points.back() ==
                     cv::Point3f(2.0f, 3.0f, 7.0f),
                 "tracking cache receives optimized position")) {
        return 1;
    }

    if (!require(map.eraseMapPoint(10), "erase map point") ||
        !require(map.observations().size() == 2,
                 "erase linked observation") ||
        !require(map.validate(), "validate map after erase") ||
        !require(map.mapPointCount() == 2, "final map point count")) {
        return 1;
    }

    std::cout << "[PASS] map batch insertion and tracking cache\n";
    return 0;
}
