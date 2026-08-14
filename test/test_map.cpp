#include "mini_vo/core/map.h"

#include <cassert>
#include <iostream>

int main() {
    mini_vo::Map map;

    mini_vo::KeyFrame keyframe;
    keyframe.id = 1;
    keyframe.image = cv::Mat::zeros(480, 640, CV_8U);
    keyframe.keypoints.emplace_back(320.0f, 240.0f, 1.0f);
    keyframe.descriptors = cv::Mat::zeros(1, 32, CV_8U);
    keyframe.Rcw = cv::Mat::eye(3, 3, CV_64F);
    keyframe.tcw = cv::Mat::zeros(3, 1, CV_64F);
    keyframe.fixed = true;
    assert(map.addKeyFrame(keyframe));
    assert(!map.addKeyFrame(keyframe));

    mini_vo::MapPoint map_point;
    map_point.id = 10;
    map_point.position_world = {0.0, 0.0, 5.0};
    map_point.descriptor = cv::Mat::zeros(1, 32, CV_8U);
    assert(map.addMapPoint(map_point));

    mini_vo::MapPoint second_point;
    second_point.id = 5;
    second_point.position_world = {1.0, 2.0, 6.0};
    second_point.descriptor = cv::Mat::ones(1, 32, CV_8U);
    assert(map.addMapPoint(second_point));

    const mini_vo::TrackingMapSnapshot snapshot = map.trackingSnapshot();
    assert(snapshot.valid());
    assert(snapshot.map_point_ids.size() == 2);
    assert(snapshot.map_point_ids[0] == 5);
    assert(snapshot.map_point_ids[1] == 10);
    assert(snapshot.points[0] == cv::Point3f(1.0f, 2.0f, 6.0f));
    assert(cv::countNonZero(snapshot.descriptors.row(0)) == 32);
    assert(cv::countNonZero(snapshot.descriptors.row(1)) == 0);

    const mini_vo::Observation observation{1, 10, 0, {320.0f, 240.0f}, 0, false};
    assert(map.addObservation(observation));
    assert(map.validate());
    assert(map.keyFrameCount() == 1);
    assert(map.mapPointCount() == 1);

    assert(map.eraseMapPoint(10));
    assert(map.observations().size() == 0);
    assert(map.validate());
    assert(map.mapPointCount() == 1);

    std::cout << "[PASS] map ownership and links\n";
    return 0;
}
