#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "mini_vo/core/observation.h"

namespace mini_vo {

struct KeyFrame {
    KeyFrameId id = 0;
    double timestamp = 0.0;
    cv::Mat image;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Mat Rcw;
    cv::Mat tcw;
    bool fixed = false;
};

struct MapPoint {
    MapPointId id = 0;
    cv::Point3d position_world{};
    cv::Mat descriptor;
    bool bad = false;
};

class Map {
public:
    bool addKeyFrame(const KeyFrame& keyframe);
    bool addMapPoint(const MapPoint& map_point);
    bool addObservation(const Observation& observation);

    KeyFrame* findKeyFrame(KeyFrameId id);
    const KeyFrame* findKeyFrame(KeyFrameId id) const;
    MapPoint* findMapPoint(MapPointId id);
    const MapPoint* findMapPoint(MapPointId id) const;

    bool eraseMapPoint(MapPointId id);

    const ObservationStore& observations() const;
    std::size_t keyFrameCount() const;
    std::size_t mapPointCount() const;
    bool validate() const;

private:
    std::unordered_map<KeyFrameId, KeyFrame> keyframes_;
    std::unordered_map<MapPointId, MapPoint> map_points_;
    ObservationStore observations_;
};

}  // namespace mini_vo
