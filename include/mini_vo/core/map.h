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

struct TrackingMapSnapshot {
    std::vector<MapPointId> map_point_ids;
    std::vector<cv::Point3f> points;
    cv::Mat descriptors;

    bool valid() const;
};

class Map {
public:
    Map() = default;
    Map(const Map&) = delete;
    Map& operator=(const Map&) = delete;
    Map(Map&&) = default;
    Map& operator=(Map&&) = default;

    bool addKeyFrame(const KeyFrame& keyframe);
    bool addMapPoint(const MapPoint& map_point);
    bool addObservation(const Observation& observation);
    bool insertKeyFrameBatch(const KeyFrame& keyframe,
                             const std::vector<MapPoint>& map_points,
                             const std::vector<Observation>& observations);

    KeyFrame* findKeyFrame(KeyFrameId id);
    const KeyFrame* findKeyFrame(KeyFrameId id) const;
    const MapPoint* findMapPoint(MapPointId id) const;
    bool updateMapPointPosition(MapPointId id,
                                const cv::Point3d& position_world);

    bool eraseMapPoint(MapPointId id);
    bool setObservationOutlier(KeyFrameId keyframe_id,
                               MapPointId map_point_id,
                               bool outlier);

    const ObservationStore& observations() const;
    const TrackingMapSnapshot& trackingSnapshot() const;
    std::size_t keyFrameCount() const;
    std::size_t mapPointCount() const;
    bool validate() const;

private:
    void appendTrackingPoint(const MapPoint& map_point) const;
    void rebuildTrackingSnapshot() const;

    std::unordered_map<KeyFrameId, KeyFrame> keyframes_;
    std::unordered_map<MapPointId, MapPoint> map_points_;
    ObservationStore observations_;
    mutable TrackingMapSnapshot tracking_snapshot_;
    mutable std::unordered_map<MapPointId, std::size_t> tracking_indices_;
    mutable bool tracking_snapshot_valid_ = true;
};

}  // namespace mini_vo
