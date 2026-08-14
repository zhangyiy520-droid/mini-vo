#include "mini_vo/core/map.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mini_vo {

namespace {

bool isFinitePoint(const cv::Point3d& point) {
    return std::isfinite(point.x) &&
           std::isfinite(point.y) &&
           std::isfinite(point.z);
}

bool isPoseShapeValid(const cv::Mat& Rcw, const cv::Mat& tcw) {
    return Rcw.rows == 3 && Rcw.cols == 3 &&
           tcw.rows == 3 && tcw.cols == 1;
}

}  // namespace

bool TrackingMapSnapshot::valid() const {
    return map_point_ids.size() == points.size() &&
           descriptors.rows == static_cast<int>(points.size());
}

bool Map::addKeyFrame(const KeyFrame& input) {
    if (keyframes_.count(input.id) != 0 ||
        input.keypoints.empty() ||
        input.descriptors.empty() ||
        input.descriptors.rows != static_cast<int>(input.keypoints.size()) ||
        !isPoseShapeValid(input.Rcw, input.tcw)) {
        return false;
    }

    KeyFrame keyframe = input;
    keyframe.image = input.image.clone();
    keyframe.descriptors = input.descriptors.clone();
    keyframe.Rcw = input.Rcw.clone();
    keyframe.tcw = input.tcw.clone();
    return keyframes_.emplace(keyframe.id, std::move(keyframe)).second;
}

bool Map::addMapPoint(const MapPoint& input) {
    if (map_points_.count(input.id) != 0 ||
        !isFinitePoint(input.position_world) ||
        input.descriptor.empty() || input.descriptor.rows != 1) {
        return false;
    }

    MapPoint map_point = input;
    map_point.descriptor = input.descriptor.clone();
    return map_points_.emplace(map_point.id, std::move(map_point)).second;
}

bool Map::addObservation(const Observation& observation) {
    const KeyFrame* keyframe = findKeyFrame(observation.keyframe_id);
    const MapPoint* map_point = findMapPoint(observation.map_point_id);
    if (keyframe == nullptr || map_point == nullptr || map_point->bad) {
        return false;
    }
    if (observation.feature_index >= keyframe->keypoints.size()) {
        return false;
    }
    if (!keyframe->image.empty()) {
        if (observation.pixel.x < 0.0f || observation.pixel.y < 0.0f ||
            observation.pixel.x >= keyframe->image.cols ||
            observation.pixel.y >= keyframe->image.rows) {
            return false;
        }
    }
    return observations_.add(observation);
}

KeyFrame* Map::findKeyFrame(KeyFrameId id) {
    const auto it = keyframes_.find(id);
    return it == keyframes_.end() ? nullptr : &it->second;
}

const KeyFrame* Map::findKeyFrame(KeyFrameId id) const {
    const auto it = keyframes_.find(id);
    return it == keyframes_.end() ? nullptr : &it->second;
}

MapPoint* Map::findMapPoint(MapPointId id) {
    const auto it = map_points_.find(id);
    return it == map_points_.end() ? nullptr : &it->second;
}

const MapPoint* Map::findMapPoint(MapPointId id) const {
    const auto it = map_points_.find(id);
    return it == map_points_.end() ? nullptr : &it->second;
}

bool Map::eraseMapPoint(MapPointId id) {
    if (map_points_.erase(id) == 0) {
        return false;
    }
    const auto linked = observations_.byMapPoint(id);
    for (const auto& observation : linked) {
        observations_.erase(observation.keyframe_id, observation.map_point_id);
    }
    return true;
}

bool Map::setObservationOutlier(KeyFrameId keyframe_id,
                                MapPointId map_point_id,
                                bool outlier) {
    return observations_.setOutlier(keyframe_id, map_point_id, outlier);
}

const ObservationStore& Map::observations() const {
    return observations_;
}

TrackingMapSnapshot Map::trackingSnapshot() const {
    TrackingMapSnapshot snapshot;
    std::vector<MapPointId> ids;
    ids.reserve(map_points_.size());
    for (const auto& item : map_points_) {
        if (!item.second.bad) {
            ids.push_back(item.first);
        }
    }
    std::sort(ids.begin(), ids.end());

    snapshot.map_point_ids.reserve(ids.size());
    snapshot.points.reserve(ids.size());
    for (MapPointId id : ids) {
        const MapPoint& point = map_points_.at(id);
        snapshot.map_point_ids.push_back(id);
        snapshot.points.emplace_back(
            static_cast<float>(point.position_world.x),
            static_cast<float>(point.position_world.y),
            static_cast<float>(point.position_world.z));
        snapshot.descriptors.push_back(point.descriptor);
    }
    return snapshot;
}

std::size_t Map::keyFrameCount() const {
    return keyframes_.size();
}

std::size_t Map::mapPointCount() const {
    return map_points_.size();
}

bool Map::validate() const {
    if (!observations_.validate()) {
        return false;
    }
    for (const auto& observation : observations_.all()) {
        const KeyFrame* keyframe = findKeyFrame(observation.keyframe_id);
        const MapPoint* map_point = findMapPoint(observation.map_point_id);
        if (keyframe == nullptr || map_point == nullptr || map_point->bad ||
            observation.feature_index >= keyframe->keypoints.size()) {
            return false;
        }
    }
    return true;
}

}  // namespace mini_vo
