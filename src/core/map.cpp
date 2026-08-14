#include "mini_vo/core/map.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
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

bool isKeyFrameValid(const KeyFrame& keyframe) {
    return !keyframe.keypoints.empty() &&
           !keyframe.descriptors.empty() &&
           keyframe.descriptors.rows ==
               static_cast<int>(keyframe.keypoints.size()) &&
           isPoseShapeValid(keyframe.Rcw, keyframe.tcw);
}

bool isMapPointValid(const MapPoint& map_point) {
    return isFinitePoint(map_point.position_world) &&
           !map_point.descriptor.empty() &&
           map_point.descriptor.rows == 1;
}

bool isObservationValidForKeyFrame(const Observation& observation,
                                   const KeyFrame& keyframe) {
    if (observation.feature_index >= keyframe.keypoints.size() ||
        !std::isfinite(observation.pixel.x) ||
        !std::isfinite(observation.pixel.y) ||
        observation.octave < 0) {
        return false;
    }
    return keyframe.image.empty() ||
           (observation.pixel.x >= 0.0f &&
            observation.pixel.y >= 0.0f &&
            observation.pixel.x < keyframe.image.cols &&
            observation.pixel.y < keyframe.image.rows);
}

struct ObservationLink {
    KeyFrameId keyframe_id = 0;
    MapPointId map_point_id = 0;

    bool operator==(const ObservationLink& other) const {
        return keyframe_id == other.keyframe_id &&
               map_point_id == other.map_point_id;
    }
};

struct ObservationLinkHash {
    std::size_t operator()(const ObservationLink& link) const {
        const std::size_t first = std::hash<KeyFrameId>{}(link.keyframe_id);
        const std::size_t second = std::hash<MapPointId>{}(link.map_point_id);
        return first ^ (second + 0x9e3779b9U + (first << 6U) +
                        (first >> 2U));
    }
};

}  // namespace

bool TrackingMapSnapshot::valid() const {
    return map_point_ids.size() == points.size() &&
           descriptors.rows == static_cast<int>(points.size());
}

bool Map::addKeyFrame(const KeyFrame& input) {
    if (keyframes_.count(input.id) != 0 || !isKeyFrameValid(input)) {
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
    if (map_points_.count(input.id) != 0 || !isMapPointValid(input)) {
        return false;
    }

    MapPoint map_point = input;
    map_point.descriptor = input.descriptor.clone();
    const auto inserted =
        map_points_.emplace(map_point.id, std::move(map_point));
    if (!inserted.second) {
        return false;
    }
    if (inserted.first->second.bad) {
        return true;
    }
    if (tracking_snapshot_valid_ &&
        (tracking_snapshot_.map_point_ids.empty() ||
         inserted.first->first > tracking_snapshot_.map_point_ids.back())) {
        appendTrackingPoint(inserted.first->second);
    } else {
        tracking_snapshot_valid_ = false;
    }
    return true;
}

bool Map::addObservation(const Observation& observation) {
    const KeyFrame* keyframe = findKeyFrame(observation.keyframe_id);
    const MapPoint* map_point = findMapPoint(observation.map_point_id);
    if (keyframe == nullptr || map_point == nullptr || map_point->bad) {
        return false;
    }
    if (!isObservationValidForKeyFrame(observation, *keyframe)) {
        return false;
    }
    return observations_.add(observation);
}

bool Map::insertKeyFrameBatch(
    const KeyFrame& input_keyframe,
    const std::vector<MapPoint>& input_points,
    const std::vector<Observation>& input_observations) {
    if (keyframes_.count(input_keyframe.id) != 0 ||
        !isKeyFrameValid(input_keyframe) || input_points.empty() ||
        input_observations.empty()) {
        return false;
    }

    std::unordered_map<MapPointId, const MapPoint*> batch_points;
    batch_points.reserve(input_points.size());
    for (const MapPoint& point : input_points) {
        if (!isMapPointValid(point) || map_points_.count(point.id) != 0 ||
            !batch_points.emplace(point.id, &point).second) {
            return false;
        }
    }

    std::unordered_set<ObservationLink, ObservationLinkHash> batch_links;
    batch_links.reserve(input_observations.size());
    for (const Observation& observation : input_observations) {
        const KeyFrame* keyframe = observation.keyframe_id == input_keyframe.id
                                       ? &input_keyframe
                                       : findKeyFrame(observation.keyframe_id);
        const auto batch_point = batch_points.find(observation.map_point_id);
        const MapPoint* point = batch_point == batch_points.end()
                                    ? findMapPoint(observation.map_point_id)
                                    : batch_point->second;
        const ObservationLink link{observation.keyframe_id,
                                   observation.map_point_id};
        if (keyframe == nullptr || point == nullptr || point->bad ||
            !isObservationValidForKeyFrame(observation, *keyframe) ||
            observations_.contains(link.keyframe_id, link.map_point_id) ||
            !batch_links.insert(link).second) {
            return false;
        }
    }

    if (!addKeyFrame(input_keyframe)) {
        return false;
    }
    for (const MapPoint& point : input_points) {
        if (!addMapPoint(point)) {
            return false;
        }
    }
    for (const Observation& observation : input_observations) {
        if (!addObservation(observation)) {
            return false;
        }
    }
    return true;
}

KeyFrame* Map::findKeyFrame(KeyFrameId id) {
    const auto it = keyframes_.find(id);
    return it == keyframes_.end() ? nullptr : &it->second;
}

const KeyFrame* Map::findKeyFrame(KeyFrameId id) const {
    const auto it = keyframes_.find(id);
    return it == keyframes_.end() ? nullptr : &it->second;
}

const MapPoint* Map::findMapPoint(MapPointId id) const {
    const auto it = map_points_.find(id);
    return it == map_points_.end() ? nullptr : &it->second;
}

bool Map::updateMapPointPosition(MapPointId id,
                                 const cv::Point3d& position_world) {
    if (!isFinitePoint(position_world)) {
        return false;
    }
    const auto point = map_points_.find(id);
    if (point == map_points_.end()) {
        return false;
    }
    point->second.position_world = position_world;
    if (point->second.bad) {
        return true;
    }
    if (tracking_snapshot_valid_) {
        const auto index = tracking_indices_.find(id);
        if (index == tracking_indices_.end()) {
            tracking_snapshot_valid_ = false;
        } else {
            tracking_snapshot_.points[index->second] = cv::Point3f(
                static_cast<float>(position_world.x),
                static_cast<float>(position_world.y),
                static_cast<float>(position_world.z));
        }
    }
    return true;
}

bool Map::eraseMapPoint(MapPointId id) {
    if (map_points_.erase(id) == 0) {
        return false;
    }
    const auto linked = observations_.byMapPoint(id);
    for (const auto& observation : linked) {
        observations_.erase(observation.keyframe_id, observation.map_point_id);
    }
    tracking_snapshot_valid_ = false;
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

void Map::appendTrackingPoint(const MapPoint& point) const {
    tracking_indices_[point.id] = tracking_snapshot_.map_point_ids.size();
    tracking_snapshot_.map_point_ids.push_back(point.id);
    tracking_snapshot_.points.emplace_back(
        static_cast<float>(point.position_world.x),
        static_cast<float>(point.position_world.y),
        static_cast<float>(point.position_world.z));
    tracking_snapshot_.descriptors.push_back(point.descriptor);
}

void Map::rebuildTrackingSnapshot() const {
    tracking_snapshot_ = TrackingMapSnapshot{};
    tracking_indices_.clear();
    std::vector<MapPointId> ids;
    ids.reserve(map_points_.size());
    for (const auto& item : map_points_) {
        if (!item.second.bad) {
            ids.push_back(item.first);
        }
    }
    std::sort(ids.begin(), ids.end());

    tracking_snapshot_.map_point_ids.reserve(ids.size());
    tracking_snapshot_.points.reserve(ids.size());
    tracking_indices_.reserve(ids.size());
    for (MapPointId id : ids) {
        appendTrackingPoint(map_points_.at(id));
    }
    tracking_snapshot_valid_ = true;
}

const TrackingMapSnapshot& Map::trackingSnapshot() const {
    if (!tracking_snapshot_valid_) {
        rebuildTrackingSnapshot();
    }
    return tracking_snapshot_;
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
