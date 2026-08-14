#include "mini_vo/core/observation.h"

#include <algorithm>
#include <cmath>

namespace mini_vo {

namespace {

bool sameLink(const Observation& observation,
              KeyFrameId keyframe_id,
              MapPointId map_point_id) {
    return observation.keyframe_id == keyframe_id &&
           observation.map_point_id == map_point_id;
}

bool isFinitePixel(const cv::Point2f& pixel) {
    return std::isfinite(pixel.x) && std::isfinite(pixel.y);
}

}  // namespace

bool ObservationStore::add(const Observation& observation) {
    if (!isFinitePixel(observation.pixel) || observation.octave < 0) {
        return false;
    }
    if (contains(observation.keyframe_id, observation.map_point_id)) {
        return false;
    }

    observations_.push_back(observation);
    links_[observation.keyframe_id].insert(observation.map_point_id);
    return true;
}

bool ObservationStore::erase(KeyFrameId keyframe_id,
                             MapPointId map_point_id) {
    const auto old_size = observations_.size();
    observations_.erase(
        std::remove_if(observations_.begin(), observations_.end(),
                       [&](const Observation& observation) {
                           return sameLink(observation,
                                           keyframe_id,
                                           map_point_id);
                       }),
        observations_.end());
    if (observations_.size() == old_size) {
        return false;
    }
    const auto keyframe = links_.find(keyframe_id);
    if (keyframe != links_.end()) {
        keyframe->second.erase(map_point_id);
        if (keyframe->second.empty()) {
            links_.erase(keyframe);
        }
    }
    return true;
}

bool ObservationStore::setOutlier(KeyFrameId keyframe_id,
                                  MapPointId map_point_id,
                                  bool outlier) {
    for (Observation& observation : observations_) {
        if (sameLink(observation, keyframe_id, map_point_id)) {
            observation.outlier = outlier;
            return true;
        }
    }
    return false;
}

bool ObservationStore::contains(KeyFrameId keyframe_id,
                                MapPointId map_point_id) const {
    const auto keyframe = links_.find(keyframe_id);
    return keyframe != links_.end() &&
           keyframe->second.count(map_point_id) != 0;
}

std::vector<Observation> ObservationStore::byKeyFrame(
    KeyFrameId keyframe_id) const {
    std::vector<Observation> result;
    for (const auto& observation : observations_) {
        if (observation.keyframe_id == keyframe_id) {
            result.push_back(observation);
        }
    }
    return result;
}

std::vector<Observation> ObservationStore::byMapPoint(
    MapPointId map_point_id) const {
    std::vector<Observation> result;
    for (const auto& observation : observations_) {
        if (observation.map_point_id == map_point_id) {
            result.push_back(observation);
        }
    }
    return result;
}

const std::vector<Observation>& ObservationStore::all() const {
    return observations_;
}

std::size_t ObservationStore::size() const {
    return observations_.size();
}

bool ObservationStore::validate() const {
    std::size_t indexed_links = 0;
    for (const auto& keyframe : links_) {
        indexed_links += keyframe.second.size();
    }
    if (indexed_links != observations_.size()) {
        return false;
    }
    for (std::size_t i = 0; i < observations_.size(); ++i) {
        const auto& observation = observations_[i];
        if (!isFinitePixel(observation.pixel) || observation.octave < 0 ||
            !contains(observation.keyframe_id,
                      observation.map_point_id)) {
            return false;
        }
    }
    return true;
}

}  // namespace mini_vo
