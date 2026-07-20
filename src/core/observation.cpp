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

    const auto duplicate = std::find_if(
        observations_.begin(), observations_.end(),
        [&](const Observation& current) {
            return sameLink(current,
                            observation.keyframe_id,
                            observation.map_point_id);
        });

    if (duplicate != observations_.end()) {
        return false;
    }

    observations_.push_back(observation);
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
    return observations_.size() != old_size;
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
    for (std::size_t i = 0; i < observations_.size(); ++i) {
        const auto& observation = observations_[i];
        if (!isFinitePixel(observation.pixel) || observation.octave < 0) {
            return false;
        }
        for (std::size_t j = i + 1; j < observations_.size(); ++j) {
            if (sameLink(observations_[j],
                         observation.keyframe_id,
                         observation.map_point_id)) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace mini_vo

