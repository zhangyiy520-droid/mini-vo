#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <opencv2/core.hpp>

namespace mini_vo {

using KeyFrameId = std::uint64_t;
using MapPointId = std::uint64_t;

struct Observation {
    KeyFrameId keyframe_id = 0;
    MapPointId map_point_id = 0;
    std::size_t feature_index = 0;
    cv::Point2f pixel{};
    int octave = 0;
    bool outlier = false;
};

class ObservationStore {
public:
    bool add(const Observation& observation);
    bool erase(KeyFrameId keyframe_id, MapPointId map_point_id);
    bool setOutlier(KeyFrameId keyframe_id,
                    MapPointId map_point_id,
                    bool outlier);
    bool contains(KeyFrameId keyframe_id, MapPointId map_point_id) const;

    std::vector<Observation> byKeyFrame(KeyFrameId keyframe_id) const;
    std::vector<Observation> byMapPoint(MapPointId map_point_id) const;

    const std::vector<Observation>& all() const;
    std::size_t size() const;
    bool validate() const;

private:
    std::vector<Observation> observations_;
    std::unordered_map<KeyFrameId, std::unordered_set<MapPointId>> links_;
};

}  // namespace mini_vo
