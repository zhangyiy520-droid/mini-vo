#include "mini_vo/backend/local_window_selector.h"

#include <algorithm>
#include <unordered_set>
#include <utility>

namespace mini_vo {
namespace {

using CountEntry = std::pair<KeyFrameId, std::size_t>;

std::vector<CountEntry> sortCounts(
    const std::unordered_map<KeyFrameId, std::size_t>& counts) {
    std::vector<CountEntry> sorted(counts.begin(), counts.end());
    std::sort(sorted.begin(), sorted.end(),
              [](const CountEntry& lhs, const CountEntry& rhs) {
                  if (lhs.second != rhs.second) {
                      return lhs.second > rhs.second;
                  }
                  return lhs.first < rhs.first;
              });
    return sorted;
}

}  // namespace

LocalBAWindow LocalWindowSelector::select(
    const Map& map,
    KeyFrameId center_keyframe_id,
    const LocalWindowOptions& options) const {
    LocalBAWindow window;
    if (map.findKeyFrame(center_keyframe_id) == nullptr) {
        window.message = "center keyframe not found";
        return window;
    }
    if (options.max_local_keyframes == 0) {
        window.message = "max_local_keyframes must be positive";
        return window;
    }

    std::unordered_set<MapPointId> center_points;
    for (const Observation& observation :
         map.observations().byKeyFrame(center_keyframe_id)) {
        const MapPoint* point = map.findMapPoint(observation.map_point_id);
        if (!observation.outlier && point != nullptr && !point->bad) {
            center_points.insert(observation.map_point_id);
        }
    }
    if (center_points.empty()) {
        window.message = "center keyframe has no valid map points";
        return window;
    }

    std::unordered_map<KeyFrameId, std::size_t> shared_counts;
    for (const Observation& observation : map.observations().all()) {
        if (!observation.outlier &&
            observation.keyframe_id != center_keyframe_id &&
            center_points.count(observation.map_point_id) != 0) {
            ++shared_counts[observation.keyframe_id];
        }
    }
    window.shared_point_counts = shared_counts;
    window.local_keyframes.push_back(center_keyframe_id);
    for (const CountEntry& entry : sortCounts(shared_counts)) {
        if (window.local_keyframes.size() >=
            options.max_local_keyframes) {
            break;
        }
        if (entry.second >= options.minimum_shared_points) {
            window.local_keyframes.push_back(entry.first);
        }
    }

    const std::unordered_set<KeyFrameId> local_keyframe_set(
        window.local_keyframes.begin(), window.local_keyframes.end());
    std::unordered_set<MapPointId> local_point_set;
    for (const Observation& observation : map.observations().all()) {
        const MapPoint* point = map.findMapPoint(observation.map_point_id);
        if (!observation.outlier && point != nullptr && !point->bad &&
            local_keyframe_set.count(observation.keyframe_id) != 0) {
            local_point_set.insert(observation.map_point_id);
        }
    }
    window.local_map_points.assign(local_point_set.begin(),
                                   local_point_set.end());
    std::sort(window.local_map_points.begin(),
              window.local_map_points.end());

    std::unordered_map<KeyFrameId, std::size_t> boundary_counts;
    for (const Observation& observation : map.observations().all()) {
        if (!observation.outlier &&
            local_point_set.count(observation.map_point_id) != 0 &&
            local_keyframe_set.count(observation.keyframe_id) == 0) {
            ++boundary_counts[observation.keyframe_id];
        }
    }
    for (const CountEntry& entry : sortCounts(boundary_counts)) {
        if (window.fixed_keyframes.size() >=
            options.max_fixed_keyframes) {
            break;
        }
        window.fixed_keyframes.push_back(entry.first);
    }

    window.success = !window.local_map_points.empty();
    window.message = window.success ? "ok" : "local window has no map points";
    return window;
}

}  // namespace mini_vo
