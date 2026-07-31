#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "mini_vo/core/map.h"

namespace mini_vo {

struct LocalWindowOptions {
    std::size_t max_local_keyframes = 5;
    std::size_t max_fixed_keyframes = 10;
    std::size_t minimum_shared_points = 5;
};

struct LocalBAWindow {
    bool success = false;
    std::vector<KeyFrameId> local_keyframes;
    std::vector<KeyFrameId> fixed_keyframes;
    std::vector<MapPointId> local_map_points;
    std::unordered_map<KeyFrameId, std::size_t> shared_point_counts;
    std::string message;
};

class LocalWindowSelector {
public:
    LocalBAWindow select(
        const Map& map,
        KeyFrameId center_keyframe_id,
        const LocalWindowOptions& options = {}) const;
};

}  // namespace mini_vo
