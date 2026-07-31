#pragma once

#include <string>

#include "mini_vo/backend/bundle_adjuster.h"
#include "mini_vo/backend/local_window_selector.h"

namespace mini_vo {

struct LocalBundleAdjustmentOptions {
    bool enabled = true;
    LocalWindowOptions window;
    BundleAdjustmentOptions optimizer;
};

struct LocalBundleAdjustmentReport {
    bool success = false;
    bool skipped = false;
    std::size_t local_keyframes = 0;
    std::size_t fixed_keyframes = 0;
    std::size_t local_map_points = 0;
    std::size_t marked_outliers = 0;
    BundleAdjustmentReport optimizer;
    std::string message;
};

class LocalBundleAdjuster {
public:
    LocalBundleAdjustmentReport optimize(
        Map& map,
        KeyFrameId center_keyframe_id,
        const CameraIntrinsics& camera,
        const LocalBundleAdjustmentOptions& options = {}) const;
};

}  // namespace mini_vo
