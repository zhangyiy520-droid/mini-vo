#include "mini_vo/backend/local_bundle_adjuster.h"

#include <algorithm>

namespace mini_vo {

LocalBundleAdjustmentReport LocalBundleAdjuster::optimize(
    Map& map,
    KeyFrameId center_keyframe_id,
    const CameraIntrinsics& camera,
    const LocalBundleAdjustmentOptions& options) const {
    LocalBundleAdjustmentReport report;
    if (!options.enabled) {
        report.success = true;
        report.skipped = true;
        report.message = "Local BA disabled by configuration";
        return report;
    }

    const LocalBAWindow window = LocalWindowSelector().select(
        map, center_keyframe_id, options.window);
    report.local_keyframes = window.local_keyframes.size();
    report.fixed_keyframes = window.fixed_keyframes.size();
    report.local_map_points = window.local_map_points.size();
    if (!window.success) {
        report.message = window.message;
        return report;
    }

    BundleAdjustmentOptions ba_options = options.optimizer;
    ba_options.keyframe_ids.insert(window.local_keyframes.begin(),
                                   window.local_keyframes.end());
    ba_options.keyframe_ids.insert(window.fixed_keyframes.begin(),
                                   window.fixed_keyframes.end());
    ba_options.map_point_ids.insert(window.local_map_points.begin(),
                                    window.local_map_points.end());
    ba_options.fixed_keyframe_ids.insert(window.fixed_keyframes.begin(),
                                         window.fixed_keyframes.end());

    std::vector<KeyFrameId> anchor_candidates = window.local_keyframes;
    std::sort(anchor_candidates.begin(), anchor_candidates.end());
    for (KeyFrameId id : anchor_candidates) {
        if (ba_options.fixed_keyframe_ids.size() >= 2U) {
            break;
        }
        ba_options.fixed_keyframe_ids.insert(id);
    }
    if (ba_options.fixed_keyframe_ids.size() < 2U) {
        report.message = "Local BA cannot anchor monocular scale";
        return report;
    }

    report.optimizer =
        BundleAdjuster().optimize(map, camera, ba_options);
    if (!report.optimizer.success) {
        report.message = report.optimizer.message;
        return report;
    }

    for (const RejectedObservation& rejected :
         report.optimizer.rejected) {
        if (map.setObservationOutlier(rejected.keyframe_id,
                                      rejected.map_point_id, true)) {
            ++report.marked_outliers;
        }
    }

    report.success = true;
    report.message = "ok";
    return report;
}

}  // namespace mini_vo
