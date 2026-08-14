#include "mini_vo/backend/backend.h"

#include "mini_vo/backend/local_bundle_adjuster.h"
#include "mini_vo/backend/pose_optimizer.h"

namespace mini_vo {

BackendReport Backend::processKeyFrame(
    Map& map,
    KeyFrameId keyframe_id,
    const CameraIntrinsics& camera) const {
    BackendReport report;
    if (map.findKeyFrame(keyframe_id) == nullptr) {
        report.message = "keyframe not found";
        return report;
    }

    const PoseOptimizationReport pose =
        PoseOptimizer().optimize(map, keyframe_id, camera);
    report.inliers = pose.inliers;
    report.outliers = pose.outliers;
    report.initial_chi2 = pose.initial_chi2;
    report.final_chi2 = pose.final_chi2;
    if (!pose.success) {
        report.message = "pose optimization failed: " + pose.message;
        return report;
    }
    report.pose_optimized = true;
    report.success = true;

    if (map.keyFrameCount() < 3U) {
        report.message = "pose optimized; local BA needs more keyframes";
        return report;
    }

    const LocalBundleAdjustmentReport local =
        LocalBundleAdjuster().optimize(map, keyframe_id, camera);
    if (!local.success) {
        report.message = "pose optimized; local BA skipped: " +
                         local.message;
        return report;
    }

    report.local_ba_optimized = !local.skipped;
    report.inliers = local.optimizer.inliers;
    report.outliers = local.optimizer.outliers;
    report.initial_chi2 = local.optimizer.initial_chi2;
    report.final_chi2 = local.optimizer.final_chi2;
    report.message = "ok";
    return report;
}

}  // namespace mini_vo
