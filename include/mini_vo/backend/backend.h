#pragma once

#include <cstddef>
#include <string>

#include "mini_vo/camera/camera_intrinsics.h"
#include "mini_vo/core/map.h"

namespace mini_vo {

struct BackendReport {
    bool success = false;
    bool pose_optimized = false;
    bool local_ba_optimized = false;
    std::size_t inliers = 0;
    std::size_t outliers = 0;
    double initial_chi2 = 0.0;
    double final_chi2 = 0.0;
    std::string message;
};

class Backend {
public:
    BackendReport processKeyFrame(
        Map& map,
        KeyFrameId keyframe_id,
        const CameraIntrinsics& camera) const;
};

}  // namespace mini_vo
