#pragma once

#include <cmath>

namespace mini_vo {

struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;

    bool valid() const {
        return std::isfinite(fx) && std::isfinite(fy) &&
               std::isfinite(cx) && std::isfinite(cy) &&
               fx > 0.0 && fy > 0.0;
    }
};

}  // namespace mini_vo
