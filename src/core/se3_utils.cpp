#include "mini_vo/core/se3_utils.h"

#include <cmath>

namespace mini_vo {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(),  v.y(),
         v.z(),  0.0, -v.x(),
        -v.y(),  v.x(),  0.0;
    return m;
}

Sophus::SE3d toSophusIncrement(const PoseIncrement& increment) {
    Eigen::Matrix<double, 6, 1> dx;
    dx.head<3>() = increment.upsilon;
    dx.tail<3>() = increment.omega;
    return Sophus::SE3d::exp(dx);
}

void applyLeftIncrement(const PoseIncrement& increment,
                        Sophus::SE3d& Tcw) {
    Tcw = toSophusIncrement(increment) * Tcw;
}

OrthonormalityCheck checkRotationMatrix(const Eigen::Matrix3d& R) {
    OrthonormalityCheck check;
    const Eigen::Matrix3d should_be_identity =
        R.transpose() * R - Eigen::Matrix3d::Identity();
    check.rt_r_error = should_be_identity.lpNorm<Eigen::Infinity>();
    check.determinant = R.determinant();
    check.valid = std::isfinite(check.rt_r_error) &&
                  std::isfinite(check.determinant) &&
                  check.rt_r_error < 1e-9 &&
                  std::abs(check.determinant - 1.0) < 1e-9;
    return check;
}

}  // namespace mini_vo
