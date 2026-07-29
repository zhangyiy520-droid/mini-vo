#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace mini_vo {

struct PoseIncrement {
    Eigen::Vector3d upsilon = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
};

struct OrthonormalityCheck {
    double rt_r_error = 0.0;
    double determinant = 1.0;
    bool valid = false;
};

Eigen::Matrix3d skew(const Eigen::Vector3d& v);

Sophus::SE3d toSophusIncrement(const PoseIncrement& increment);

void applyLeftIncrement(const PoseIncrement& increment,
                        Sophus::SE3d& Tcw);

OrthonormalityCheck checkRotationMatrix(const Eigen::Matrix3d& R);

}
