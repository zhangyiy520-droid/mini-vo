#include "mini_vo/core/se3_utils.h"

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <cmath>
#include <iostream>

namespace {

constexpr double kTolerance = 1e-12;

bool near(const Eigen::Vector3d& lhs,
          const Eigen::Vector3d& rhs,
          double tolerance = kTolerance) {
    return (lhs - rhs).norm() < tolerance;
}

bool testSkew() {
    const Eigen::Vector3d a(1.0, 2.0, 3.0);
    const Eigen::Vector3d b(-4.0, 5.0, 2.0);
    return near(mini_vo::skew(a) * b, a.cross(b));
}

bool testPureTranslationIncrement() {
    mini_vo::PoseIncrement increment;
    increment.upsilon = Eigen::Vector3d(1.0, -2.0, 0.5);

    const Sophus::SE3d delta = mini_vo::toSophusIncrement(increment);
    return near(delta.translation(), increment.upsilon) &&
           delta.so3().log().norm() < kTolerance;
}

bool testLeftIncrement() {
    const Sophus::SE3d original(
        Sophus::SO3d::exp(Eigen::Vector3d(0.0, 0.0, 0.25)),
        Eigen::Vector3d(1.0, 2.0, 3.0));

    mini_vo::PoseIncrement increment;
    increment.upsilon = Eigen::Vector3d(0.2, -0.1, 0.3);
    increment.omega = Eigen::Vector3d(0.01, -0.02, 0.03);

    Sophus::SE3d actual = original;
    mini_vo::applyLeftIncrement(increment, actual);
    const Sophus::SE3d expected =
        mini_vo::toSophusIncrement(increment) * original;

    return (actual.matrix() - expected.matrix()).norm() < kTolerance;
}

bool testRotationCheck() {
    const Eigen::Matrix3d valid_rotation =
        Sophus::SO3d::exp(Eigen::Vector3d(0.1, -0.2, 0.3)).matrix();
    const mini_vo::OrthonormalityCheck valid =
        mini_vo::checkRotationMatrix(valid_rotation);

    Eigen::Matrix3d invalid_rotation = valid_rotation;
    invalid_rotation(0, 0) += 0.1;
    const mini_vo::OrthonormalityCheck invalid =
        mini_vo::checkRotationMatrix(invalid_rotation);

    return valid.valid && valid.rt_r_error < 1e-9 &&
           std::abs(valid.determinant - 1.0) < 1e-9 &&
           !invalid.valid;
}

}  // namespace

int main() {
    if (!testSkew()) {
        std::cerr << "[FAIL] skew(v) * p must equal v.cross(p)\n";
        return 1;
    }
    if (!testPureTranslationIncrement()) {
        std::cerr << "[FAIL] pure translation SE3 increment is incorrect\n";
        return 1;
    }
    if (!testLeftIncrement()) {
        std::cerr << "[FAIL] applyLeftIncrement does not left-multiply Tcw\n";
        return 1;
    }
    if (!testRotationCheck()) {
        std::cerr << "[FAIL] rotation matrix validation is incorrect\n";
        return 1;
    }

    std::cout << "[PASS] SE3 utilities: skew, exp, left update, rotation check\n";
    return 0;
}
