#pragma once

#include "types.h"

namespace mini_vo {

/**
 * Initialize the VO system from two frames.
 *
 * Pipeline:
 *   1. ORB extract + BF match + ratio test
 *   2. Essential matrix (RANSAC) + SVD correction
 *   3. recoverPose → R, t
 *   4. Triangulate map points
 *   5. Median-depth-based outlier filtering
 *   6. Reprojection error check
 *
 * On success: populates vo.R_cw / vo.t_cw / vo.map_points / vo.map_descs
 *             and sets prev_* / kf_* state for the next tracking frame.
 */
bool initialize(VOSystem& vo,
                const cv::Mat& img1,
                const cv::Mat& img2,
                const std::vector<cv::KeyPoint>& kp1,
                const cv::Mat& desc1,
                const std::vector<cv::KeyPoint>& kp2,
                const cv::Mat& desc2);

} // namespace mini_vo
