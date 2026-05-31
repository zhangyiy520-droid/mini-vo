#pragma once

#include "types.h"

namespace mini_vo {

/**
 * Track current frame against the previous frame (2D-2D).
 *
 * Pipeline:
 *   1. ORB extract on new frame
 *   2. BF match current → previous descriptors (ratio test)
 *   3. Essential matrix (RANSAC) → E
 *   4. recoverPose → R_rel, t_rel
 *   5. Compose: R_cur = R_rel * R_prev, t_cur = R_rel * t_prev + t_rel
 *
 * Returns false if matching fails (<30 matches or <20 inliers).
 */
bool track(VOSystem& vo,
           const cv::Mat& img,
           const std::vector<cv::KeyPoint>& kp,
           const cv::Mat& desc,
           cv::Mat& R_out,
           cv::Mat& t_out);

/**
 * Triangulate new map points between last keyframe and current frame.
 * Called every N frames (keyframe interval).
 */
void triangulateNewPoints(VOSystem& vo,
                          const cv::Mat& img,
                          const std::vector<cv::KeyPoint>& kp,
                          const cv::Mat& desc,
                          const cv::Mat& R_cur,
                          const cv::Mat& t_cur);

} // namespace mini_vo
