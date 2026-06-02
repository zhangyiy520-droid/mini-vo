#pragma once

#include "types.h"

namespace mini_vo {

/**
 * Track current frame against existing map points (3D-2D PnP).
 *
 * Pipeline:
 *   1. ORB extract on current frame (2000 features)
 *   2. BF match desc → state.map_descs (ratio test 0.8)
 *   3. Collect 2D-3D correspondences
 *   4. solvePnPRansac → R, t
 *
 * Returns false if fewer than 10 matches or <10 PnP inliers.
 *
 * Optional outputs (pass non-null pointers for debug visualization):
 *   out_kp      → ORB keypoints extracted from img
 *   out_pts2D   → 2D points used in PnP
 *   out_pts3D   → 3D map points used in PnP
 *   out_inliers → PnP inlier indices
 */
bool track(const cv::Mat& img,
           const cv::Mat& K,
           VOSystem& state,
           cv::Mat& R_out,
           cv::Mat& t_out,
           std::vector<cv::KeyPoint>* out_kp = nullptr,
           std::vector<cv::Point2f>* out_pts2D = nullptr,
           std::vector<cv::Point3f>* out_pts3D = nullptr,
           cv::Mat* out_inliers = nullptr);

/**
 * Triangulate new map points between keyframe and current frame.
 * Called every N frames (keyframe interval).
 */
void triangulateNewPoints(VOSystem& vo,
                          const cv::Mat& img,
                          const std::vector<cv::KeyPoint>& kp,
                          const cv::Mat& desc,
                          const cv::Mat& R_cur,
                          const cv::Mat& t_cur);

} // namespace mini_vo
