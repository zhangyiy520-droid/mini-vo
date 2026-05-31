#pragma once

#include "types.h"

namespace mini_vo {

/**
 * Main VO entry point — processes one frame through the state machine.
 *
 * States:
 *   UNINITIALIZED → wait for 2nd frame, run initialize()
 *   TRACKING      → track against prev frame, optional keyframe
 *   LOST          → attempt re-initialization from prev frame
 */
void processFrame(VOSystem& vo, const cv::Mat& img);

/**
 * Save trajectory in TUM format:
 *   frame_id tx ty tz qx qy qz qw
 */
void saveTrajectory(const std::string& path,
                    const std::vector<cv::Mat>& Rs,
                    const std::vector<cv::Mat>& ts);

/**
 * Save map points as ASCII PLY (colored by depth).
 */
void saveMapPLY(const std::string& path,
                const std::vector<cv::Point3f>& pts);

} // namespace mini_vo
