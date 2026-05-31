#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace mini_vo {

enum class VOStatus { UNINITIALIZED, TRACKING, LOST };

struct VOSystem {
    cv::Mat K;                               // camera intrinsics

    cv::Mat R_cw;                            // current rotation (world → camera)
    cv::Mat t_cw;                            // current translation

    std::vector<cv::Point3f> map_points;      // 3D map (world frame)
    cv::Mat map_descriptors;                  // descriptors for map points

    cv::Mat prev_img;
    std::vector<cv::KeyPoint> prev_kp;
    cv::Mat prev_desc;
    cv::Mat R_prev, t_prev;                  // previous frame pose

    cv::Mat kf_img;                           // last keyframe image
    std::vector<cv::KeyPoint> kf_kp;
    cv::Mat kf_desc;
    cv::Mat R_kf, t_kf;                      // keyframe pose

    int frame_count = 0;
    int last_keyframe = 0;
    VOStatus status = VOStatus::UNINITIALIZED;

    std::vector<cv::Mat> trajectory_R;        // all frame rotations
    std::vector<cv::Mat> trajectory_t;        // all frame translations
};

} // namespace mini_vo
