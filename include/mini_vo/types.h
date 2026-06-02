#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace mini_vo {

enum class VOStatus { UNINITIALIZED, TRACKING, LOST };

struct VOSystem {
    cv::Mat K;                                // 相机内参矩阵

    cv::Mat R_cw;                             // 当前旋转（世界 → 相机）
    cv::Mat t_cw;                             // 当前平移

    std::vector<cv::Point3f> map_points;       // 3D 地图点（世界坐标系）
    cv::Mat map_descs;                        // 地图点描述子

    cv::Mat prev_img;
    std::vector<cv::KeyPoint> prev_kp;
    cv::Mat prev_desc;
    cv::Mat R_prev, t_prev;                   // 上一帧位姿

    cv::Mat kf_img;                           // 关键帧图像（init 第一帧，三角化参考）
    std::vector<cv::KeyPoint> kf_kp;
    cv::Mat kf_desc;
    cv::Mat R_kf, t_kf;                       // 关键帧位姿

    int frame_count = 0;
    int last_keyframe = 0;
    VOStatus status = VOStatus::UNINITIALIZED;

    std::vector<cv::Mat> trajectory_R;        // 全局轨迹旋转
    std::vector<cv::Mat> trajectory_t;        // 全局轨迹平移
    std::vector<double> trajectory_ts;        // 每帧时间戳
};

} // namespace mini_vo
