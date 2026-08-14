#pragma once

#include "types.h"

namespace mini_vo {

struct TriangulationReport {
    bool keyframe_inserted = false;
    KeyFrameId keyframe_id = 0;
    std::size_t points_added = 0;
};

/**
 * 用地图点追踪当前帧（3D-2D PnP）。
 *
 * 流程：
 *   1. 当前帧 ORB 提取（2000 特征点）
 *   2. 从 state.map 获取跟踪快照并执行 BF ratio-test 匹配
 *   3. 收集 2D-3D 对应点
 *   4. solvePnPRansac → R, t
 *
 * 匹配数 < 10 或 PnP 内点 < 10 时返回 false。
 *
 * 可选调试输出（传非空指针获取可视化数据）：
 *   out_kp      → 当前帧 ORB 特征点
 *   out_pts2D   → PnP 使用的 2D 观测点
 *   out_pts3D   → PnP 使用的 3D 地图点
 *   out_inliers → PnP RANSAC 内点索引
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
 * 在关键帧和当前帧之间三角化新地图点。
 */
TriangulationReport triangulateNewPoints(
    VOSystem& vo,
    const cv::Mat& img,
    const std::vector<cv::KeyPoint>& kp,
    const cv::Mat& desc,
    const cv::Mat& R_cur,
    const cv::Mat& t_cur);

} // namespace mini_vo
