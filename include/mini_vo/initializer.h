#pragma once

#include "types.h"

namespace mini_vo {

/**
 * 从两帧初始化 VO 系统。
 *
 * 流程：
 *   1. ORB 提取 + BF 匹配 + ratio test
 *   2. 本质矩阵（RANSAC）+ SVD 校正（强制 σ₁=σ₂, σ₃=0）
 *   3. recoverPose → R, t
 *   4. DLT 三角化地图点
 *   5. 中值深度离群过滤 + 重投影误差检查
 *
 * 成功时：填充 vo.R_cw / vo.t_cw / vo.map_points / vo.map_descs，
 *         并设置 prev_* / kf_* 状态供后续跟踪。
 *
 * kf_* 固定为第一帧（世界原点），后续三角化均以此为参考做大基线三角化。
 */
bool initialize(VOSystem& vo,
                const cv::Mat& img1,
                const cv::Mat& img2,
                const std::vector<cv::KeyPoint>& kp1,
                const cv::Mat& desc1,
                const std::vector<cv::KeyPoint>& kp2,
                const cv::Mat& desc2);

} // namespace mini_vo
