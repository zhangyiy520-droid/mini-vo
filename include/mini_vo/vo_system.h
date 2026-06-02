#pragma once

#include "types.h"

namespace mini_vo {

/**
 * 主入口 — 处理单帧图像。
 * visualize = true 时保存 debug 图片到 debug/ 目录。
 * timestamp 用于 TUM 格式轨迹输出（非 TUM 数据集时可用帧序号）。
 */
void processFrame(VOSystem& vo, const cv::Mat& img, bool visualize = false, double timestamp = -1.0);

/// 保存轨迹为 TUM 格式（帧序号作为时间戳）
void saveTrajectory(const std::string& path,
                    const std::vector<cv::Mat>& Rs,
                    const std::vector<cv::Mat>& ts);

/// 保存轨迹为 TUM 格式（真实时间戳，用于 evo 评估）
void saveTrajectoryTUM(const std::string& path,
                        const std::vector<double>& timeStamps,
                        const std::vector<cv::Mat>& Rs,
                        const std::vector<cv::Mat>& ts);

/// 保存稀疏 3D 地图点为 ASCII PLY（按深度着色，可用 MeshLab 打开）
void saveMapPLY(const std::string& path,
                const std::vector<cv::Point3f>& pts);

} // namespace mini_vo
