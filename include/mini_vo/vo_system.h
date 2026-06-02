#pragma once

#include "types.h"

namespace mini_vo {

void processFrame(VOSystem& vo, const cv::Mat& img, bool visualize = false, double timestamp = -1.0);

void saveTrajectory(const std::string& path,
                    const std::vector<cv::Mat>& Rs,
                    const std::vector<cv::Mat>& ts);

void saveTrajectoryTUM(const std::string& path,
                        const std::vector<double>& timeStamps,
                        const std::vector<cv::Mat>& Rs,
                        const std::vector<cv::Mat>& ts);

void saveMapPLY(const std::string& path,
                const std::vector<cv::Point3f>& pts);

} // namespace mini_vo
