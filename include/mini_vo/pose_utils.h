#pragma once
#include<opencv2/core.hpp>

namespace mini_vo{

struct PoseCW{
    cv::Mat Rcw;
    cv::Mat tcw;
};

struct PoseWC{
    cv::Mat Rwc;
    cv::Mat twc;
};

PoseWC invertPose(const PoseCW& posecw);
cv::Point3d cameraCenterWorld(const PoseCW& posewc);
}
