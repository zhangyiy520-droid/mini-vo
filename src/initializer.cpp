#include "mini_vo/initializer.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace mini_vo {

bool initialize(VOSystem& vo, const cv::Mat& img1, const cv::Mat& img2,
                const std::vector<cv::KeyPoint>& kp1,
                const cv::Mat& desc1,
                const std::vector<cv::KeyPoint>& kp2,
                const cv::Mat& desc2)
{
    // --- 1. ratio test 匹配 ---
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn;
    matcher.knnMatch(desc1, desc2, knn, 2);

    std::vector<cv::DMatch> good;
    std::vector<cv::Point2f> pts1, pts2;
    for (const auto& m : knn) {
        if (m[0].distance < 0.8f * m[1].distance) {
            good.push_back(m[0]);
            pts1.push_back(kp1[m[0].queryIdx].pt);
            pts2.push_back(kp2[m[0].trainIdx].pt);
        }
    }
    std::cout << "[init] 匹配数: " << good.size() << std::endl;
    if (good.size() < 30) return false;

    // --- 2. 本质矩阵（RANSAC 内置） ---
    cv::Mat mask_E;
    cv::Mat E = cv::findEssentialMat(pts1, pts2, vo.K,
                                      cv::RANSAC, 0.999, 1.0, mask_E);
    if (E.empty() || cv::countNonZero(mask_E) < 20) return false;

    // --- 3. SVD 校正（强制 σ₁=σ₂, σ₃=0，保证 E 满足本质矩阵约束） ---
    cv::Mat w, u, vt;
    cv::SVD::compute(E, w, u, vt);
    double sm = (w.at<double>(0) + w.at<double>(1)) / 2.0;
    cv::Mat wc = (cv::Mat_<double>(3,3) << sm,0,0, 0,sm,0, 0,0,0);
    cv::Mat Ec = u * wc * vt;

    // --- 4. 恢复位姿 ---
    cv::Mat R, t, mask_pose;
    cv::recoverPose(Ec, pts1, pts2, vo.K, R, t, mask_pose);

    // 收集位姿内点 + 保存原始 good[] 索引（修复：mask_pose 过滤后索引错位）
    std::vector<cv::Point2f> ipts1, ipts2;
    std::vector<int> iorig;   // 每个内点在 good[] 中的原始索引
    for (int i = 0; i < mask_pose.rows; i++) {
        if (mask_pose.at<uchar>(i)) {
            ipts1.push_back(pts1[i]);
            ipts2.push_back(pts2[i]);
            iorig.push_back(i);
        }
    }
    std::cout << "[init] 位姿内点: " << ipts1.size() << std::endl;
    if (ipts1.size() < 20) return false;

    // --- 5. DLT 三角化 ---
    cv::Mat P1 = vo.K * (cv::Mat_<double>(3,4) << 1,0,0,0, 0,1,0,0, 0,0,1,0);
    cv::Mat Rt; cv::hconcat(R, t, Rt);
    cv::Mat P2 = vo.K * Rt;
    cv::Mat pts4D;
    cv::triangulatePoints(P1, P2, ipts1, ipts2, pts4D);

    // --- 6. 中值深度自适应离群阈值 ---
    std::vector<double> zs;
    for (int i = 0; i < pts4D.cols; i++) {
        cv::Mat c = pts4D.col(i);
        float W = c.at<float>(3);
        if (std::abs(W) < 1e-6f) continue;
        double Z = c.at<float>(2) / W;
        if (Z > 0) zs.push_back(Z);
    }
    if (zs.size() < 10) return false;
    std::sort(zs.begin(), zs.end());
    double medZ = zs[zs.size()/2];
    double maxZ = medZ * 10.0;

    // --- 7. 质量过滤（用 iorig[i] 保证描述子索引正确） ---
    std::vector<cv::Point3f> pts3D;
    cv::Mat descs_out;
    for (int i = 0; i < pts4D.cols; i++) {
        cv::Mat c = pts4D.col(i);
        float W = c.at<float>(3);
        if (std::abs(W) < 1e-6f) continue;
        double X = c.at<float>(0)/W, Y = c.at<float>(1)/W, Z = c.at<float>(2)/W;

        cv::Mat Xc2 = Rt * (cv::Mat_<double>(4,1) << X,Y,Z,1.0);
        if (Z <= 0 || Z > maxZ || Xc2.at<double>(2) <= 0) continue;

        // 重投影误差检查
        cv::Mat Xh = (cv::Mat_<double>(4,1) << X,Y,Z,1.0);
        cv::Mat pr1 = P1 * Xh, pr2 = P2 * Xh;
        double u1 = pr1.at<double>(0)/pr1.at<double>(2);
        double v1 = pr1.at<double>(1)/pr1.at<double>(2);
        double u2 = pr2.at<double>(0)/pr2.at<double>(2);
        double v2 = pr2.at<double>(1)/pr2.at<double>(2);
        double e1 = std::sqrt(std::pow(u1-ipts1[i].x,2)+std::pow(v1-ipts1[i].y,2));
        double e2 = std::sqrt(std::pow(u2-ipts2[i].x,2)+std::pow(v2-ipts2[i].y,2));
        if (std::max(e1,e2) > 3.0) continue;

        pts3D.push_back(cv::Point3f(X,Y,Z));
        descs_out.push_back(desc2.row(good[iorig[i]].trainIdx));
    }
    std::cout << "[init] 地图点: " << pts3D.size() << std::endl;
    if (pts3D.size() < 20) return false;

    // --- 8. 写入状态 ---
    vo.R_cw = R.clone();
    vo.t_cw = t.clone();
    vo.map_points = pts3D;
    vo.map_descs = descs_out.clone();

    vo.prev_img = img2.clone();
    vo.prev_kp = kp2;
    vo.prev_desc = desc2.clone();
    vo.R_prev = R.clone();
    vo.t_prev = t.clone();

    // kf_* = init 的第一帧（世界原点，做大基线三角化参考）
    vo.kf_img = img1.clone();
    vo.kf_kp = kp1;
    vo.kf_desc = desc1.clone();
    vo.R_kf = cv::Mat::eye(3,3,CV_64F);
    vo.t_kf = cv::Mat::zeros(3,1,CV_64F);

    return true;
}

} // namespace mini_vo
