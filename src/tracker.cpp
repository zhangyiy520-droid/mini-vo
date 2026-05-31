#include "mini_vo/tracker.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace mini_vo {

bool track(VOSystem& vo, const cv::Mat& img,
           const std::vector<cv::KeyPoint>& kp, const cv::Mat& desc,
           cv::Mat& R_out, cv::Mat& t_out)
{
    // --- 1. match current → previous (ratio test) ---
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(desc, vo.prev_desc, knn_matches, 2);

    std::vector<cv::Point2f> pts_cur, pts_ref;
    for (const auto& m : knn_matches) {
        if (m[0].distance < 0.8f * m[1].distance) {
            int idx = m[0].trainIdx;
            if (idx >= (int)vo.prev_kp.size()) continue;
            pts_cur.push_back(kp[m[0].queryIdx].pt);
            pts_ref.push_back(vo.prev_kp[idx].pt);
        }
    }

    std::cout << "[track] matches: " << pts_cur.size();
    if (pts_cur.size() < 30) {
        std::cout << " (too few)" << std::endl;
        return false;
    }

    // --- 2. essential matrix ---
    cv::Mat mask_E;
    cv::Mat E = cv::findEssentialMat(pts_cur, pts_ref, vo.K,
                                      cv::RANSAC, 0.999, 1.0, mask_E);
    int E_inliers = cv::countNonZero(mask_E);
    std::cout << "  E inliers: " << E_inliers;
    if (E_inliers < 20) { std::cout << std::endl; return false; }

    // --- 3. recover relative pose ---
    cv::Mat R_rel, t_rel, mask_pose;
    cv::recoverPose(E, pts_cur, pts_ref, vo.K, R_rel, t_rel, mask_pose);
    int n_inliers = cv::countNonZero(mask_pose);
    std::cout << "  pose inliers: " << n_inliers << std::endl;
    if (n_inliers < 20) return false;

    // --- 4. compose with previous absolute pose ---
    R_out = R_rel * vo.R_prev;
    t_out = R_rel * vo.t_prev + t_rel;
    return true;
}

void triangulateNewPoints(VOSystem& vo, const cv::Mat& /*img*/,
                          const std::vector<cv::KeyPoint>& kp,
                          const cv::Mat& desc,
                          const cv::Mat& R_cur, const cv::Mat& t_cur)
{
    // --- 1. match keyframe → current ---
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(vo.kf_desc, desc, knn_matches, 2);

    std::vector<cv::DMatch> good_matches;
    std::vector<cv::Point2f> pts_kf, pts_cur;
    for (const auto& m : knn_matches) {
        if (m[0].distance < 0.8f * m[1].distance) {
            good_matches.push_back(m[0]);
            pts_kf.push_back(vo.kf_kp[m[0].queryIdx].pt);
            pts_cur.push_back(kp[m[0].trainIdx].pt);
        }
    }
    std::cout << "[tri] matches: " << good_matches.size() << std::endl;
    if (good_matches.size() < 30) return;

    // --- 2. projection matrices ---
    cv::Mat Rt1, Rt2;
    cv::hconcat(vo.R_kf, vo.t_kf, Rt1);
    cv::hconcat(R_cur, t_cur, Rt2);
    cv::Mat P1 = vo.K * Rt1, P2 = vo.K * Rt2;

    cv::Mat pts4D;
    cv::triangulatePoints(P1, P2, pts_kf, pts_cur, pts4D);

    // --- 3. adaptive distance threshold from existing map ---
    std::vector<double> valid_dists;
    for (const auto& p : vo.map_points) {
        double d = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        if (d > 0) valid_dists.push_back(d);
    }
    double max_dist = 100.0;
    if (!valid_dists.empty()) {
        std::sort(valid_dists.begin(), valid_dists.end());
        max_dist = valid_dists[valid_dists.size()/2] * 20.0;
    }

    // --- 4. quality filter ---
    int added = 0, rej_depth = 0, rej_dist = 0, rej_reproj = 0;
    for (int i = 0; i < pts4D.cols; i++) {
        cv::Mat c = pts4D.col(i);
        float W = c.at<float>(3);
        if (std::abs(W) < 1e-6f) { rej_depth++; continue; }
        double X = c.at<float>(0)/W, Y = c.at<float>(1)/W, Z = c.at<float>(2)/W;

        cv::Mat Xw = (cv::Mat_<double>(4,1) << X,Y,Z,1.0);
        cv::Mat Xc1 = Rt1 * Xw, Xc2 = Rt2 * Xw;
        if (Xc1.at<double>(2) <= 0 || Xc2.at<double>(2) <= 0) {
            rej_depth++; continue;
        }

        double dist = std::sqrt(X*X+Y*Y+Z*Z);
        if (dist > max_dist) { rej_dist++; continue; }

        cv::Mat pr1 = P1 * Xw, pr2 = P2 * Xw;
        double u1 = pr1.at<double>(0)/pr1.at<double>(2);
        double v1 = pr1.at<double>(1)/pr1.at<double>(2);
        double u2 = pr2.at<double>(0)/pr2.at<double>(2);
        double v2 = pr2.at<double>(1)/pr2.at<double>(2);
        double e1 = std::sqrt(std::pow(u1-pts_kf[i].x,2)+std::pow(v1-pts_kf[i].y,2));
        double e2 = std::sqrt(std::pow(u2-pts_cur[i].x,2)+std::pow(v2-pts_cur[i].y,2));
        if (std::max(e1,e2) > 3.0) { rej_reproj++; continue; }

        vo.map_points.push_back(cv::Point3f(X,Y,Z));
        vo.map_descriptors.push_back(desc.row(good_matches[i].trainIdx));
        added++;
    }
    std::cout << "[tri] added:" << added << " rej(depth:" << rej_depth
              << " dist:" << rej_dist << " reproj:" << rej_reproj
              << ") total:" << vo.map_points.size() << std::endl;
}

} // namespace mini_vo
