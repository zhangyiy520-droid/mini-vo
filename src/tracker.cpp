#include "mini_vo/tracker.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace mini_vo {

bool track(const cv::Mat& img,
           const cv::Mat& K,
           VOSystem& state,
           cv::Mat& R_out,
           cv::Mat& t_out,
           std::vector<cv::KeyPoint>* out_kp,
           std::vector<cv::Point2f>* out_pts2D,
           std::vector<cv::Point3f>* out_pts3D,
           cv::Mat* out_inliers)
{
    // --- 1. ORB ---
    auto orb = cv::ORB::create(2000);
    std::vector<cv::KeyPoint> kp;
    cv::Mat desc;
    orb->detectAndCompute(img, cv::Mat(), kp, desc);

    // --- 2. match current descriptors → map descriptors ---
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn;
    matcher.knnMatch(desc, state.map_descs, knn, 2);

    // --- 3. ratio test + collect 2D-3D correspondences ---
    std::vector<cv::Point2f> pts2D;
    std::vector<cv::Point3f> pts3D;
    for (const auto& m : knn) {
        if (m[0].distance < 0.8f * m[1].distance) {
            int idx = m[0].trainIdx;
            if (idx >= (int)state.map_points.size()) continue;
            pts2D.push_back(kp[m[0].queryIdx].pt);
            pts3D.push_back(state.map_points[idx]);
        }
    }

    std::cout << "[track] 2D-3D: " << pts2D.size();
    if (pts2D.size() < 10) {
        std::cout << " (too few)" << std::endl;
        return false;
    }

    // --- 4. PnP ---
    cv::Mat rvec, tvec, inliers;
    bool ok = cv::solvePnPRansac(pts3D, pts2D, K, cv::Mat(),
                                  rvec, tvec, false, 100, 6.0, 0.99, inliers);

    int n_inliers = ok && !inliers.empty() ? inliers.rows : 0;
    std::cout << "  inliers: " << n_inliers << std::endl;
    if (!ok || inliers.empty() || n_inliers < 10) return false;

    cv::Rodrigues(rvec, R_out);
    t_out = tvec;

    // --- 5. optional debug outputs ---
    if (out_kp)    *out_kp = std::move(kp);
    if (out_pts2D) *out_pts2D = pts2D;
    if (out_pts3D) *out_pts3D = pts3D;
    if (out_inliers) *out_inliers = inliers.clone();

    return true;
}

void triangulateNewPoints(VOSystem& vo, const cv::Mat& /*img*/,
                          const std::vector<cv::KeyPoint>& kp,
                          const cv::Mat& desc,
                          const cv::Mat& R_cur, const cv::Mat& t_cur)
{
    // --- 1. match keyframe → current ---
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn;
    matcher.knnMatch(vo.kf_desc, desc, knn, 2);

    std::vector<cv::DMatch> good;
    std::vector<cv::Point2f> pts_kf, pts_cur;
    for (const auto& m : knn) {
        if (m[0].distance < 0.8f * m[1].distance) {
            good.push_back(m[0]);
            pts_kf.push_back(vo.kf_kp[m[0].queryIdx].pt);
            pts_cur.push_back(kp[m[0].trainIdx].pt);
        }
    }
    std::cout << "[tri] matches: " << good.size() << std::endl;
    if (good.size() < 30) return;

    // --- 2. projection matrices ---
    cv::Mat Rt1, Rt2;
    cv::hconcat(vo.R_kf, vo.t_kf, Rt1);
    cv::hconcat(R_cur, t_cur, Rt2);
    cv::Mat P1 = vo.K * Rt1, P2 = vo.K * Rt2;

    cv::Mat pts4D;
    cv::triangulatePoints(P1, P2, pts_kf, pts_cur, pts4D);

    // --- 3. camera centers for parallax check ---
    cv::Mat O1, O2;
    {
        cv::Mat Rkf_t;
        cv::transpose(vo.R_kf, Rkf_t);
        O1 = -Rkf_t * vo.t_kf;
        cv::Mat Rc_t;
        cv::transpose(R_cur, Rc_t);
        O2 = -Rc_t * t_cur;
    }

    // --- 4. adaptive distance threshold (×5 not ×20) ---
    std::vector<double> zs;
    for (const auto& p : vo.map_points) {
        double d = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        if (d > 0) zs.push_back(d);
    }
    double maxD = 100.0, minD = 0.0;
    if (!zs.empty()) {
        std::sort(zs.begin(), zs.end());
        double med = zs[zs.size()/2];
        maxD = med * 5.0;
        minD = med * 0.1;
    }

    // --- 5. quality filter ---
    int added = 0, rej_depth = 0, rej_parallax = 0, rej_dist = 0, rej_reproj = 0;
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

        // parallax angle check
        cv::Mat Xw3 = (cv::Mat_<double>(3,1) << X, Y, Z);
        cv::Mat ray1 = Xw3 - O1;
        cv::Mat ray2 = Xw3 - O2;
        double n1 = cv::norm(ray1), n2 = cv::norm(ray2);
        if (n1 > 1e-6 && n2 > 1e-6) {
            double cos_angle = ray1.dot(ray2) / (n1 * n2);
            if (cos_angle > 0.999) { rej_parallax++; continue; }
        }

        double dist = std::sqrt(X*X+Y*Y+Z*Z);
        if (dist < minD || dist > maxD) { rej_dist++; continue; }

        cv::Mat pr1 = P1 * Xw, pr2 = P2 * Xw;
        double u1 = pr1.at<double>(0)/pr1.at<double>(2);
        double v1 = pr1.at<double>(1)/pr1.at<double>(2);
        double u2 = pr2.at<double>(0)/pr2.at<double>(2);
        double v2 = pr2.at<double>(1)/pr2.at<double>(2);
        double e1 = std::sqrt(std::pow(u1-pts_kf[i].x,2)+std::pow(v1-pts_kf[i].y,2));
        double e2 = std::sqrt(std::pow(u2-pts_cur[i].x,2)+std::pow(v2-pts_cur[i].y,2));
        if (std::max(e1,e2) > 3.0) { rej_reproj++; continue; }

        vo.map_points.push_back(cv::Point3f(X,Y,Z));
        vo.map_descs.push_back(desc.row(good[i].trainIdx));
        added++;
    }
    std::cout << "[tri] added:" << added
              << " rej(depth:" << rej_depth
              << " parallax:" << rej_parallax
              << " dist:" << rej_dist
              << " reproj:" << rej_reproj
              << ") total:" << vo.map_points.size() << std::endl;
}

} // namespace mini_vo
