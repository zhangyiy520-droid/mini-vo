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
    // --- 1. ratio test ---
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(desc1, desc2, knn_matches, 2);

    std::vector<cv::DMatch> good_matches;
    std::vector<cv::Point2f> pts1, pts2;
    for (const auto& m : knn_matches) {
        if (m[0].distance < 0.8f * m[1].distance) {
            good_matches.push_back(m[0]);
            pts1.push_back(kp1[m[0].queryIdx].pt);
            pts2.push_back(kp2[m[0].trainIdx].pt);
        }
    }
    std::cout << "[init] matches: " << good_matches.size() << std::endl;
    if (good_matches.size() < 30) return false;

    // --- 2. essential matrix (RANSAC inside) ---
    cv::Mat mask_E;
    cv::Mat E = cv::findEssentialMat(pts1, pts2, vo.K,
                                      cv::RANSAC, 0.999, 1.0, mask_E);
    if (E.empty() || cv::countNonZero(mask_E) < 20) return false;

    // --- 3. SVD correction (enforce sigma1=sigma2, sigma3=0) ---
    cv::Mat w, u, vt;
    cv::SVD::compute(E, w, u, vt);
    double singular_mean = (w.at<double>(0) + w.at<double>(1)) / 2.0;
    cv::Mat w_correct = (cv::Mat_<double>(3,3) << singular_mean,0,0, 0,singular_mean,0, 0,0,0);
    cv::Mat E_correct = u * w_correct * vt;

    // --- 4. recover pose ---
    cv::Mat R_cv, t_cv, mask_pose;
    cv::recoverPose(E_correct, pts1, pts2, vo.K, R_cv, t_cv, mask_pose);

    std::vector<cv::Point2f> inlier_pts1, inlier_pts2;
    for (int i = 0; i < mask_pose.rows; i++) {
        if (mask_pose.at<uchar>(i)) {
            inlier_pts1.push_back(pts1[i]);
            inlier_pts2.push_back(pts2[i]);
        }
    }
    std::cout << "[init] pose inliers: " << inlier_pts1.size() << std::endl;
    if (inlier_pts1.size() < 20) return false;

    // --- 5. triangulation ---
    cv::Mat P1 = vo.K * (cv::Mat_<double>(3,4) << 1,0,0,0, 0,1,0,0, 0,0,1,0);
    cv::Mat Rt; cv::hconcat(R_cv, t_cv, Rt);
    cv::Mat P2 = vo.K * Rt;
    cv::Mat pts4D;
    cv::triangulatePoints(P1, P2, inlier_pts1, inlier_pts2, pts4D);

    // --- 6. median depth for adaptive outlier threshold ---
    std::vector<double> valid_Z;
    for (int i = 0; i < pts4D.cols; i++) {
        cv::Mat c = pts4D.col(i);
        float W = c.at<float>(3);
        if (std::abs(W) < 1e-6f) continue;
        double Z = c.at<float>(2) / W;
        if (Z > 0) valid_Z.push_back(Z);
    }
    if (valid_Z.size() < 10) return false;
    std::sort(valid_Z.begin(), valid_Z.end());
    double median_Z = valid_Z[valid_Z.size()/2];
    double max_Z = median_Z * 10.0;

    // --- 7. quality filtering ---
    std::vector<cv::Point3f> points3D;
    cv::Mat desc_inliers;
    for (int i = 0; i < pts4D.cols; i++) {
        cv::Mat c = pts4D.col(i);
        float W = c.at<float>(3);
        if (std::abs(W) < 1e-6f) continue;
        double X = c.at<float>(0)/W, Y = c.at<float>(1)/W, Z = c.at<float>(2)/W;

        cv::Mat Xc2 = Rt * (cv::Mat_<double>(4,1) << X,Y,Z,1.0);
        if (Z <= 0 || Z > max_Z || Xc2.at<double>(2) <= 0) continue;

        // reprojection error
        cv::Mat Xh = (cv::Mat_<double>(4,1) << X,Y,Z,1.0);
        cv::Mat pr1 = P1 * Xh, pr2 = P2 * Xh;
        double u1 = pr1.at<double>(0)/pr1.at<double>(2);
        double v1 = pr1.at<double>(1)/pr1.at<double>(2);
        double u2 = pr2.at<double>(0)/pr2.at<double>(2);
        double v2 = pr2.at<double>(1)/pr2.at<double>(2);
        double e1 = std::sqrt(std::pow(u1-inlier_pts1[i].x,2)+std::pow(v1-inlier_pts1[i].y,2));
        double e2 = std::sqrt(std::pow(u2-inlier_pts2[i].x,2)+std::pow(v2-inlier_pts2[i].y,2));
        if (std::max(e1,e2) > 3.0) continue;

        points3D.push_back(cv::Point3f(X,Y,Z));
        desc_inliers.push_back(desc2.row(good_matches[i].trainIdx));
    }
    std::cout << "[init] map: " << points3D.size() << " points" << std::endl;
    if (points3D.size() < 20) return false;

    // --- 8. write state ---
    vo.R_cw = R_cv.clone();
    vo.t_cw = t_cv.clone();
    vo.map_points = points3D;
    vo.map_descriptors = desc_inliers.clone();

    vo.prev_img = img2.clone();
    vo.prev_kp = kp2;
    vo.prev_desc = desc2.clone();
    vo.R_prev = R_cv.clone();
    vo.t_prev = t_cv.clone();

    vo.kf_img = img2.clone();
    vo.kf_kp = kp2;
    vo.kf_desc = desc2.clone();
    vo.R_kf = R_cv.clone();
    vo.t_kf = t_cv.clone();

    return true;
}

} // namespace mini_vo
