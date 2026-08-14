#include "mini_vo/tracker.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <utility>

namespace mini_vo {
namespace {

struct TriangulatedCandidate {
    cv::Point3d position;
    int reference_feature_index = 0;
    int current_feature_index = 0;
};

}  // namespace

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
    // --- 1. ORB 特征提取（2000 点） ---
    auto orb = cv::ORB::create(2000);
    std::vector<cv::KeyPoint> kp;
    cv::Mat desc;
    orb->detectAndCompute(img, cv::Mat(), kp, desc);

    const TrackingMapSnapshot snapshot = state.map.trackingSnapshot();
    if (desc.empty() || !snapshot.valid() ||
        snapshot.descriptors.rows < 2) {
        return false;
    }

    // --- 2. 当前帧描述子 → 地图描述子匹配 ---
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn;
    matcher.knnMatch(desc, snapshot.descriptors, knn, 2);

    // --- 3. ratio test + 收集 2D-3D 对应点 ---
    std::vector<cv::Point2f> pts2D;
    std::vector<cv::Point3f> pts3D;
    for (const auto& m : knn) {
        if (m[0].distance < 0.8f * m[1].distance) {
            int idx = m[0].trainIdx;
            if (idx >= static_cast<int>(snapshot.points.size())) continue;
            pts2D.push_back(kp[m[0].queryIdx].pt);
            pts3D.push_back(snapshot.points[idx]);
        }
    }

    if (pts2D.size() < 10) {
        std::cout << "[track]"
		  << "frame: " << state.frame_count
		  << "matches: " << pts2D.size()
		  << "inliner = 0"
		  << "result=NOT_ENOUGH_MATCHES"
		  << "map_points= " << snapshot.points.size()
		  << std::endl;

        return false;
    }

    // --- 4. PnP 位姿解算（solvePnPRansac + 返回值检查） ---
    cv::Mat rvec, tvec, inliers;
    bool ok = cv::solvePnPRansac(pts3D, pts2D, K, cv::Mat(),
                                  rvec, tvec, false, 100, 6.0, 0.99, inliers);

    int n_inliers = ok && !inliers.empty() ? inliers.rows : 0;
    std::cout << "[TRACK]"
          << " frame=" << state.frame_count
          << " matches=" << pts2D.size()
          << " inliers=" << n_inliers
          << " map_points=" << snapshot.points.size()
          << " result="
          << ((!ok || n_inliers < 10) ? "PNP_FAILED" : "OK")
          << std::endl;
    if (!ok || inliers.empty() || n_inliers < 10) return false;

    cv::Rodrigues(rvec, R_out);
    t_out = tvec;

    // --- 5. 可选调试输出 ---
    if (out_kp)    *out_kp = std::move(kp);
    if (out_pts2D) *out_pts2D = pts2D;
    if (out_pts3D) *out_pts3D = pts3D;
    if (out_inliers) *out_inliers = inliers.clone();

    return true;
}

TriangulationReport triangulateNewPoints(
    VOSystem& vo, const cv::Mat& img,
    const std::vector<cv::KeyPoint>& kp,
    const cv::Mat& desc,
    const cv::Mat& R_cur, const cv::Mat& t_cur)
{
    TriangulationReport report;
    if (vo.kf_desc.empty() || desc.empty()) {
        return report;
    }
    // --- 1. 关键帧 → 当前帧匹配 ---
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
    std::cout << "[tri] 匹配: " << good.size() << std::endl;
    if (good.size() < 30) return report;

    // --- 2. 投影矩阵 ---
    cv::Mat Rt1, Rt2;
    cv::hconcat(vo.R_kf, vo.t_kf, Rt1);
    cv::hconcat(R_cur, t_cur, Rt2);
    cv::Mat P1 = vo.K * Rt1, P2 = vo.K * Rt2;

    cv::Mat pts4D;
    cv::triangulatePoints(P1, P2, pts_kf, pts_cur, pts4D);

    // --- 3. 光心坐标（视差角检查用） ---
    cv::Mat O1, O2;
    {
        cv::Mat Rkf_t;
        cv::transpose(vo.R_kf, Rkf_t);
        O1 = -Rkf_t * vo.t_kf;
        cv::Mat Rc_t;
        cv::transpose(R_cur, Rc_t);
        O2 = -Rc_t * t_cur;
    }

    // --- 4. 自适应距离阈值（中值 ×5，下界 ×0.1） ---
    std::vector<double> zs;
    const TrackingMapSnapshot snapshot = vo.map.trackingSnapshot();
    for (const auto& p : snapshot.points) {
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

    // --- 5. 质量过滤 ---
    int added = 0, rej_depth = 0, rej_parallax = 0, rej_dist = 0, rej_reproj = 0;
    std::vector<TriangulatedCandidate> candidates;
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

        // 视差角检查（cos > 0.999 ≈ 视差角 < 2.5°，三角化极不稳定，剔除）
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

        // 重投影误差检查
        cv::Mat pr1 = P1 * Xw, pr2 = P2 * Xw;
        double u1 = pr1.at<double>(0)/pr1.at<double>(2);
        double v1 = pr1.at<double>(1)/pr1.at<double>(2);
        double u2 = pr2.at<double>(0)/pr2.at<double>(2);
        double v2 = pr2.at<double>(1)/pr2.at<double>(2);
        double e1 = std::sqrt(std::pow(u1-pts_kf[i].x,2)+std::pow(v1-pts_kf[i].y,2));
        double e2 = std::sqrt(std::pow(u2-pts_cur[i].x,2)+std::pow(v2-pts_cur[i].y,2));
        if (std::max(e1,e2) > 3.0) { rej_reproj++; continue; }

        candidates.push_back(
            {{X, Y, Z}, good[i].queryIdx, good[i].trainIdx});
    }
    if (!candidates.empty()) {
        Map updated_map = vo.map;
        KeyFrame current_keyframe;
        current_keyframe.id = vo.next_keyframe_id;
        current_keyframe.image = img;
        current_keyframe.keypoints = kp;
        current_keyframe.descriptors = desc;
        current_keyframe.Rcw = R_cur;
        current_keyframe.tcw = t_cur;
        if (!updated_map.addKeyFrame(current_keyframe)) {
            return report;
        }

        for (std::size_t index = 0; index < candidates.size(); ++index) {
            const TriangulatedCandidate& candidate = candidates[index];
            MapPoint point;
            point.id = vo.next_map_point_id + index;
            point.position_world = candidate.position;
            point.descriptor =
                desc.row(candidate.current_feature_index);
            if (!updated_map.addMapPoint(point)) {
                return report;
            }

            Observation reference_observation;
            reference_observation.keyframe_id = vo.reference_keyframe_id;
            reference_observation.map_point_id = point.id;
            reference_observation.feature_index =
                candidate.reference_feature_index;
            reference_observation.pixel =
                vo.kf_kp[candidate.reference_feature_index].pt;
            reference_observation.octave =
                vo.kf_kp[candidate.reference_feature_index].octave;

            Observation current_observation;
            current_observation.keyframe_id = current_keyframe.id;
            current_observation.map_point_id = point.id;
            current_observation.feature_index =
                candidate.current_feature_index;
            current_observation.pixel =
                kp[candidate.current_feature_index].pt;
            current_observation.octave =
                kp[candidate.current_feature_index].octave;
            if (!updated_map.addObservation(reference_observation) ||
                !updated_map.addObservation(current_observation)) {
                return report;
            }
        }
        if (!updated_map.validate()) {
            return report;
        }
        vo.map = std::move(updated_map);
        added = static_cast<int>(candidates.size());
        report.keyframe_inserted = true;
        report.keyframe_id = current_keyframe.id;
        report.points_added = candidates.size();
        ++vo.next_keyframe_id;
        vo.next_map_point_id += candidates.size();
    }
    std::cout << "[tri] 新增:" << added
              << " 拒绝(深度:" << rej_depth
              << " 视差:" << rej_parallax
              << " 距离:" << rej_dist
              << " 重投影:" << rej_reproj
              << ") 总计:" << vo.map.mapPointCount() << std::endl;
    return report;
}

} // namespace mini_vo
