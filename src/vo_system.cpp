#include "mini_vo/vo_system.h"
#include "mini_vo/initializer.h"
#include "mini_vo/tracker.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <filesystem>

namespace mini_vo {

namespace fs = std::filesystem;

// ============================================================
// Debug visualization helpers
// ============================================================

static void saveDebugImage(const cv::Mat& img, int frame_id,
                           const std::string& tag,
                           const std::string& dir = "debug")
{
    fs::create_directories(dir);
    std::ostringstream ss;
    ss << dir << "/frame_" << std::setfill('0') << std::setw(5)
       << frame_id << "_" << tag << ".png";
    cv::imwrite(ss.str(), img);
}

// ============================================================
// processFrame
// ============================================================

void processFrame(VOSystem& vo, const cv::Mat& img, bool visualize, double timestamp)
{
    vo.frame_count++;

    // ========================================================
    // UNINITIALIZED
    // ========================================================
    if (vo.status == VOStatus::UNINITIALIZED) {
        auto orb = cv::ORB::create(2000);
        std::vector<cv::KeyPoint> kp;
        cv::Mat desc;
        orb->detectAndCompute(img, cv::Mat(), kp, desc);

        if (visualize) {
            cv::Mat kp_vis;
            cv::drawKeypoints(img, kp, kp_vis, cv::Scalar(0, 255, 0),
                              cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            saveDebugImage(kp_vis, vo.frame_count, "orb_uninit");
        }

        if (vo.prev_img.empty()) {
            vo.prev_img = img.clone();
            vo.prev_kp = kp;
            vo.prev_desc = desc.clone();
            vo.trajectory_R.push_back(cv::Mat::eye(3,3,CV_64F));
            vo.trajectory_t.push_back(cv::Mat::zeros(3,1,CV_64F));
            vo.trajectory_ts.push_back(timestamp);
            std::cout << "[frame " << vo.frame_count << "] waiting "
                      << "orb:" << kp.size() << std::endl;
            return;
        }

        std::cout << "[frame " << vo.frame_count << "] init... ";
        if (initialize(vo, vo.prev_img, img, vo.prev_kp, vo.prev_desc, kp, desc)) {
            vo.status = VOStatus::TRACKING;
            vo.last_keyframe = vo.frame_count;
            vo.trajectory_R.push_back(vo.R_cw.clone());
            vo.trajectory_t.push_back(vo.t_cw.clone());
            vo.trajectory_ts.push_back(timestamp);
            std::cout << "[frame " << vo.frame_count << "] INIT OK "
                      << "map:" << vo.map_points.size() << std::endl;

            if (visualize) {
                cv::Mat init_vis;
                cv::cvtColor(img, init_vis, cv::COLOR_GRAY2BGR);
                cv::drawKeypoints(init_vis, kp, init_vis, cv::Scalar(0, 255, 0),
                                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                std::string label = "INIT OK  map:" + std::to_string(vo.map_points.size());
                cv::putText(init_vis, label, cv::Point(10, 25),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                saveDebugImage(init_vis, vo.frame_count, "init_ok");
            }
            return;
        }
        vo.prev_img = img.clone();
        vo.prev_kp = kp;
        vo.prev_desc = desc.clone();
        std::cout << "FAILED" << std::endl;
        return;
    }

    // ========================================================
    // TRACKING — 3D-2D PnP with debug viz
    // ========================================================
    if (vo.status == VOStatus::TRACKING) {
        cv::Mat R, t;
        std::vector<cv::KeyPoint> dbg_kp;
        std::vector<cv::Point2f> dbg_pts2D;
        std::vector<cv::Point3f> dbg_pts3D;
        cv::Mat dbg_inliers;

        bool ok = false;
        if (visualize) {
            ok = track(img, vo.K, vo, R, t, &dbg_kp, &dbg_pts2D, &dbg_pts3D, &dbg_inliers);
        } else {
            ok = track(img, vo.K, vo, R, t);
        }

        if (ok) {
            vo.R_cw = R.clone();
            vo.t_cw = t.clone();
            vo.trajectory_R.push_back(R.clone());
            vo.trajectory_t.push_back(t.clone());
            vo.trajectory_ts.push_back(timestamp);

            if (visualize && !dbg_kp.empty()) {
                cv::Mat vis;
                cv::cvtColor(img, vis, cv::COLOR_GRAY2BGR);
                for (const auto& k : dbg_kp)
                    cv::circle(vis, k.pt, 2, cv::Scalar(255, 0, 0), -1);
                for (const auto& p : dbg_pts2D)
                    cv::circle(vis, p, 3, cv::Scalar(0, 255, 255), -1);
                if (!dbg_inliers.empty()) {
                    for (int j = 0; j < dbg_inliers.rows; j++) {
                        int idx = dbg_inliers.at<int>(j);
                        if (idx < (int)dbg_pts2D.size())
                            cv::circle(vis, dbg_pts2D[idx], 5, cv::Scalar(0, 255, 0), 2);
                    }
                }
                int n_inl = dbg_inliers.empty() ? 0 : dbg_inliers.rows;
                std::string label = "track " + std::to_string(vo.frame_count) +
                                    "  ORB:" + std::to_string(dbg_kp.size()) +
                                    "  match:" + std::to_string(dbg_pts2D.size()) +
                                    "  inlier:" + std::to_string(n_inl);
                cv::putText(vis, label, cv::Point(10, 25),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
                saveDebugImage(vis, vo.frame_count, "track");
            }

            if (vo.frame_count - vo.last_keyframe >= 1) {
                auto orb = cv::ORB::create(2000);
                std::vector<cv::KeyPoint> kp_tri;
                cv::Mat desc_tri;
                orb->detectAndCompute(img, cv::Mat(), kp_tri, desc_tri);
                triangulateNewPoints(vo, img, kp_tri, desc_tri, R, t);
                vo.last_keyframe = vo.frame_count;
            }

            return;
        }

        vo.status = VOStatus::LOST;
        if (visualize) {
            cv::Mat lost_vis;
            cv::cvtColor(img, lost_vis, cv::COLOR_GRAY2BGR);
            cv::putText(lost_vis, "LOST", cv::Point(10, 25),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            saveDebugImage(lost_vis, vo.frame_count, "lost");
        }
        return;
    }

    // ========================================================
    // LOST — retry init with growing parallax
    // ========================================================
    if (vo.status == VOStatus::LOST) {
        auto orb = cv::ORB::create(2000);
        std::vector<cv::KeyPoint> kp;
        cv::Mat desc;
        orb->detectAndCompute(img, cv::Mat(), kp, desc);

        if (initialize(vo, vo.kf_img, img, vo.kf_kp, vo.kf_desc, kp, desc)) {
            vo.status = VOStatus::TRACKING;
            vo.last_keyframe = vo.frame_count;
            vo.trajectory_R.push_back(vo.R_cw.clone());
            vo.trajectory_t.push_back(vo.t_cw.clone());
            vo.trajectory_ts.push_back(timestamp);

            if (visualize) {
                cv::Mat reinit_vis;
                cv::cvtColor(img, reinit_vis, cv::COLOR_GRAY2BGR);
                cv::drawKeypoints(reinit_vis, kp, reinit_vis, cv::Scalar(0, 255, 0),
                                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                cv::putText(reinit_vis, "RE-INIT OK", cv::Point(10, 25),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                saveDebugImage(reinit_vis, vo.frame_count, "reinit_ok");
            }
            return;
        }

        if (visualize) {
            cv::Mat fail_vis;
            cv::cvtColor(img, fail_vis, cv::COLOR_GRAY2BGR);
            cv::putText(fail_vis, "re-init FAIL", cv::Point(10, 25),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            saveDebugImage(fail_vis, vo.frame_count, "reinit_fail");
        }
    }
}

// ============================================================
void saveTrajectory(const std::string& path,
                    const std::vector<cv::Mat>& Rs,
                    const std::vector<cv::Mat>& ts)
{
    std::ofstream f(path);
    f << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < Rs.size(); i++) {
        cv::Mat R = Rs[i];
        double qw = std::sqrt(1.0 + R.at<double>(0,0)
                              + R.at<double>(1,1)
                              + R.at<double>(2,2)) / 2.0;
        double qx = (R.at<double>(2,1) - R.at<double>(1,2)) / (4.0 * qw);
        double qy = (R.at<double>(0,2) - R.at<double>(2,0)) / (4.0 * qw);
        double qz = (R.at<double>(1,0) - R.at<double>(0,1)) / (4.0 * qw);
        double tx = ts[i].at<double>(0);
        double ty = ts[i].at<double>(1);
        double tz = ts[i].at<double>(2);
        f << i << " " << tx << " " << ty << " " << tz << " "
          << qx << " " << qy << " " << qz << " " << qw << "\n";
    }
    f.close();
    std::cout << "Saved " << path << " (" << Rs.size() << " poses)" << std::endl;
}

void saveTrajectoryTUM(const std::string& path,
                        const std::vector<double>& timeStamps,
                        const std::vector<cv::Mat>& Rs,
                        const std::vector<cv::Mat>& ts)
{
    std::ofstream f(path);
    f << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < Rs.size(); i++) {
        cv::Mat R = Rs[i];
        double qw = std::sqrt(1.0 + R.at<double>(0,0)
                              + R.at<double>(1,1)
                              + R.at<double>(2,2)) / 2.0;
        double qx = (R.at<double>(2,1) - R.at<double>(1,2)) / (4.0 * qw);
        double qy = (R.at<double>(0,2) - R.at<double>(2,0)) / (4.0 * qw);
        double qz = (R.at<double>(1,0) - R.at<double>(0,1)) / (4.0 * qw);
        double tx = ts[i].at<double>(0);
        double ty = ts[i].at<double>(1);
        double tz = ts[i].at<double>(2);
        f << timeStamps[i] << " " << tx << " " << ty << " " << tz << " "
          << qx << " " << qy << " " << qz << " " << qw << "\n";
    }
    f.close();
    std::cout << "Saved " << path << " (" << Rs.size() << " poses, TUM timestamps)" << std::endl;
}

void saveMapPLY(const std::string& path, const std::vector<cv::Point3f>& pts)
{
    if (pts.empty()) return;
    std::ofstream f(path);
    f << "ply\nformat ascii 1.0\n";
    f << "element vertex " << pts.size() << "\n";
    f << "property float x\nproperty float y\nproperty float z\n";
    f << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    f << "end_header\n";
    for (const auto& p : pts) {
        int r = std::min(255, (int)(std::abs(p.z) * 20));
        f << p.x << " " << p.y << " " << p.z << " "
          << r << " " << (255 - r) << " 0\n";
    }
    f.close();
    std::cout << "Saved " << path << " (" << pts.size() << " points)" << std::endl;
}

} // namespace mini_vo
