#include "mini_vo/vo_system.h"
#include "mini_vo/initializer.h"
#include "mini_vo/tracker.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

namespace mini_vo {

void processFrame(VOSystem& vo, const cv::Mat& img)
{
    vo.frame_count++;

    auto orb = cv::ORB::create(2000);
    std::vector<cv::KeyPoint> kp;
    cv::Mat desc;
    orb->detectAndCompute(img, cv::Mat(), kp, desc);

    // ========================================================
    // UNINITIALIZED
    // ========================================================
    if (vo.status == VOStatus::UNINITIALIZED) {
        if (vo.prev_img.empty()) {
            vo.prev_img = img.clone();
            vo.prev_kp = kp;
            vo.prev_desc = desc.clone();
            vo.trajectory_R.push_back(cv::Mat::eye(3,3,CV_64F));
            vo.trajectory_t.push_back(cv::Mat::zeros(3,1,CV_64F));
            std::cout << "[frame " << vo.frame_count << "] waiting" << std::endl;
            return;
        }
        std::cout << "[frame " << vo.frame_count << "] init... ";
        if (initialize(vo, vo.prev_img, img, vo.prev_kp, vo.prev_desc, kp, desc)) {
            vo.status = VOStatus::TRACKING;
            vo.last_keyframe = vo.frame_count;
            vo.trajectory_R.push_back(vo.R_cw.clone());
            vo.trajectory_t.push_back(vo.t_cw.clone());
            std::cout << "[frame " << vo.frame_count << "] INIT OK" << std::endl;
            return;
        }
        vo.prev_img = img.clone();
        vo.prev_kp = kp;
        vo.prev_desc = desc.clone();
        std::cout << "FAILED" << std::endl;
        return;
    }

    // ========================================================
    // TRACKING
    // ========================================================
    if (vo.status == VOStatus::TRACKING) {
        cv::Mat R, t;
        if (track(vo, img, kp, desc, R, t)) {
            vo.R_cw = R.clone();
            vo.t_cw = t.clone();
            vo.trajectory_R.push_back(R.clone());
            vo.trajectory_t.push_back(t.clone());

            // keyframe insertion every 5 frames
            if (vo.frame_count - vo.last_keyframe >= 5) {
                triangulateNewPoints(vo, img, kp, desc, R, t);
                vo.kf_img = img.clone();
                vo.kf_kp = kp;
                vo.kf_desc = desc.clone();
                vo.R_kf = R.clone();
                vo.t_kf = t.clone();
                vo.last_keyframe = vo.frame_count;
                std::cout << "[frame " << vo.frame_count << "] keyframe" << std::endl;
            }

            vo.prev_img = img.clone();
            vo.prev_kp = kp;
            vo.prev_desc = desc.clone();
            vo.R_prev = R.clone();
            vo.t_prev = t.clone();
            return;
        }

        vo.status = VOStatus::LOST;
        std::cout << "[frame " << vo.frame_count << "] LOST" << std::endl;
        return;
    }

    // ========================================================
    // LOST
    // ========================================================
    if (vo.status == VOStatus::LOST) {
        if (initialize(vo, vo.prev_img, img, vo.prev_kp, vo.prev_desc, kp, desc)) {
            vo.status = VOStatus::TRACKING;
            vo.last_keyframe = vo.frame_count;
            vo.trajectory_R.push_back(vo.R_cw.clone());
            vo.trajectory_t.push_back(vo.t_cw.clone());
            std::cout << "[frame " << vo.frame_count << "] RE-INIT OK" << std::endl;
            return;
        }
        vo.prev_img = img.clone();
        vo.prev_kp = kp;
        vo.prev_desc = desc.clone();
        std::cout << "[frame " << vo.frame_count << "] re-init failed" << std::endl;
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
        // rotation matrix → quaternion
        double qw = std::sqrt(1.0 + R.at<double>(0,0)
                              + R.at<double>(1,1)
                              + R.at<double>(2,2)) / 2.0;
        double qx = (R.at<double>(2,1) - R.at<double>(1,2)) / (4.0 * qw);
        double qy = (R.at<double>(0,2) - R.at<double>(2,0)) / (4.0 * qw);
        double qz = (R.at<double>(1,0) - R.at<double>(0,1)) / (4.0 * qw);
        double tx = ts[i].at<double>(0);
        double ty = ts[i].at<double>(1);
        double tz = ts[i].at<double>(2);
        // TUM format: timestamp tx ty tz qx qy qz qw
        f << i << " " << tx << " " << ty << " " << tz << " "
          << qx << " " << qy << " " << qz << " " << qw << "\n";
    }
    f.close();
    std::cout << "Saved " << path << " (" << Rs.size() << " poses)" << std::endl;
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
        // color by depth
        int r = std::min(255, (int)(std::abs(p.z) * 20));
        f << p.x << " " << p.y << " " << p.z << " "
          << r << " " << (255 - r) << " 0\n";
    }
    f.close();
    std::cout << "Saved " << path << " (" << pts.size() << " points)" << std::endl;
}

} // namespace mini_vo
