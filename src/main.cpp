#include "mini_vo/vo_system.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <filesystem>

#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

// ============================================================
// 数据集加载器
// ============================================================

/** 从目录加载图像（EuRoC 模式） */
std::vector<std::string> loadDirectory(const std::string& dir,
                                        int max_frames = 200)
{
    std::vector<std::string> files;
    if (!fs::is_directory(dir)) {
        std::cerr << "非目录: " << dir << std::endl;
        return files;
    }
    for (const auto& entry : fs::directory_iterator(dir)) {
        std::string ext = entry.path().extension().string();
        if (ext == ".png" || ext == ".jpg" || ext == ".jpeg")
            files.push_back(entry.path().string());
    }
    std::sort(files.begin(), files.end());
    if (!files.empty() && (int)files.size() > max_frames)
        files.resize(max_frames);
    std::cout << "加载 " << files.size() << " 张图像 (" << dir << ")" << std::endl;
    return files;
}

/** 从 rgb.txt 加载图像（TUM 模式），同步提取时间戳 */
std::vector<std::string> loadTUM(const std::string& rgb_file,
                                  const std::string& data_dir,
                                  int max_frames,
                                  std::vector<double>& out_ts)
{
    std::vector<std::string> files;
    std::ifstream f(rgb_file);
    if (!f.is_open()) {
        std::cerr << "无法打开 " << rgb_file << std::endl;
        return files;
    }
    std::string line;
    int count = 0;
    while (std::getline(f, line) && count < max_frames) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        double timestamp;
        std::string relpath;
        if (iss >> timestamp >> relpath) {
            files.push_back(data_dir + "/" + relpath);
            out_ts.push_back(timestamp);
            count++;
        }
    }
    std::cout << "加载 " << files.size() << " 张图像 (" << rgb_file << ")" << std::endl;
    return files;
}

// ============================================================
void printUsage(const char* prog)
{
    std::cout << "用法:\n"
              << "  " << prog << " euroc <图像目录> [帧数] [--viz]\n"
              << "  " << prog << " tum  <数据集目录> [帧数] [--viz]\n"
              << "\n"
              << "  euroc: 从目录加载 *.png/*.jpg\n"
              << "  tum:   从数据集目录下的 rgb.txt 加载\n"
              << "  --viz: 保存调试图片到 debug/ 目录\n"
              << "  默认帧数 = 200\n";
}

// ============================================================
int main(int argc, char** argv)
{
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    std::string mode = argv[1];
    std::string path = argv[2];
    int max_frames = 200;
    bool visualize = false;

    for (int i = 3; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--viz") {
            visualize = true;
        } else {
            max_frames = std::stoi(arg);
        }
    }

    std::vector<std::string> files;
    std::vector<double> timestamps;

    if (mode == "euroc") {
        files = loadDirectory(path, max_frames);
    } else if (mode == "tum") {
        files = loadTUM(path + "/rgb.txt", path, max_frames, timestamps);
    } else {
        std::cerr << "未知模式: " << mode << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    if (files.empty()) {
        std::cerr << "未找到图像!" << std::endl;
        return 1;
    }

    // --- 相机内参 ---
    cv::Mat K;
    if (mode == "tum") {
        // TUM RGB-D fr1 (Kinect)
        K = (cv::Mat_<double>(3,3) <<
            517.3, 0, 318.6,
            0, 516.5, 255.3,
            0, 0, 1);
    } else {
        // EuRoC MH_01 cam0
        K = (cv::Mat_<double>(3,3) <<
            458.654, 0, 367.215,
            0, 457.296, 248.375,
            0, 0, 1);
    }
    std::cout << "K:\n" << K << std::endl;

    if (visualize) std::cout << "可视化: 开启 → debug/\n";

    // --- 运行 VO ---
    mini_vo::VOSystem vo;
    vo.K = K.clone();

    for (size_t i = 0; i < files.size(); i++) {
        cv::Mat img = cv::imread(files[i], cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            std::cerr << "失败: " << files[i] << std::endl;
            continue;
        }
        if (i % 50 == 0 || i == files.size() - 1) {
            std::cout << "\n==== 帧 " << (i+1) << "/" << files.size()
                      << " ====" << std::endl;
        }
        double ts = timestamps.empty() ? (double)i : timestamps[i];
        mini_vo::processFrame(vo, img, visualize, ts);
    }

    // --- 汇总 ---
    std::cout << "\n========== 完成 ==========" << std::endl;
    std::cout << "地图点: " << vo.map_points.size() << std::endl;
    std::cout << "轨迹: " << vo.trajectory_R.size() << "/" << files.size()
              << " 帧 (" << (100.0 * vo.trajectory_R.size() / files.size())
              << "%)" << std::endl;

    if (!timestamps.empty() && !vo.trajectory_ts.empty()) {
        mini_vo::saveTrajectoryTUM("trajectory.txt", vo.trajectory_ts,
                                    vo.trajectory_R, vo.trajectory_t);
    } else {
        mini_vo::saveTrajectory("trajectory.txt", vo.trajectory_R, vo.trajectory_t);
    }
    mini_vo::saveMapPLY("map.ply", vo.map_points);

    if (visualize) std::cout << "调试图片已保存到 debug/" << std::endl;

    return 0;
}
