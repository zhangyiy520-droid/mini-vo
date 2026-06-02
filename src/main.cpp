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
// Dataset loaders
// ============================================================

std::vector<std::string> loadDirectory(const std::string& dir,
                                        int max_frames = 200)
{
    std::vector<std::string> files;
    if (!fs::is_directory(dir)) {
        std::cerr << "Not a directory: " << dir << std::endl;
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
    std::cout << "Loaded " << files.size() << " images from " << dir << std::endl;
    return files;
}

std::vector<std::string> loadTUM(const std::string& rgb_file,
                                  const std::string& data_dir,
                                  int max_frames,
                                  std::vector<double>& out_ts)
{
    std::vector<std::string> files;
    std::ifstream f(rgb_file);
    if (!f.is_open()) {
        std::cerr << "Cannot open " << rgb_file << std::endl;
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
    std::cout << "Loaded " << files.size() << " images from " << rgb_file << std::endl;
    return files;
}

// ============================================================
void printUsage(const char* prog)
{
    std::cout << "Usage:\n"
              << "  " << prog << " euroc <image_dir> [max_frames] [--viz]\n"
              << "  " << prog << " tum  <data_dir>  [max_frames] [--viz]\n"
              << "\n"
              << "  euroc: load *.png/*.jpg from a directory\n"
              << "  tum:   load from rgb.txt inside data_dir\n"
              << "  --viz: save debug images to debug/ directory\n"
              << "  default max_frames = 200\n";
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
        std::cerr << "Unknown mode: " << mode << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    if (files.empty()) {
        std::cerr << "No images found!" << std::endl;
        return 1;
    }

    // --- camera intrinsics ---
    cv::Mat K;
    if (mode == "tum") {
        K = (cv::Mat_<double>(3,3) <<
            517.3, 0, 318.6,
            0, 516.5, 255.3,
            0, 0, 1);
    } else {
        K = (cv::Mat_<double>(3,3) <<
            458.654, 0, 367.215,
            0, 457.296, 248.375,
            0, 0, 1);
    }
    std::cout << "K:\n" << K << std::endl;

    if (visualize) std::cout << "Visualization: ON → debug/\n";

    // --- run VO ---
    mini_vo::VOSystem vo;
    vo.K = K.clone();

    for (size_t i = 0; i < files.size(); i++) {
        cv::Mat img = cv::imread(files[i], cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            std::cerr << "fail: " << files[i] << std::endl;
            continue;
        }
        if (i % 50 == 0 || i == files.size() - 1) {
            std::cout << "\n==== frame " << (i+1) << "/" << files.size()
                      << " ====" << std::endl;
        }
        double ts = timestamps.empty() ? (double)i : timestamps[i];
        mini_vo::processFrame(vo, img, visualize, ts);
    }

    // --- summary ---
    std::cout << "\n========== Done ==========" << std::endl;
    std::cout << "Map: " << vo.map_points.size() << " points" << std::endl;
    std::cout << "Trajectory: " << vo.trajectory_R.size() << "/" << files.size()
              << " frames (" << (100.0 * vo.trajectory_R.size() / files.size())
              << "%)" << std::endl;

    if (!timestamps.empty() && !vo.trajectory_ts.empty()) {
        mini_vo::saveTrajectoryTUM("trajectory.txt", vo.trajectory_ts,
                                    vo.trajectory_R, vo.trajectory_t);
    } else {
        mini_vo::saveTrajectory("trajectory.txt", vo.trajectory_R, vo.trajectory_t);
    }
    mini_vo::saveMapPLY("map.ply", vo.map_points);

    if (visualize) std::cout << "Debug images saved to debug/" << std::endl;

    return 0;
}
