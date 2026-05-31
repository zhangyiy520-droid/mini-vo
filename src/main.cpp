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

/** Load images from a directory (for EuRoC-style datasets). */
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

/** Load images from TUM rgb.txt. */
std::vector<std::string> loadTUM(const std::string& rgb_file,
                                  const std::string& data_dir,
                                  int max_frames = 200)
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
        std::string timestamp, relpath;
        if (iss >> timestamp >> relpath) {
            files.push_back(data_dir + "/" + relpath);
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
              << "  " << prog << " euroc <image_dir> [max_frames]\n"
              << "  " << prog << " tum  <data_dir>  [max_frames]\n"
              << "\n"
              << "  euroc: load *.png/*.jpg from a directory\n"
              << "  tum:   load from rgb.txt inside data_dir\n"
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
    int max_frames = (argc >= 4) ? std::stoi(argv[3]) : 200;

    std::vector<std::string> files;
    if (mode == "euroc") {
        files = loadDirectory(path, max_frames);
    } else if (mode == "tum") {
        files = loadTUM(path + "/rgb.txt", path, max_frames);
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
    // Default: EuRoC MH_01 (cam0). Override via command line or config.
    cv::Mat K = (cv::Mat_<double>(3,3) <<
        458.654, 0, 367.215,
        0, 457.296, 248.375,
        0, 0, 1);
    std::cout << "K:\n" << K << std::endl;

    // --- run VO ---
    mini_vo::VOSystem vo;
    vo.K = K.clone();

    for (size_t i = 0; i < files.size(); i++) {
        cv::Mat img = cv::imread(files[i], cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            std::cerr << "fail: " << files[i] << std::endl;
            continue;
        }

        if (i % 20 == 0 || i == files.size() - 1) {
            std::cout << "\n==== frame " << (i+1) << "/" << files.size()
                      << " ====" << std::endl;
        }
        mini_vo::processFrame(vo, img);
    }

    // --- summary ---
    std::cout << "\n========== Done ==========" << std::endl;
    std::cout << "Map: " << vo.map_points.size() << " points" << std::endl;
    std::cout << "Trajectory: " << vo.trajectory_R.size() << " frames" << std::endl;

    mini_vo::saveTrajectory("trajectory.txt", vo.trajectory_R, vo.trajectory_t);
    mini_vo::saveMapPLY("map.ply", vo.map_points);

    return 0;
}
