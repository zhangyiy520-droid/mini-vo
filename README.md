# Mini VO

A lightweight monocular Visual Odometry pipeline built from scratch with OpenCV.

**Pipeline:** ORB extraction → Ratio-test matching → Essential matrix → recoverPose → Triangulation → Frame-to-frame tracking → Keyframe management → TUM-format trajectory output.

## Quick Start

```bash
# Build
mkdir build && cd build
cmake .. && make -j$(nproc)

# Run on EuRoC dataset
./vo_bin euroc /path/to/MH_01_easy/mav0/cam0/data 50

# Run on TUM RGB-D dataset
./vo_bin tum /path/to/rgbd_dataset_freiburg1_xyz 200
```

**Output:**
- `trajectory.txt` — TUM format (timestamp tx ty tz qx qy qz qw)
- `map.ply` — sparse 3D map points (colored by depth)

## Project Structure

```
mini-vo/
├── include/mini_vo/     # Public headers
│   ├── types.h              # VOSystem struct, VOStatus enum
│   ├── initializer.h        # Map initialization (E + triangulation)
│   ├── tracker.h            # Frame-to-frame tracking + triangulation
│   └── vo_system.h          # State machine + I/O
├── src/                 # Implementation
│   ├── initializer.cpp
│   ├── tracker.cpp
│   ├── vo_system.cpp
│   └── main.cpp             # CLI runner
├── config/
│   └── euroc.yaml           # Camera intrinsics & parameters
├── scripts/             # (WIP) eval scripts
├── docs/                # Pipeline docs & results
├── test/                # Unit tests
└── CMakeLists.txt
```

## Algorithm Overview

| Stage              | Method                                |
|--------------------|---------------------------------------|
| Feature extraction | ORB (2000 features)                   |
| Matching           | BFMatcher + knn ratio test (0.8)      |
| Relative pose      | Essential matrix (RANSAC) + recoverPose |
| Triangulation      | DLT with median-depth outlier filter  |
| Tracking           | 2D-2D essential matrix frame-to-frame |
| Keyframe           | Every 5 frames with new point triangulation |
| Trajectory output  | TUM RGB-D benchmark format            |

## Dependencies

- **OpenCV** ≥ 4.x (features2d, calib3d, imgproc, highgui)
- **CMake** ≥ 3.16
- **C++17** (gcc ≥ 8 / clang ≥ 7)

## Supported Datasets

| Dataset | Mode   | K (fx, fy, cx, cy)                    |
|---------|--------|---------------------------------------|
| EuRoC   | `euroc`| 458.654, 457.296, 367.215, 248.375    |
| TUM     | `tum`  | 525.0, 525.0, 319.5, 239.5            |

## Limitations

- Monocular scale ambiguity (trajectory is up-to-scale, not metric)
- No bundle adjustment / loop closure
- ORB-based matching degrades with motion blur and low texture
- Default camera intrinsics for EuRoC MH_01; override for other sequences

## References

- [ORB-SLAM](https://github.com/raulmur/ORB_SLAM) — feature-based SLAM reference
- [TUM RGB-D Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) — dataset format & evaluation tools
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
