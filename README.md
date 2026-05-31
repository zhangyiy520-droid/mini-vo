# Mini VO

从零构建的轻量级单目视觉里程计，仅依赖 OpenCV。

**Pipeline：** ORB 提取 → Ratio-test 匹配 → 本质矩阵 → recoverPose → 三角化 → 帧到帧追踪 → 关键帧管理 → TUM 格式轨迹输出。

## 快速开始

```bash
# 编译
mkdir build && cd build
cmake .. && make -j$(nproc)

# EuRoC 数据集
./vo_bin euroc /path/to/MH_01_easy/mav0/cam0/data 50

# TUM RGB-D 数据集
./vo_bin tum /path/to/rgbd_dataset_freiburg1_xyz 200
```

**输出文件：**
- `trajectory.txt` — TUM 格式（timestamp tx ty tz qx qy qz qw）
- `map.ply` — 稀疏 3D 地图点（按深度着色）

## 项目结构

```
mini-vo/
├── include/mini_vo/     # 头文件
│   ├── types.h              # VOSystem 结构体、VOStatus 状态枚举
│   ├── initializer.h        # 初始化：E 矩阵 + 三角化地图
│   ├── tracker.h            # 帧间追踪 + 新地图点三角化
│   └── vo_system.h          # 状态机 + 轨迹/PLY 输出
├── src/                 # 实现
│   ├── initializer.cpp
│   ├── tracker.cpp
│   ├── vo_system.cpp
│   └── main.cpp             # CLI 入口（euroc / tum 两种模式）
├── config/
│   └── euroc.yaml           # 相机内参 + VO 超参数
├── scripts/             # 评估脚本（待完善）
├── docs/                # 文档
├── test/                # 单元测试（待完善）
└── CMakeLists.txt
```

## 算法流程

| 阶段         | 方法                                    |
|-------------|----------------------------------------|
| 特征提取     | ORB（2000 个特征点）                     |
| 特征匹配     | BFMatcher + knn ratio test（阈值 0.8）   |
| 相对位姿估计 | 本质矩阵（RANSAC） → recoverPose         |
| 三角化       | DLT + 中值深度离群过滤                    |
| 帧间追踪     | 当前帧 ↔ 上一帧 2D-2D 本质矩阵估计         |
| 关键帧       | 每 5 帧插入，与当前帧三角化新地图点         |
| 轨迹输出     | TUM RGB-D benchmark 格式                |

## 状态机

```
UNINITIALIZED ──(2 帧, 初始化成功)──▶ TRACKING
     ▲                                  │
     │         (跟踪失败)                │
     └──────────── LOST ◀───────────────┘
     │                                  │
     └─── (重初始化成功, 回到 TRACKING) ──┘
```

## 依赖

- **OpenCV** ≥ 4.x（features2d、calib3d、imgproc）
- **CMake** ≥ 3.16
- **C++17**（gcc ≥ 8 / clang ≥ 7）

## 支持的数据集

| 数据集   | 模式     | 内参 K (fx, fy, cx, cy)               |
|---------|---------|---------------------------------------|
| EuRoC   | `euroc` | 458.654, 457.296, 367.215, 248.375    |
| TUM     | `tum`   | 525.0, 525.0, 319.5, 239.5            |

## 已知局限

- 单目尺度不确定性（轨迹无绝对尺度）
- 无 Bundle Adjustment / 回环检测
- 运动模糊或弱纹理场景下 ORB 匹配退化
- 默认使用 EuRoC MH_01 标定参数，其他序列需手动修改

## 参考

- [ORB-SLAM](https://github.com/raulmur/ORB_SLAM) — 特征法 SLAM 标杆
- [TUM RGB-D Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) — 数据集格式与 evo 评估工具
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
