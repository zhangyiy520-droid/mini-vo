# Mini VO

从零构建的轻量级单目视觉里程计，仅依赖 OpenCV。不基于 ORB-SLAM 等现有框架。

## TUM fr1/xyz 评测结果

| 指标 | 数值 |
|------|------|
| **跟踪成功率** | **198/200 (99%)** |
| 初始化匹配 / 位姿内点 | 1,394 / 92 |
| PnP 峰值匹配 / 内点 | **587 / 580** |
| 累积稀疏地图点 | **80,982** |
| 地图范围 X × Y × Z | 55m × 48m × 85m |
| 平摊平移误差 | 0.9 cm/帧 |
| 单帧平均平移 | 0.585m（单目尺度下） |
| 已知局限 | 单目尺度歧义，无可恢复绝对尺度、无 BA/回环 |

## 调试中发现的深层问题

| # | 问题 | 类型 | 修复 |
|---|------|------|------|
| 1 | 初始化描述子索引错位（mask_pose 过滤后 good[] 索引失效） | Bug | 保存原始 good 索引 `iorig[]` |
| 2 | track() 缺少 SVD 校正（与 initialize() 不一致） | Bug | 补全 SVD 分解 + 强制 σ₁=σ₂, σ₃=0 |
| 3 | 2D-2D recoverPose 的 t 是单位向量，逐帧复合尺度漂移 | 架构 | 移除 2D-2D，改为纯 PnP |
| 4 | LOST 恢复用 `vo.prev_img` 每帧追赶，零视差无法初始化 | Bug | 改用 init 关键帧固定锚点 |
| 5 | 三角化缺视差角检查，小基线点被错误加入地图 | 逻辑 | 光心射线夹角 cos < 0.999 剔除 |
| 6 | 距离阈值 ×20 过松，几乎不滤离群 | 逻辑 | 收紧为 ×5 + 下界 ×0.1 |
| 7 | triangulate 基线为相邻帧（短基线，全被视差过滤拒掉） | 架构 | 固定在 init 第一帧做大基线参考 |
| 8 | solvePnPRansac 失败返回时 rvec 未初始化 | Bug | 检查返回值 + inliers.empty() |

## 快速开始

```bash
# 编译
mkdir build && cd build
cmake .. && make -j$(nproc)

# TUM RGB-D 数据集（推荐）
./vo_bin tum /path/to/rgbd_dataset_freiburg1_xyz 200

# EuRoC 数据集
./vo_bin euroc /path/to/MH_01_easy/mav0/cam0/data 50

# 可视化（保存 debug 图片到 debug/ 目录）
./vo_bin tum /path/to/dataset 200 --viz
```

**输出文件：**
- `trajectory.txt` — TUM 格式（timestamp tx ty tz qx qy qz qw），可直接用 evo 评估
- `map.ply` — 稀疏 3D 地图点（按深度着色），可用 MeshLab 打开
- `debug/` — 可视化图片（ORB 特征点、PnP 内点、轨迹状态）

## Pipeline

| 阶段 | 方法 |
|------|------|
| 特征提取 | ORB（2000 特征点） |
| 特征匹配 | BFMatcher + knn ratio test（阈值 0.8） |
| 初始化 | 本质矩阵 RANSAC → SVD 校正 → recoverPose → DLT 三角化 → 中值深度过滤 + 重投影误差检查 |
| 帧间追踪 | 3D-2D PnP（solvePnPRansac，阈值 6.0px），地图点 ≥ 10 即可跟踪 |
| 关键帧 | 每帧插入，与 init 第一帧做大基线三角化（含视差角检查） |
| 轨迹输出 | TUM benchmark 格式，含真实时间戳 |

## 状态机

```
UNINITIALIZED ──(2帧初始化成功)──▶ TRACKING
     ▲                              │
     │         (跟踪失败)            │
     │                              ▼
     │              LOST ──(关键帧重初始化成功)──▶ TRACKING
     │                 │
     │                 └── (失败) 关键帧冻结，等待运动积累 → 重试
     └──────────────────────────────────────────────────────┘
```

## 项目结构

```
mini-vo/
├── include/mini_vo/
│   ├── types.h          # VOSystem 结构体、VOStatus 状态枚举
│   ├── initializer.h    # 初始化：E 矩阵 + 三角化地图
│   ├── tracker.h        # PnP 追踪 + 地图点三角化
│   └── vo_system.h      # 状态机 + 轨迹/TUM/PLY 输出
├── src/
│   ├── initializer.cpp
│   ├── tracker.cpp
│   ├── vo_system.cpp
│   └── main.cpp         # CLI 入口（euroc / tum + --viz）
├── config/
│   └── euroc.yaml       # 相机内参 + VO 超参数
├── scripts/             # 评估脚本
├── docs/                # 文档
├── test/                # 单元测试
└── CMakeLists.txt
```

## 依赖

- **OpenCV** ≥ 4.x（features2d、calib3d、imgproc）
- **CMake** ≥ 3.16
- **C++17**（gcc ≥ 8 / clang ≥ 7）

## 支持的数据集

| 数据集 | 模式 | 内参 K (fx, fy, cx, cy) |
|--------|------|--------------------------|
| TUM RGB-D fr1 | `tum` | 517.3, 516.5, 318.6, 255.3 |
| EuRoC MH_01 | `euroc` | 458.654, 457.296, 367.215, 248.375 |

## 已知局限

- 单目尺度不确定性（无绝对尺度，需数据集真值对齐恢复）
- 无 Bundle Adjustment / 回环检测（纯前端 VO）
- 每次重初始化重建世界帧，各段坐标系不连续
- 弱纹理 / 运动过慢场景 ORB 匹配退化

## 参考

- [ORB-SLAM](https://github.com/raulmur/ORB_SLAM) — 特征法 SLAM 标杆
- [TUM RGB-D Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset)
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
