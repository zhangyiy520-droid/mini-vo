# Mini-VO 项目总览

本文是 `mini-vo` 的项目级总文档，覆盖数据输入、状态机、初始化、跟踪、地图、几何基础、g2o 优化、Local BA、测试、工具脚本和结果输出。

## 1. 项目定位

`mini-vo` 是一个使用 C++17 构建的轻量级单目视觉里程计与 BA 学习工程，核心依赖包括：

- OpenCV：图像、ORB、匹配、Essential Matrix、PnP 和三角化；
- Eigen3：线性代数；
- Sophus：SE(3) 表达；
- g2o：非线性图优化；
- CMake/CTest：构建和自动测试。

仓库目前同时包含两层实现：

1. **实时 VO 主链路**：使用 `VOSystem` 的扁平状态，完成初始化、PnP 跟踪、增量三角化、丢失重初始化和轨迹/点云输出；
2. **结构化地图与 BA 后端**：使用 `Map`、`KeyFrame`、`MapPoint`、`Observation`，提供重投影、三种 BA 模式、鲁棒外点分类、局部窗口和 Local BA。

这两层已经分别实现并测试，但 BA 后端尚未接入实时 `processFrame()` 主循环。理解这一点很重要：仓库“有 BA 模块”，但当前 `vo_bin` 运行时仍主要走传统 VO 主链路。

## 2. 仓库结构

```text
mini-vo/
├── apps/
│   └── day35_gauss_newton.cpp      # 独立高斯牛顿学习程序
├── config/
│   └── euroc.yaml                   # EuRoC 相机参数示例
├── docs/
│   ├── pipeline.md                  # 早期 VO 流程说明
│   ├── project_overview.md          # 本项目总文档
│   └── results.md                   # 数据集结果说明
├── include/mini_vo/
│   ├── initializer.h                # 双目帧单目初始化
│   ├── tracker.h                    # 3D-2D PnP 跟踪与新点三角化
│   ├── vo_system.h                  # 实时主循环及输出接口
│   ├── types.h                      # VOSystem 与状态机
│   ├── camera/                      # 针孔相机模型
│   ├── core/                        # Map、Observation、SE(3)
│   └── backend/                     # 重投影、g2o、BA、Local BA
├── scripts/
│   ├── convert_gt.py                # 轨迹真值转换
│   └── run_vo.py                    # 运行辅助脚本
├── src/
│   ├── main.cpp                     # vo_bin CLI 入口
│   ├── initializer.cpp
│   ├── tracker.cpp
│   ├── vo_system.cpp
│   ├── camera/
│   ├── core/
│   └── backend/
└── test/                            # 14 个 CTest 测试目标
```

## 3. 构建产物

| Target | 类型 | 用途 |
|---|---|---|
| `mini_vo` | 静态库 | 项目全部核心与后端实现 |
| `vo_bin` | 可执行程序 | EuRoC/TUM 图像序列运行入口 |
| `day35_gauss_newton` | 可执行程序 | 最小高斯牛顿学习实验 |
| `test_*` | 测试程序 | 初始化、跟踪、地图、相机、g2o、BA 等单元/集成测试 |

## 4. 实时 VO 总体数据流

```text
图像序列
  │
  ▼
vo_bin: 加载 EuRoC 目录或 TUM rgb.txt
  │
  ▼
processFrame()
  │
  ├── UNINITIALIZED
  │     ├── ORB 特征
  │     ├── BF + ratio test
  │     ├── Essential Matrix + RANSAC
  │     ├── recoverPose
  │     └── DLT 三角化初始化地图
  │
  ├── TRACKING
  │     ├── 当前帧 ORB
  │     ├── 当前描述子匹配全局地图描述子
  │     ├── solvePnPRansac
  │     ├── 更新 Tcw 与轨迹
  │     └── 与固定关键帧三角化新点
  │
  └── LOST
        └── 使用冻结关键帧与当前帧重新初始化
  │
  ▼
trajectory.txt + map.ply + 可选 debug 图片
```

主入口：

- [`src/main.cpp`](../src/main.cpp)
- [`src/vo_system.cpp`](../src/vo_system.cpp)
- [`include/mini_vo/vo_system.h`](../include/mini_vo/vo_system.h)

## 5. 状态机

状态定义在 [`types.h`](../include/mini_vo/types.h)：

```text
UNINITIALIZED
    │ 两帧初始化成功
    ▼
TRACKING
    │ PnP 跟踪失败
    ▼
LOST
    │ 与冻结关键帧重初始化成功
    └──────────────────────▶ TRACKING
```

### UNINITIALIZED

第一帧只保存图像、关键点和描述子，并把单位位姿写入轨迹。第二帧开始尝试双帧初始化；失败时滑动第一帧候选，等待下一帧再次尝试。

### TRACKING

使用当前帧的二维特征与已有三维地图描述子建立 2D–3D 对应，然后通过 `solvePnPRansac` 求解 `Tcw`。成功后记录轨迹并三角化新地图点。

当前代码在每次成功跟踪后都允许三角化，因为条件是：

```cpp
vo.frame_count - vo.last_keyframe >= 1
```

### LOST

跟踪失败后，不继续滑动参考帧，而是冻结关键帧，等待当前图像与该关键帧积累足够视差后重新初始化。

## 6. 双帧初始化

入口：[`initialize()`](../include/mini_vo/initializer.h)

主要步骤：

1. 两帧分别提取 ORB；
2. BFMatcher + ratio test；
3. `findEssentialMat()` 使用 RANSAC 估计本质矩阵；
4. SVD 修正本质矩阵奇异值；
5. `recoverPose()` 恢复相对旋转和平移方向；
6. DLT 三角化地图点；
7. 检查有限值、正深度、距离和重投影误差；
8. 建立初始地图、描述子和关键帧状态。

单目初始化只能得到相对尺度，不能恢复真实米制尺度。

## 7. PnP 跟踪与地图扩展

入口：[`track()`](../include/mini_vo/tracker.h)

跟踪流程：

```text
当前图像
  ├── ORB 2000 features
  ├── 当前描述子 ↔ map_descs
  ├── ratio test
  ├── 收集 2D–3D 对应
  ├── solvePnPRansac
  └── 输出 Rcw/tcw 与内点索引
```

匹配不足或 PnP 内点不足时返回失败，状态机进入 `LOST`。

新点由 `triangulateNewPoints()` 生成。当前实现使用初始化关键帧作为固定参考，通过较大基线进行三角化，并把通过质量检查的点追加到扁平 `map_points/map_descs`。

## 8. 两套地图表示

### 实时主链路：`VOSystem`

[`VOSystem`](../include/mini_vo/types.h) 直接保存：

- 当前和上一帧位姿；
- 关键帧图像、特征与位姿；
- `std::vector<cv::Point3f> map_points`；
- 与地图点同行对应的 `map_descs`；
- 轨迹旋转、平移和时间戳。

结构简单，适合教学和快速运行，但不显式保存“哪个关键帧观测了哪个地图点”。

### BA 后端：`Map`

[`Map`](../include/mini_vo/core/map.h) 使用结构化实体：

```text
KeyFrame
├── id / timestamp
├── image / keypoints / descriptors
├── Rcw / tcw
└── fixed

MapPoint
├── id
├── position_world
├── descriptor
└── bad

Observation
├── keyframe_id
├── map_point_id
├── feature_index / pixel / octave
└── outlier
```

`ObservationStore` 支持按关键帧、地图点查询以及外点标记，为 g2o 构图和 Local BA 提供稳定关联。

## 9. 相机、SE(3) 与重投影

几何基础模块：

- [`pinhole_camera.h`](../include/mini_vo/camera/pinhole_camera.h)
- [`se3_utils.h`](../include/mini_vo/core/se3_utils.h)
- [`reprojection_error.h`](../include/mini_vo/backend/reprojection_error.h)

项目统一使用 `Tcw`：

$$
\mathbf P_c = \mathbf R_{cw}\mathbf P_w + \mathbf t_{cw}
$$

针孔投影：

$$
u=f_x\frac{x_c}{z_c}+c_x,
\qquad
v=f_y\frac{y_c}{z_c}+c_y
$$

有效投影需要：

- 输入和结果均为有限值；
- `z_c > 0`；
- 相机内参有效；
- 观测和地图点没有被标记为无效。

## 10. g2o 图构建

入口：

- [`g2o_graph_builder.h`](../include/mini_vo/backend/g2o_graph_builder.h)
- [`g2o_graph_builder.cpp`](../src/backend/g2o_graph_builder.cpp)

构图器负责：

1. 验证 Map；
2. 根据选择范围收集关键帧、地图点和观测；
3. 创建 `VertexSE3Expmap`；
4. 创建 `VertexPointXYZ` 或 PoseOnly 一元边；
5. 添加信息矩阵和观测像素；
6. 保存顶点 ID 映射；
7. 保存 g2o edge 到原始 Observation 的绑定。

三种模式：

| `GraphMode` | 位姿 | 地图点 | 用途 |
|---|---|---|---|
| `PoseOnly` | 优化 | 固定 | PnP 后位姿精修 |
| `PointOnly` | 固定 | 优化 | 多帧地图点精修 |
| `FullBundleAdjustment` | 按策略固定或优化 | 优化 | 联合 BA |

## 11. BA 优化器

### Pose Optimizer

[`PoseOptimizer`](../include/mini_vo/backend/pose_optimizer.h) 固定地图点，只优化一个关键帧位姿。默认至少需要 6 条有效观测，并在写回前检查迭代、χ²、有限值和正深度。

### Point Optimizer

[`PointOptimizer`](../include/mini_vo/backend/point_optimizer.h) 固定相机位姿，只优化一个地图点。单目单帧不能确定唯一深度，因此至少需要两次有效观测。PointOnly 使用动态块求解器 `BlockSolverX`。

### Bundle Adjuster

[`BundleAdjuster`](../include/mini_vo/backend/bundle_adjuster.h) 同时优化选中的位姿和地图点。调用者可以指定：

- 优化关键帧集合；
- 优化地图点集合；
- 固定关键帧集合；
- 鲁棒预优化与最终优化次数；
- 最低有效边数；
- Huber 和 χ² 参数。

## 12. Gauge、χ² 与鲁棒外点

单目联合 BA 存在整体旋转、平移和尺度 Gauge。当前实现要求至少两个 fixed pose，以在课程场景中锚定位姿和已知基线尺度。

二维重投影残差的 χ²：

$$
\chi^2=\mathbf e^T\Omega\mathbf e
$$

默认参数：

| 参数 | 默认值 | 用途 |
|---|---:|---|
| `chi2_threshold` | `5.991` | 二维残差 95% 卡方外点阈值 |
| `huber_delta` | `2.447651936` | $\sqrt{5.991}$，Huber 范数切换点 |

鲁棒 BA 流程：

```text
安装 Huber 核
  → 短预优化
  → χ² + 正深度分类
  → 外点 level=1
  → 移除 Huber 核
  → 重新初始化活跃边
  → 仅内点最终优化
```

实现位置：[`robust_policy.cpp`](../src/backend/robust_policy.cpp)。

## 13. Local BA

Local BA 由两个模块组成：

1. [`LocalWindowSelector`](../include/mini_vo/backend/local_window_selector.h)：根据共享地图点数量选择局部关键帧、局部地图点和边界固定关键帧；
2. [`LocalBundleAdjuster`](../include/mini_vo/backend/local_bundle_adjuster.h)：组装局部优化选项、保证至少两个尺度锚、执行联合 BA，并把拒绝观测标记为 outlier。

默认窗口参数：

| 参数 | 默认值 |
|---|---:|
| `max_local_keyframes` | `5` |
| `max_fixed_keyframes` | `10` |
| `minimum_shared_points` | `5` |

Local BA 后端已经通过测试，但当前实时 `VOSystem` 使用另一套扁平地图结构，因此尚未在 `processFrame()` 中调用。

## 14. 结果写回安全策略

优化器先在独立 g2o 图中求解，再验证：

- 迭代配置有效；
- 鲁棒策略有效；
- fixed pose 数量足够；
- 有效观测边数量足够；
- 优化实际执行；
- 最终 χ² 有限且不高于初始 χ²；
- 所有 pose/point estimate 有限；
- 活跃观测保持正深度。

任何检查失败都只返回报告，原 Map 不变。全部通过后才统一写回 `Rcw/tcw` 和 `position_world`。

## 15. CLI 与数据集

`vo_bin` 支持两种输入模式。

### EuRoC 目录模式

```bash
./build/vo_bin euroc /path/to/MH_01_easy/mav0/cam0/data 200
```

程序扫描目录下的 `.png/.jpg/.jpeg`，按文件名排序。当前 EuRoC 内参直接写在 `main.cpp` 中。

### TUM 模式

```bash
./build/vo_bin tum /path/to/rgbd_dataset_freiburg1_xyz 200
```

程序读取数据集目录中的 `rgb.txt`，保留真实时间戳，并使用 `main.cpp` 中的 TUM fr1 内参。

### 调试图像

```bash
./build/vo_bin tum /path/to/dataset 200 --viz
```

`--viz` 会在 `debug/` 中保存 ORB、初始化、跟踪内点和丢失状态图像。

## 16. 输出文件

| 输出 | 格式 | 内容 |
|---|---|---|
| `trajectory.txt` | TUM trajectory | 时间戳、平移和四元数 |
| `map.ply` | ASCII PLY | 按深度着色的稀疏地图点 |
| `debug/*.png` | PNG | 可选的逐阶段调试图像 |

已有数据集结果见 [`results.md`](results.md)。

## 17. 构建方法

在 WSL 中：

```bash
cd /home/slamdev/workspace/mini-vo

CMAKE_PREFIX_PATH=$HOME/workspace/deps/sophus:$HOME/workspace/deps/g2o \
  cmake -S . -B build -DBUILD_TESTS=ON

cmake --build build -j4
```

依赖：

```text
OpenCV >= 4.x
CMake >= 3.16
C++17 compiler
Eigen3
Sophus
g2o: core, types_sba, solver_dense
```

## 18. 测试体系

运行全部测试：

```bash
ctest --test-dir build --output-on-failure
```

当前共 14 项：

| 层次 | 测试 |
|---|---|
| 前端 | `test_initializer`, `test_tracker` |
| 数据模型 | `test_observation`, `test_map` |
| 几何 | `test_camera`, `test_reprojection_error`, `test_se3_utils` |
| g2o | `test_g2o_graph_builder` |
| 单变量优化 | `test_pose_optimizer`, `test_point_optimizer` |
| 鲁棒与联合 BA | `test_robust_policy`, `test_bundle_adjuster` |
| Local BA | `test_local_window_selector`, `test_local_bundle_adjuster` |

测试成功只能证明这些受控场景通过；真实数据集仍需要轨迹、地图和日志量化验证。

## 19. 配置与脚本

- [`config/euroc.yaml`](../config/euroc.yaml)：EuRoC 相机参数示例；当前 `vo_bin` 尚未读取该 YAML，运行时内参来自 `main.cpp`；
- [`scripts/run_vo.py`](../scripts/run_vo.py)：运行辅助；
- [`scripts/convert_gt.py`](../scripts/convert_gt.py)：真值轨迹格式转换；
- [`apps/day35_gauss_newton.cpp`](../apps/day35_gauss_newton.cpp)：独立优化学习目标，不属于实时 VO 主循环。

## 20. 当前已实现能力

- EuRoC 图像目录和 TUM `rgb.txt` 输入；
- ORB 特征与 ratio-test 匹配；
- Essential Matrix + RANSAC 双帧初始化；
- `recoverPose` 与 DLT 三角化；
- 3D–2D PnP 跟踪；
- 跟踪失败后的冻结关键帧重初始化；
- TUM 轨迹和 PLY 稀疏地图输出；
- 显式 `Map/Observation` 数据模型；
- 针孔相机、SE(3) 和重投影模块；
- PoseOnly、PointOnly 和联合 BA；
- Huber、χ²、正深度外点分类；
- 共视局部窗口和 Local BA；
- 14 项自动测试。

## 21. 当前限制与后续接线

### 实时 VO 与 BA 后端尚未统一

`VOSystem` 使用扁平地图，而 BA 使用结构化 `Map`。目前缺少稳定的转换/统一层，因此 Pose BA、Point BA 和 Local BA 没有在实时帧处理中自动执行。

### 相机配置仍硬编码

虽然仓库存在 `config/euroc.yaml`，但 `main.cpp` 仍直接写入 EuRoC 和 TUM fr1 内参。

### 单目尺度不确定

初始化和实时轨迹没有外部尺度源。BA 只能保持已有尺度锚，不能凭像素观测恢复真实米制尺度。

### 没有回环检测与位姿图优化

仓库当前没有词袋、回环候选、Sim(3) 验证或 pose graph optimization，长轨迹仍会累积漂移。

### 地图维护较基础

实时主链路缺少完整的关键帧剔除、地图点融合、长期可见性统计和自适应关键帧策略。

## 22. 推荐排错顺序

### 程序没有轨迹输出

1. 确认图像是否成功加载；
2. 查看初始化是否成功；
3. 查看 PnP 匹配数和内点数；
4. 查看状态是否进入 `LOST`；
5. 最后检查输出路径权限。

### 初始化反复失败

1. 检查内参和图像尺寸；
2. 检查 ORB 数量与匹配数；
3. 检查帧间视差；
4. 检查 Essential Matrix RANSAC 内点；
5. 检查三角化正深度和重投影误差。

### PnP 跟踪失败

1. 检查 `map_descs` 与 `map_points` 是否同行对应；
2. 检查 2D–3D 匹配数；
3. 检查 PnP RANSAC 内点；
4. 检查地图点质量和视角变化；
5. 检查相机内参是否属于当前数据集。

### BA 失败

1. 读取 report message；
2. 检查 `Tcw/Twc` 约定；
3. 检查 Map 和 Observation 关联；
4. 检查有效边数和 fixed pose；
5. 检查 χ²、正深度和鲁棒参数；
6. 不要用增加迭代次数掩盖 Gauge 或错误输入。

## 23. 项目级验收清单

- [ ] `vo_bin` 能加载目标 EuRoC/TUM 数据；
- [ ] 初始化能够建立有限、正深度的稀疏地图；
- [ ] TRACKING 状态能持续输出 PnP 位姿；
- [ ] LOST 状态能够等待并尝试重初始化；
- [ ] `trajectory.txt` 和 `map.ply` 正常生成；
- [ ] Map、Observation 和相机几何测试通过；
- [ ] 三种 g2o 优化模式测试通过；
- [ ] Gauge 与鲁棒外点策略测试通过；
- [ ] Local BA 窗口和写回测试通过；
- [ ] 完整 CTest 14/14 通过；
- [ ] 文档明确区分已实现后端与已接入实时主链路的能力。
