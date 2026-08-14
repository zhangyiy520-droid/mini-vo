# Day 40–41：联合 BA、χ² 与鲁棒外点处理

本文对应 `mini-vo` 的 Day40 和 Day41，实现目标是：

1. 同时优化关键帧位姿与地图点；
2. 正确处理单目 BA 的 Gauge freedom；
3. 使用 Huber 核降低大残差的影响；
4. 使用 χ² 和正深度检查识别外点；
5. 只在全部验证通过后写回地图。

## 1. 代码入口

| 功能 | 接口 | 实现 | 测试 |
|---|---|---|---|
| 联合 Bundle Adjustment | [`bundle_adjuster.h`](../include/mini_vo/backend/bundle_adjuster.h) | [`bundle_adjuster.cpp`](../src/backend/bundle_adjuster.cpp) | [`test_bundle_adjuster.cpp`](../test/test_bundle_adjuster.cpp) |
| 鲁棒核与外点分类 | [`robust_policy.h`](../include/mini_vo/backend/robust_policy.h) | [`robust_policy.cpp`](../src/backend/robust_policy.cpp) | [`test_robust_policy.cpp`](../test/test_robust_policy.cpp) |
| g2o 图构建 | [`g2o_graph_builder.h`](../include/mini_vo/backend/g2o_graph_builder.h) | [`g2o_graph_builder.cpp`](../src/backend/g2o_graph_builder.cpp) | [`test_g2o_graph_builder.cpp`](../test/test_g2o_graph_builder.cpp) |

核心调用入口：

```cpp
mini_vo::BundleAdjustmentOptions options;
options.fixed_keyframe_ids = {0, 1};

const mini_vo::BundleAdjustmentReport report =
    mini_vo::BundleAdjuster().optimize(map, camera, options);
```

不要只检查 `report.success`。调试时至少同时记录：

```cpp
std::cout << "ok=" << report.success
          << " chi2=" << report.initial_chi2
          << "->" << report.final_chi2
          << " inliers=" << report.inliers
          << " outliers=" << report.outliers
          << " message=" << report.message << '\n';
```

## 2. 联合 BA 优化什么

联合 BA 同时调整两类变量：

- 关键帧位姿 `Tcw`；
- 地图点世界坐标 `Pw`。

每条观测边约束一个地图点在一个关键帧中的重投影位置：

```text
Pw --Tcw--> Pc --camera projection--> predicted pixel
```

残差是观测像素与预测像素之差：

$$
\mathbf e =
\begin{bmatrix}
u_{obs} - u_{pred} \\
v_{obs} - v_{pred}
\end{bmatrix}
$$

优化器寻找一组位姿和地图点，使所有有效观测的总体误差尽可能小。

## 3. 为什么单目 BA 必须处理 Gauge

单目重投影只约束图像几何，不能单独确定全局坐标系和绝对尺度。

- 不固定任何位姿：整体旋转、平移和缩放都可以变化；
- 只固定一帧：消除了整体旋转和平移，但尺度仍然自由；
- 固定两帧及其已知基线：本项目的小场景同时获得位姿锚和尺度锚。

因此 `BundleAdjuster` 会拒绝少于两个 fixed pose 的单目 BA：

```text
monocular BA needs two fixed poses to anchor pose and scale
```

这不是数值优化失败，而是构图约束不足。增加迭代次数或调大 LM 阻尼不能修复 Gauge。

## 4. χ² 是什么

χ² 是加权残差的平方：

$$
\chi^2 = \mathbf e^T \Omega \mathbf e
$$

其中 $\Omega$ 是信息矩阵，表示观测可信度。当 $\Omega=I$ 时：

$$
\chi^2 = (\Delta u)^2 + (\Delta v)^2
$$

例如像素残差为 `(2, 1)`：

```text
chi2 = 2² + 1² = 5
```

本项目对二维单目重投影使用 95% 卡方阈值：

```text
chi2_threshold = 5.991
```

分类规则：

```text
positive depth && finite chi2 && chi2 <= 5.991  => inlier
otherwise                                       => outlier
```

正深度必须单独检查。一个点即使数值残差很小，只要落在相机后方，也不能作为有效观测。

## 5. Huber delta 与 χ² 阈值不能混用

Huber 核负责“降权”，χ² 阈值负责“分类”。二者不是同一个量。

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `huber_delta` | `2.447651936` | 限制大残差对优化方向的影响 |
| `chi2_threshold` | `5.991` | 判断观测是内点还是外点 |

两者的关系为：

$$
\sqrt{5.991} \approx 2.44765
$$

`huber_delta` 作用于误差范数，`chi2_threshold` 作用于平方误差。把二者直接互换会改变鲁棒核和外点分类的实际尺度。

当前实现会拒绝非有限值、零值和负值，避免非法策略进入优化器：

```cpp
mini_vo::RobustPolicyOptions policy;
if (!policy.valid()) {
    // Do not start BA.
}
```

## 6. 两阶段鲁棒优化

默认流程不是一次性把所有边优化到底：

```text
构图
  ↓
安装 Huber 核
  ↓
短预优化（默认 3 次）
  ↓
按 chi2 + 正深度分类，外点设为 level=1
  ↓
移除 Huber 核
  ↓
仅使用 level=0 内点做最终优化（默认 10 次）
  ↓
验证结果
  ↓
统一写回 Map
```

短预优化的目的，是先让初值进入合理范围。预优化太久时，一条强外点可能先拖动位姿和地图点，随后导致正确观测被误判。

## 7. 写回前的安全检查

`BundleAdjuster` 在修改 `Map` 前检查：

- 迭代次数有效；
- 鲁棒策略参数有效；
- 至少两个 fixed pose；
- 重投影边数量达到最低要求；
- 最终 χ² 有限且没有高于初始 χ²；
- 所有位姿和地图点 estimate 均为有限值；
- 所有活跃边保持正深度。

只有全部通过，才统一写回位姿和地图点。失败时返回 `BundleAdjustmentReport::message`，原 Map 保持不变。

## 8. 构建与测试

在 WSL 项目根目录执行：

```bash
cd /home/slamdev/workspace/mini-vo

cmake -S . -B build -DBUILD_TESTS=ON
cmake --build build --target test_bundle_adjuster test_robust_policy -j4

./build/test_bundle_adjuster
./build/test_robust_policy
```

预期关键输出：

```text
[PASS] BA chi2 13148.7 -> ... pose_error=... edges=120 invalid_policy_rejected=true
[PASS] robust inliers=5 outliers=1 bad_chi2=6400 invalid_threshold_rejected=true
```

运行完整回归测试：

```bash
ctest --test-dir build --output-on-failure
```

## 9. 常见问题定位顺序

### BA 直接拒绝启动

1. 检查 `report.message`；
2. 检查 fixed pose 数量；
3. 检查有效边数量；
4. 检查 Huber delta 和 χ² 阈值。

### χ² 很低但地图尺度错误

先检查 Gauge 和固定基线。低重投影误差不能证明单目地图尺度正确。

### 外点仍影响第二轮优化

确认分类后执行了：

```cpp
edge->setLevel(1);
optimizer.initializeOptimization(0);
```

仅修改业务层的 `outlier` 布尔值，不会自动更新 g2o 当前的活跃边集合。

### 所有观测都被判为外点

依次核对：

1. `Tcw` 与 `Twc` 是否混用；
2. 信息矩阵和图像金字塔尺度；
3. χ² 阈值是否误填成 Huber delta；
4. 地图点是否位于相机前方；
5. 初始位姿是否已经严重偏离。

## 10. 验收清单

- [ ] 联合 BA 使用 120 条重投影边收敛；
- [ ] 只固定一帧时主动拒绝优化；
- [ ] 能解释单目系统的 7DoF Gauge；
- [ ] 能区分 Huber 降权和 χ² 外点分类；
- [ ] 5 个内点、1 个外点分类正确；
- [ ] 非法鲁棒策略被拒绝且 Map 不变；
- [ ] 完整 CTest 全部通过。
