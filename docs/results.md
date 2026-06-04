# Evaluation Results

Dataset: **TUM RGB-D fr1/xyz** (200 frames)

## Output Summary

| Metric     | Value  |
|------------|--------|
| Frames     | 198    |
| Map points | 80,992 |
| Format     | TUM (timestamp tx ty tz qx qy qz qw) |

## Visualizations

### 轨迹（俯视）
![trajectory](results/map_traj.png)

### 稀疏地图 — 俯视
![map top](results/map_topdown.png)

### 稀疏地图 — 侧视
![map side](results/map_side.png)

### 3D 地图（交互）
浏览器打开 [results/map_3d.html](results/map_3d.html)

## 评估

```bash
# 用 evo 评估（单目需 Sim(3) 对齐）
evo_ape tum groundtruth.txt results/trajectory.txt --plot --plot_mode xyz -as
```

> 当前版本使用 3D-2D PnP 跟踪（solvePnPRansac），尺度一致性已优于纯 2D-2D。
> 下一步加局部 BA 可进一步降低漂移。

## Notes

- Monocular scale is arbitrary — align trajectories before comparison
- TUM ground truth: `timestamp tx ty tz qx qy qz qw`
