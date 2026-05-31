# Mini VO Pipeline

## 1. System State Machine

```
UNINITIALIZED ──(2 frames, init OK)──▶ TRACKING
     ▲                                    │
     │         (track fails)              │
     └──────────── LOST ◀─────────────────┘
     │                                    │
     └──── (re-init OK, back to TRACKING)─┘
```

## 2. Initialization (2 frames)

```
Frame1 + Frame2
    │
    ├─ ORB extract (2000 features each)
    ├─ BFMatcher + ratio test (0.8)
    ├─ findEssentialMat (RANSAC, 0.999 confidence, 1.0px)
    ├─ SVD correction (σ1=σ2=mean, σ3=0)
    ├─ recoverPose → R, t
    ├─ triangulatePoints (DLT)
    ├─ Median-depth outlier filter (|Z| > 10× median → reject)
    ├─ Reprojection error check (< 3.0 px)
    └─ Initialize map (≥ 20 points)
```

## 3. Tracking (frame-to-frame)

```
Current frame
    │
    ├─ ORB extract
    ├─ BF match current → previous descriptors
    ├─ findEssentialMat (RANSAC)
    ├─ recoverPose → R_rel, t_rel
    ├─ Compose: R_cur = R_rel × R_prev
    │          t_cur = R_rel × t_prev + t_rel
    └─ If fails → LOST state
```

## 4. Keyframe Insertion (every 5 frames)

```
Keyframe + Current frame
    │
    ├─ BF match kf_descs → cur_descs
    ├─ triangulatePoints
    ├─ Adaptive distance filter (20× median map depth)
    ├─ Reprojection check
    └─ Append to map_points + map_descs
```

## 5. Output

- **trajectory.txt**: TUM format `timestamp tx ty tz qx qy qz qw`
- **map.ply**: ASCII PLY, colored by depth (red = far, blue = near)

## 6. Key Parameters

| Parameter           | Value  | Rationale                         |
|---------------------|--------|-----------------------------------|
| ORB features        | 2000   | Enough for indoor, not too slow   |
| Ratio threshold     | 0.8    | Lowe's ratio test                 |
| RANSAC confidence   | 0.999  | High confidence for essential mat |
| RANSAC threshold    | 1.0 px | Pixel reprojection tolerance     |
| Min matches         | 30     | Minimum for reliable estimation   |
| Min inliers         | 20     | After RANSAC filtering            |
| Keyframe interval   | 5      | Trade-off density vs. drift       |
| Max reproj. error   | 3.0 px | Quality filter for map points     |
| Depth factor        | 10×    | Median-depth multiplier           |
