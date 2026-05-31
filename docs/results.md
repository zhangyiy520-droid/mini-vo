# Evaluation Results

> Placeholder — run VO on a dataset and evaluate with [evo](https://github.com/MichaelGrupp/evo).

## Example: EuRoC MH_01_easy

```bash
# Run VO
./build/vo_bin euroc /path/to/MH_01_easy/mav0/cam0/data 200

# Evaluate
evo_ape tum groundtruth.txt trajectory.txt --plot --plot_mode xyz
```

| Metric | Value |
|--------|-------|
| RMSE   | TBD   |
| Mean   | TBD   |
| Max    | TBD   |
| Frames | TBD   |

## Notes

- Monocular scale is arbitrary — align trajectories before comparison
- Groundtruth format should match TUM (timestamp tx ty tz qx qy qz qw)
