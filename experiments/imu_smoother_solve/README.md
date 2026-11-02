# IMU smoother: dense against sparse solve

Question: with `use_imu_preintegration` on, the node skipped 18-19% of the scans of the
first 120 s of Boreas, more towards the end. Stack samples of the running node showed
`ImuGtsamSmoother::optimize` on the scan path. What does an update cost?

`optimize()` runs up to 10 Gauss-Newton iterations over the window (50 poses, 9 states
each, plus 6 bias states). It built the normal equations as a dense 456 × 456 matrix and
solved them with a dense LDLT. The matrix is block tridiagonal over the window plus the
bias rows and columns. It is now assembled from triplets and factored with
`Eigen::SimplicialLDLT`, with the ordering computed once per update, since all
iterations have the same factors. The dense LDLT stays as a fallback for a matrix the
sparse one cannot factor.

## Synthetic drive

`smoother_bench.cpp`: 10 m/s along x, 200 Hz IMU, 10 Hz NDT poses with 2 cm noise, 120
scans; one core, `-O3`.

| Solve | Scans 11-50 median | Scans 51-120 median / max | Final pose |
| --- | --- | --- | --- |
| dense LDLT (`main`) | 29-41 ms | 87-96 ms / 141-143 ms | x 120.015617371 |
| sparse, ordering per iteration | 13 ms | 20 ms / 47 ms | same to 1e-9 m |
| **sparse, ordering once per update** | **9-10 ms** | **13 ms / 22-23 ms** | same to 1e-9 m |

The two dense runs differ by machine load. With the window full, the dense update took
most of the 100 ms scan period by itself.

## Boreas, first 120 s, `--set use_imu_preintegration=true`

| Solve | Runs | RMSE m | Max error m | Scans processed |
| --- | --- | --- | --- | --- |
| dense | 2 | 0.120, 0.122 | 0.36, 0.61 | 82%, 81% |
| sparse | 2 | 0.127 (both) | 0.36 | 100%, 99% |

The preset without the IMU scores 0.132 m with every scan processed, and is
byte-identical before and after (it does not run the smoother). Measured 2026-11-02 on
ROS 2 Jazzy, 4 vCPUs; a public-dataset replay, not a physical-robot test.

## Rerun

```bash
g++ -O3 -DNDEBUG -std=c++17 -Iinclude -I/usr/include/eigen3 \
  experiments/imu_smoother_solve/smoother_bench.cpp -o /tmp/smoother_bench
/tmp/smoother_bench
python3 tools/benchmark/boreas_replay.py /tmp/boreas_fixture /tmp/boreas_preint \
  --ws "$WS" --set use_imu_preintegration=true
```
