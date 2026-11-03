# What NDT_OMP alignment costs on Boreas, and what can be cut

Question: in the dense stretches of Boreas the scan callback falls behind real time, and
alignment is most of each scan. What does an alignment cost there, and what can be cut
without losing accuracy?

## Setup

- **The scans:** `export_scans.py` takes every second scan of 84-116 s of the 120 s
  fixture (154 scans), with the ground-truth poses of it and of the two scans before.
- **The benchmark:** `boreas_ndt_bench.cpp` aligns them the way
  `param/boreas_ndt_velodyne.yaml` does:
  - a 100 m crop of the fixture map, rebuilt every 10 m;
  - the scan cut to 1-60 m and voxel-filtered at 1.5 m, about 4 650 points;
  - NDT_OMP with resolution 1.0, step 0.1, epsilon 0.01, 25 iterations, 8 threads
    and DIRECT7.
- **The seed:** the ground truth extrapolated at constant velocity from the two scans
  before. It is close to perfect.
- **Excluded from the timing:** the first `align()` after a new target, which builds
  the target search tree; the node builds that tree ahead.

All numbers come from one 4-vCPU container, run back to back. Timings drift between
sessions, so only compare rows of one table.

## Where the time goes

| Setting | `align()` mean / p90 | `getFitnessScore()` | Iterations | Translation RMSE / max |
| --- | --- | --- | --- | --- |
| preset | 27-32 / 43-46 ms | 6.5-7.1 ms | 5.18 | 0.123 / 0.301 m |

`alignment_time_sec` in `/alignment_status` times `align()` only. The fitness score
comes after it, on the scan path, and is serial in PCL: one nearest-neighbour search
per scan point against the crop. That is a fifth of the alignment again.

## Cheaper settings converge less

With the near-perfect seed, every setting that does less looks better, because it
stays near the seed:

| Setting | `align()` mean | Iterations | Translation RMSE |
| --- | --- | --- | --- |
| DIRECT1 neighbours | 9.1 ms | 5.10 | 0.090 m |
| epsilon 0.05 | 15.3 ms | 2.04 | 0.074 m |
| voxel 2.0 m (2 850 points) | 19.3 ms | 5.40 | 0.128 m |
| resolution 1.5 / 2.0 | 24.6 / 24.5 ms | 4.86 / 5.27 | 0.133 / 0.163 m |
| 4 threads | 32.9 ms | 5.18 | 0.123 m |

So the seed was moved by 0.5 m in x and y and 1° in yaw (`PERTURB` 0.5), which the
alignment has to recover:

| Setting | `align()` mean | Iterations | Translation RMSE / max |
| --- | --- | --- | --- |
| preset | 65.9 ms | 13.9 | 0.148 / 0.302 m |
| DIRECT1 neighbours | 34.8 ms | 23.9 (cap 25) | 0.201 / 0.626 m |
| epsilon 0.05 | 37.1 ms | 7.4 | 0.213 / 0.607 m |
| DIRECT1 and epsilon 0.05 | 5.4 ms | 2.5 | 0.616 / 0.699 m |

DIRECT1 and a looser epsilon buy their speed by converging less. Off a worse seed
they end 0.6 m off, where the preset ends 0.3 m off. The preset keeps both.

## The fitness score in parallel

`lidar_localization::parallelFitnessScore()` (`registration_fitness.hpp`) runs the same
nearest-neighbour searches as `pcl::Registration::getFitnessScore()`, spread over the
registration threads. It then sums them in point order, as PCL does, so the score is
bit-identical:

| Threads | PCL | Parallel | Scores that differ |
| --- | --- | --- | --- |
| 8 | 7.09 ms | 2.15 ms | 0 of 154 |
| 4 | 6.51 ms | 2.06 ms | 0 of 154 |

The localizer now uses it for every registration attempt. It saves about 4.5-5 ms of
each scan callback, with the same score and so the same decisions: the Boreas preset's
trajectory over 120 s is byte-identical.

## Inside `align()`: false sharing

Timers inside NDT_OMP showed where `align()` goes. They ran on 4 threads with the
preset, with the hash table described below already in:

- **The derivative passes are all of it.** One pass runs per iteration plus one at the
  seed, about 6.2 per scan. They took 29.2 of the 29.5 ms. The point transforms took
  0.1 ms and the 6x6 solves 0.02 ms.
- **Too slow for the arithmetic.** A pass handles about 25 000 point-voxel pairs
  (5.4 voxels per point). That came to 460 ns of thread time per pair, outside the
  voxel lookups. Computing each point's transform derivatives once instead of once per
  voxel saved nothing at this point.
- **The cause was false sharing.** Each thread added its score, gradient, Hessian and
  correspondence counts into its own entry of shared vectors, for every point and
  every correspondence. Those entries sit in the same cache lines, so the threads kept
  invalidating each other's caches.
- **The voxel lookups went through a `std::map`**, about 17 tree nodes for the
  100 000 voxels of a 100 m crop. On their own, the DIRECT7 lookups for one scan took
  1.7-1.9 ms on 8 threads, and 1.0-1.1 ms with a hash table.

[ndt_omp_ros2 #15](https://github.com/rsasaki0109/ndt_omp_ros2/pull/15) has each thread
sum into locals, and looks the voxels up in a hash table. The final transforms are
bit-identical on all 154 scans, for all four neighbour search methods:

| Setting | Threads | Before | After |
| --- | --- | --- | --- |
| preset | 8 | 27.5-29 ms | 14.0-14.6 ms |
| preset | 4 | 33.6 ms | 12.0-12.5 ms |
| preset, seed moved by `PERTURB` 0.5 | 8 | 65.0 ms | 32.8 ms |
| DIRECT1, seed moved by `PERTURB` 0.5 | 8 | 33.5 ms | 13.7 ms |

Taken one at a time on 8 threads, the per-thread locals take 27.5 ms to 19.5 ms and the
hash table takes that to 14 ms.

In the node, over the Boreas preset's 120 s, two pairs of runs alternated back to back
gave the following `alignment_time_sec`, with a byte-identical trajectory:

| ndt_omp_ros2 | Mean | p90 | p99 | Max |
| --- | --- | --- | --- | --- |
| before #15, run 1 | 18.1 ms | 31.8 ms | 58.1 ms | 94.0 ms |
| with #15, run 1 | 12.4 ms | 22.8 ms | 44.4 ms | 71.8 ms |
| before #15, run 2 | 19.1 ms | 34.1 ms | 68.8 ms | 120.1 ms |
| with #15, run 2 | 12.2 ms | 21.1 ms | 43.6 ms | 73.9 ms |

The node gains a third rather than half. Its other threads share the 4 vCPUs, and most
of its scans are smaller than the dense stretch above.

## Rerun

```bash
source /opt/ros/jazzy/setup.bash
python3 experiments/ndt_alignment_cost/export_scans.py FIXTURE /tmp/boreas_scans
g++ -O3 -DNDEBUG -std=c++17 -fopenmp -Iinclude -I"$WS/src/ndt_omp_ros2/include" \
  -I/usr/include/pcl-1.14 -I/usr/include/eigen3 \
  experiments/ndt_alignment_cost/boreas_ndt_bench.cpp -o /tmp/boreas_ndt_bench \
  -lpcl_common -lpcl_filters -lpcl_kdtree -lpcl_search -lpcl_registration
# THREADS RESOLUTION LEAF METHOD EPSILON [PERTURB]
/tmp/boreas_ndt_bench FIXTURE/map.ply /tmp/boreas_scans 8 1.0 1.5 2 0.01
/tmp/boreas_ndt_bench FIXTURE/map.ply /tmp/boreas_scans 8 1.0 1.5 2 0.01 0.5
```
