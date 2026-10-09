# Online Koide replay profile

Use this opt-in profile for the public [Hard Point Cloud Localization Dataset](https://zenodo.org/records/10122133) (Koide et al., CC BY 4.0), sequence `outdoor_hard_02b`, with its binary little-endian float32 `map_outdoor_hard.ply`. It runs online RKO-LIO and localization together at 1x.

The localizer uses a 75 m local-map radius, reuses the target until movement reaches 10 m, and receives `/livox/points` with RELIABLE QoS and depth 10. IMU integration and twist prediction in the localizer are disabled; RKO supplies `odom -> livox_frame`, and the localizer estimates `map -> odom`. Pose output includes odometry bridge predictions through rejected map matches. Timer publication is disabled and Path history is capped at 2,000 poses.

The radius is a dataset-specific choice: it changes which map points are available for registration. The conservative Nav2 preset and the offline GIF profile keep their existing radii. The localizer's relative-motion deskew is not applied with its IMU preintegration disabled; RKO performs its own deskew.

## One-command check

No robot or RViz is needed. Build and source this package and the `rko_lio` online
node from [lidar_slam_ros2](https://github.com/rsasaki0109/lidar_slam_ros2) in the
same ROS 2 environment. ROS 2 Humble is the environment used for the recorded
measurements below. The localizer package declares the Python/rosbag dependencies;
RKO is an optional dependency needed for this particular demo.

```bash
ros2 run lidar_localization_ros2 run_koide_public_bag.py --download
```

This downloads the official bag, map and GT archives (~1.3 GB), verifies their
published MD5 checksums, extracts them, and converts acceleration from g to SI.
Known acceleration covariances scale by g squared; unknown covariance stays unknown.
PointCloud2 bytes are preserved, and the prepared bag contains only points and IMU.
Verified input and conversion files are cached in `./koide-data` for subsequent runs.
Allow at least 6 GB of free disk space and about five minutes for the 1x replay after
data preparation. It uses the 75 m preset and actual online RKO odometry.

The command activates localization, waits for both RELIABLE point subscriptions,
starts playback, supplies the nearest GT pose once at +4 s, drains pending results,
stops its processes, and writes:

- `summary.json`: map-frame position RMSE, matched pose count, diagnostic coverage,
  and `passed`.
- `estimate.tum`: the observed pose trajectory, including odometry bridge poses.
- `receipt.json`: dataset, replay settings, executable hashes and exit codes.
- `localizer.yaml`, `localizer.log`, `rko.log`, `player.log`, `alignment.jsonl`:
  configuration and diagnostic logs for investigation.

A successful check exits 0 and prints `PASS`. It requires RMSE <= 0.35 m, diagnostic
observations at >= 97% of expected cloud stamps after initialization, and distinct
GT-matched pose timestamps at >= 95% of that cloud count. GT matching uses the same
0.15 s tolerance as the comparison below, without spatial alignment. These are demo
acceptance criteria for this sequence, not a complete release-regression suite.

Useful options:

```bash
# Check prerequisites and the plan without downloads, files or nodes.
ros2 run lidar_localization_ros2 run_koide_public_bag.py --dry-run

# Reuse cached data and choose a new results directory.
ros2 run lidar_localization_ros2 run_koide_public_bag.py \
  --data-dir /absolute/path/to/koide-data --output /absolute/path/to/new-run
```

The default ROS domain is 83; an occupied domain is refused to avoid mixing other
publishers into the evaluation. Use `--ros-domain-id 84` if it is in use. An unsourced
RKO workspace produces a dependency hint; an independently installed executable can
be selected with `--rko-executable /absolute/path/to/online_node`. A nonempty output
directory is refused. Ctrl+C stops the replay processes and keeps logs; use a new
output directory when retrying. Corrupt cache files are reported instead of reused.
The measurements below are the separately controlled four-run comparison; results
from this command describe the machine and binaries on which it is run.

## Manual reproduction

Build and source both this workspace and [lidar_slam_ros2](https://github.com/rsasaki0109/lidar_slam_ros2), including the `rko_lio` online executable. Download the sequence, map, and `gt.zip` from the dataset record. Convert its IMU acceleration from g to SI using `lidar_slam_ros2/tools/readme_media/scale_imu_bag.py`, as described in [README media](readme-media.md). Use the scaled standard PointCloud2/IMU bag without adding an odometry TF trajectory.

From the localizer checkout, replace the map placeholder:

```bash
sed "s|MAP_OUTDOOR_HARD_PLY|$PWD/map_outdoor_hard.ply|" \
  tools/readme_media/koide_outdoor_hard_02b_online.yaml > koide_online.yaml
```

In separate terminals with the same ROS domain, start RKO, localization, and then playback:

```bash
ros2 run rko_lio online_node --ros-args \
  --params-file "$PWD/tools/readme_media/koide_outdoor_hard_02b_online_rko.yaml"

ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:="$PWD/koide_online.yaml" \
  base_frame_id:=livox_frame lidar_frame_id:=livox_frame publish_lidar_tf:=false \
  cloud_topic:=/livox/points imu_topic:=/livox/imu

ros2 bag play outdoor_hard_02b_scaled --clock --rate 1 \
  --qos-profile-overrides-path tools/readme_media/koide_reliable_points_qos.yaml \
  --topics /livox/points /livox/imu
```

Publish one ground-truth initial pose at 4 s of playback using the command printed by:

```bash
python3 tools/readme_media/initial_pose_from_tum.py \
  traj_lidar_outdoor_hard_02.txt outdoor_hard_02b_scaled --offset 4
```

Ground truth supplies only that initial pose and evaluation; it is not an odometry input. Record `/pcl_pose` and `/alignment_status` before playing the bag. For the 150 m comparison, change only `local_map_radius` in the generated localizer YAML.

## v1.3.0 tag verification (2026-10-09)

One full `outdoor_hard_02b` replay at 1x using the unchanged 75 m profile passed
on release commit `3878a7ad64a5ef4119942b559f05bf8868270477`. The localizer was
rebuilt from the tag in a separate ROS 2 Humble workspace; online RKO-LIO used
the same pinned 0.3.2 image described below. This is one cloud-container public-bag
run, with no physical-robot validation or multi-hour stability claim.

```bash
ros2 run lidar_localization_ros2 run_koide_public_bag.py \
  --data-dir /workspace/v130-data --output /workspace/v130-results/replay
```

Official bag, map and GT MD5 checksums were verified before SI IMU conversion.
PointCloud2 bytes were preserved, no TF was inserted, and GT supplied only one
initial pose at +4 s and the evaluation reference. Errors use map-frame poses,
including odometry bridge predictions, without spatial alignment and with the
same 0.15 s nearest-GT tolerance as the comparison below.

| Metric | Observed | Demo acceptance |
| --- | --- | --- |
| Position RMSE | 0.279573 m | <= 0.35 m |
| Diagnostic cloud stamps | 2,209 / 2,209 (100%) | >= 97% |
| Distinct GT-matched pose stamps / expected clouds | 2,203 / 2,209 (99.73%) | >= 95% |
| GT-matched pose rows | 2,204 | — |
| Position error p95 | 0.799069 m | — |
| Maximum output timestamp gap | 0.901278 s | — |
| Command / RKO / localizer / player exit codes | all 0 | successful exit |

The related public-bag Python tests also passed (10 tests). RKO logged 20 frame
drops for insufficient ICP keypoints; these are retained in the evidence and
are not hidden by the PASS result. Diagnostic coverage does not measure NDT
acceptance or absence of upstream drops. The localizer also warned that a
per-point `t` span was about 0.20 s versus a configured 0.10 s scan period;
this profile disables localizer IMU integration and relative-motion deskew,
while RKO performs its own deskew. The run passes only the sequence-specific
demo gates, not a full release-regression suite. It does not repeat the paired
150 m / 75 m experiment or add new latency/RSS measurements.

Machine-readable [summary](validation/v1.3.0-koide-20261009/summary.json),
[receipt](validation/v1.3.0-koide-20261009/receipt.json), and
[provenance](validation/v1.3.0-koide-20261009/provenance.json) record the result,
configuration/input/executable hashes and source/image revisions. The localizer
SHA-256 was `fa964caebcb6a533065f8bda71ee92d5593be63d11f279d3f5a5cf2032576832`.

[Download the logs, trajectory, configurations and receipts](validation/v1.3.0-koide-20261009/evidence.tar.gz).
Archive SHA-256: `6e743c521245c5d6ae1f93f786e56ab6e0c8b3a58a13ac94b4a6ce673b9e3d71`. The archive includes build and replay logs;
it contains no bag or map files. Local `/workspace` paths in receipts describe
this run and are not prerequisites on another machine.

## Measured comparison

The localizer binary was built from `9f6be5d`. Online RKO-LIO 0.3.2 binaries came from `ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:ebb77154154d569a11d68d143f79112c03367b3c68da59b9f2a1d9afce63aeed` (image source revision `78df89bfda4edec68dd329777de584ff78796974`); both processes ran in the same ROS 2 Humble container. The crop radii were tested in the order 150, 75, 150, 75 m. Each replay lasted about 299 s at 1x. Only the radius changed: NDT used four threads and RKO two, with identical bag, initial pose, reliability, queue depths and observer scripts.

| Local-map radius | Runs | Position RMSE | Diagnostic observations after initialization | Receipt-to-diagnostic p95 lag | Localizer peak RSS |
| --- | --- | --- | --- | --- | --- |
| 150 m | 2 | 0.258–0.260 m | 98.1–98.8% | 1.10–1.20 s | 1,052–1,057 MiB |
| 75 m | 2 | 0.263–0.265 m | 2,209/2,209 (100%) in each run | 0.53–0.61 s | 593–596 MiB |

Paired p95 lag decreased by 44–56% and localizer peak RSS by about 44%. Across the same 2,137 cloud stamps in all four runs, map-frame RMSE was 0.254–0.257 m at 150 m and 0.255–0.258 m at 75 m. Online RKO TF p95 lag also fell from 0.65–0.90 s to 0.48–0.54 s; all runs emitted 2,226 RKO poses. Localizer process CPU medians were 0.70–0.71 cores at 150 m and 0.61–0.64 at 75 m (one core means one CPU-second per wall-second).

Position errors use `/pcl_pose`, including bridge predictions, in the map frame without spatial alignment; nearest GT time tolerance is 0.15 s. Ground truth was supplied only once at 4 s for initialization. Diagnostic coverage counts exact original cloud stamps after that point; it is not the fraction of accepted NDT measurements or direct callback-loss telemetry. Diagnostics were observed with RELIABLE depth 1,000 and Path via its raw serialized header/count; native process PIDs supplied CPU and RSS. Lag is the approximate replay-clock elapsed time from the original bag receive timestamp to diagnostic reception, extrapolated by wall time at rate 1 through the final drain; it includes transport and observer delay. No future-delivered TF was used.

These results cover one public sequence and do not establish real-robot or multi-hour moving stability. The individual costs of crop construction and search-tree rebuilding were not instrumented separately.
