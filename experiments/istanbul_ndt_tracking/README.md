# Why NDT lost the Autoware Istanbul route

Question: why did `param/istanbul_gtsam_smoother.yaml` drift to 6-24 m horizontal RMSE
on the Istanbul localization bag
([twist_smoother_slope](../twist_smoother_slope/README.md)), although the twist matches
GNSS? Three causes stacked, and each one hid the next.

## 1. The full-map NDT target was empty

Without `enable_local_map_crop` the NDT voxel grid spans the map's bounding box,
15.2 x 12.7 km and 284 m of height: 5.5e10 cells at `ndt_resolution: 1.0` (6.9e9 at 2 m;
it first fits at 3 m, 2.0e9). pclomp indexes the grid with int32, so it printed
`Leaf size is too small for the input dataset. Integer indices would overflow.`, left the
target empty, and NDT returned its initial guess on every scan (corrections of 0.008 m).
The pose was the twist dead reckoning from the GNSS initial pose.

The cross-track drift that looked like a yaw-rate error came from that initial pose: the
GNSS orientation in this bag is 1.9-2.3° off the direction of travel (median -2.26° over
the bag, computed from GNSS positions above 5 m/s), and 300 m at 1.9° is 10 m. Wherever
NDT registers, its yaw matches the direction of travel within 0.06°, so **yaw RMSE
against GNSS carries a ~2° reference bias** in the tables below.

The localizer now logs an error with the map's extent when it builds such a grid (the
full-map target, or the NDT initializer of the GICP backends), and the preset crops.

## 2. The smoother placed each NDT pose at the processing time

With the crop, NDT registered (fitness 0.2 while parked, against 0.74 for the GNSS
pose), but the replay diverged once the car reached about 3 m/s: 53 m horizontal RMSE in
window A. The pose fell behind along the track while the cross-track error stayed small.

`TwistGtsamSmoother::update()` accepted the scan stamp and did not use it. The NDT pose is
the pose at the scan stamp, but it was put on a node built from all twist received until
the scan was processed, and the accumulated twist was then cleared. Every update dropped
the motion made while the scan waited and was aligned, and the next seed started behind
by speed x delay: 1-1.5 m at 3 m/s, 10-20 m at 10 m/s behind a 1.7 s target rebuild, out
of NDT's reach. Without registration (cause 1) the NDT pose equals the seed, so nothing
showed.

The smoother now counts each twist message toward the scan its stamp precedes, carries
the later ones over to the new node, and seeds NDT with the pose predicted at the scan
stamp (`test/test_twist_gtsam_smoother.cpp`).

## 3. Twist was dropped while a scan held the state lock

The local map crop and target build run under the state lock, 1.7 s per rebuild here,
and with a pose backend enabled the twist callback waits for that lock. The twist
subscription kept the default depth of 5 (50 ms at 100 Hz), and the smoother skips a gap
over 1 s, so each rebuild cost about 1.2 s of motion: with rebuilds every 30 m the pose
fell back 10-14 m at each one. The subscription now queues 1000 messages.

`TwistEkf::update()` took no stamp either and had cause 2 as well; see *Twist EKF* below.

## Method

`../twist_smoother_slope/compare.py` replays three windows three times per variant with
`param/istanbul_gtsam_smoother.yaml`, starts from the GNSS pose at the window start and
scores `/pcl_pose` against the GNSS reference.

| Window | Bag time | Why |
| --- | --- | --- |
| A | 0-60 s | the old 60 s regression start (parked for 15 s, then up to 10 m/s) |
| B | 2820-2970 s | sustained 6-10 % grades |
| C | 2300-2420 s | the fastest climb: 4-5 % at up to 21 m/s |

GNSS is accurate in all three (sigma at most 0.52 m). It is not during 750-1030 s and
3020-3215 s (sigma up to 92 m): the earlier window C at 3160 s started inside that outage,
so the seed and the reference were tens of meters off, and its results (also in
[twist_smoother_slope](../twist_smoother_slope/README.md)) say nothing.

## Result

Median of three runs (range). *Preset* is `971e29a` with the preset as it was (no crop);
*+ crop* is the same build with `enable_local_map_crop: true`, `local_map_radius: 150.0`,
`local_map_update_distance: 10.0`; *fix* is this change with the same crop.

| Window | Variant | Horizontal RMSE m | z RMSE m | Yaw RMSE deg | 3D p95 m |
| --- | --- | --- | --- | --- | --- |
| A | preset | 5.900 (4.122-6.160) | 0.731 (0.731-0.733) | 0.712 (0.463-0.799) | 12.322 (8.267-12.903) |
| A | + crop | 53.044 (49.685-55.307) | 0.647 (0.467-2.512) | 6.343 (3.328-8.711) | 120.036 (119.412-130.277) |
| A | **fix** | **0.261** (0.224-0.267) | **0.236** (0.232-0.256) | 1.850 (1.841-1.866) | **0.514** (0.476-0.589) |
| B | preset | 20.053 (19.695-20.228) | 38.420 (38.420-38.432) | 0.803 (0.789-0.933) | 58.056 (57.523-58.109) |
| B | + crop | 249.014 (235.273-334.694) | 33.232 (28.967-37.411) | 11.788 (10.565-25.801) | 332.022 (331.656-429.600) |
| B | **fix** | **1.106** (0.905-9.065) | **4.231** (0.518-37.894) | 1.766 (1.686-2.891) | **12.451** (2.003-48.903) |
| C | preset | 51.050 (51.030-79.897) | 48.099 (47.772-48.468) | 0.281 (0.265-1.624) | 99.248 (98.788-131.563) |
| C | + crop | 251.893 (220.978-380.528) | 38.186 (34.772-39.882) | 10.325 (8.055-13.328) | 308.469 (262.940-466.047) |
| C | **fix** | **0.511** (0.472-0.585) | **0.618** (0.427-0.797) | 2.117 (2.092-2.151) | **1.494** (1.108-2.356) |

- The crop alone makes every window worse: NDT now registers, and cause 2 throws it off
  once the car moves.
- With the fix, eight of nine runs stay within 1.2 m horizontal RMSE (the six of A and C
  within 0.6 m). The ninth is the window B z failure under *Known limits*.
- Yaw RMSE rises to about 2° wherever NDT registers: that is the GNSS orientation bias
  (cause 1), not the localizer. The preset's low yaw RMSE comes from keeping the GNSS
  initial yaw.
- While parked in window A the fix stays within 0.1-0.3 m of the GNSS position, at NDT
  fitness 0.2.
- The *fix* runs queued 200 twist messages. One run per window with the committed build
  (1000 messages, preset with the crop): 0.223, 0.918 and 0.504 m horizontal RMSE.

## Twist EKF

The EKF compared the NDT result, the pose at the scan stamp, with the state predicted up
to the latest twist, so every update pulled the state back by speed x delay. It now
keeps its predicted trajectory, carries the result along it from the scan stamp to the
state's time (turned by the measured heading), shifts the trajectory by every
correction, and seeds NDT and publishes with the pose at the scan stamp
(`test/test_twist_ekf.cpp`).

The same three windows with `use_twist_ekf: true`, `use_gtsam_smoother: false` and the
crop, against `main` at `2d91060`, median of three runs (range):

| Window | Variant | Horizontal RMSE m | z RMSE m | 3D p95 m |
| --- | --- | --- | --- | --- |
| A | main | 19.047 (14.213-33.546) | 8.721 (6.180-9.675) | 51.949 (38.384-81.512) |
| A | **fix** | **0.253** (0.205-0.340) | **0.501** (0.254-0.804) | **0.836** (0.560-2.068) |
| B | main | 456.330 (412.452-471.294) | 63.798 (22.753-131.827) | 644.058 (559.150-685.918) |
| B | fix | 36.978 (28.515-46.145) | 43.951 (19.919-50.019) | 83.134 (48.802-89.706) |
| C | main | 270.216 (248.568-323.545) | 34.836 (11.362-42.738) | 342.849 (303.544-392.624) |
| C | **fix** | **0.537** (0.516-1.220) | **0.750** (0.728-2.661) | **1.445** (1.369-6.223) |

Window B still fails with the EKF, in z first: on the 6 % descent its z fell 2.5 m below
GNSS within 23 s while NDT matched (fitness 0.4), NDT then rejected the scans, and the
pose diverged. The smoother tracks the same window. Why the EKF's z does not follow the
accepted NDT z is not established here; no preset enables the EKF. Measured 2026-10-28
on ROS 2 Jazzy, 4 cores.

## Known limits

- **z on grades.** The smoother is planar and keeps the last NDT z. In one of the three
  window B runs NDT was rejected for a few scans on the 6 % descent, z fell behind by
  about 1 m/s, and NDT did not recover (9.1 m horizontal, 37.9 m z RMSE). The slope-aware
  z seed rejected in [twist_smoother_slope](../twist_smoother_slope/README.md) was
  measured while NDT did not register; it is worth measuring again on top of this fix.
- **Throughput.** Rebuilding the 150 m crop every 10 m takes about 1.7 s on 4 cores, so
  the pose rate falls to 0.9-3 Hz while driving (0.9 Hz at 21 m/s in window C). A larger `local_map_update_distance` or
  moving the crop out of the state lock would help; not measured here.

Measured 2026-10-27 on ROS 2 Jazzy, 4 cores; baseline = `971e29a`. This is a
public-dataset replay, not a physical-robot test.

## Rerun

```bash
python3 experiments/twist_smoother_slope/prepare.py "$ISTANBUL/localization_bag" /tmp/istanbul_fixture
# BASE_WS built from 971e29a, CAND_WS from this change
python3 experiments/twist_smoother_slope/compare.py \
  --base-ws BASE_WS --cand-ws CAND_WS --fixture /tmp/istanbul_fixture \
  --bag "$ISTANBUL/localization_bag" --map "$ISTANBUL/map/pointcloud_map.pcd" \
  --out /tmp/istanbul_compare \
  --set enable_local_map_crop=false --cand-set enable_local_map_crop=true
```

`replay.py` reads the preset from this checkout, which now crops: the overrides turn the
crop off for the baseline, as the preset was, and back on for the candidate. Pass only
`--set enable_local_map_crop=true` to compare both builds with the crop.
