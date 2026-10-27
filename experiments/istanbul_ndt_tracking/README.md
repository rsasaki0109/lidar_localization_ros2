# Why NDT lost the Autoware Istanbul route

Question: why did `param/istanbul_gtsam_smoother.yaml` drift to 6-24 m horizontal RMSE
on the Istanbul localization bag
([twist_smoother_slope](../twist_smoother_slope/README.md)), although the twist matches
GNSS? Three causes stacked, and each one hid the next.

## 1. The full-map NDT target was empty

Without `enable_local_map_crop` the NDT voxel grid spans the map's bounding box,
15.2 x 12.7 km and 284 m of height: 5.5e10 cells at `ndt_resolution: 1.0` (6.9e9 at 2 m,
2.0e9 at 3 m). pclomp indexes the grid with int32, so it printed
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

`TwistEkf::update()` takes no stamp either and has cause 2 as well. No preset enables
it; it is left for a separate change.

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

RESULTS_TABLE

## Known limits

- **z on grades.** The smoother is planar and keeps the last NDT z. In one of the three
  window B runs NDT was rejected for a few scans on the 6 % descent, z fell behind by
  about 1 m/s, and NDT did not recover (9.1 m horizontal, 37.9 m z RMSE). The slope-aware
  z seed rejected in [twist_smoother_slope](../twist_smoother_slope/README.md) was
  measured while NDT did not register; it is worth measuring again on top of this fix.
- **Throughput.** Rebuilding the 150 m crop every 10 m takes about 1.7 s on 4 cores, so
  the pose rate falls to 1.5-3 Hz while driving. A larger `local_map_update_distance` or
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
