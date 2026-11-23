# Localization Benchmark

`tools/benchmark` replays public datasets through the same two commands a user runs,
odometry plus `quickstart.py --map <map>`, and scores the published pose (`/pcl_pose`,
or `/localization/pose_with_covariance` for a suite that runs `--profile nav2` or
`mid360`) against ground truth.

## What is measured

Every run starts from nothing: no saved pose and no initial pose, so global
initialization has to find the robot. Scores are taken **in the map frame without
any alignment**. The maps are in the ground-truth frame, and aligning the estimate to
the truth first would hide the worst failure: a wrong initialization that is then
tracked consistently. On a Unitree Go2, such runs were 1.8 m off but looked like
0.2 m after alignment.

| column | meaning |
| --- | --- |
| initialized | runs that published a pose |
| wrong init | runs whose first 2 s of poses were more than 1 m from the truth |
| time to first pose | seconds from the start of the replay |
| median / p95 / max xy | horizontal error of the matched poses |
| >1 m | share of poses more than 1 m off (median over runs) |
| tracked | share of the ground-truth time from the first pose to the end of the replay that has a matched pose (median over runs); a localizer that stops publishing loses it |

Runs also record the localizer's per-scan `/alignment_status`. A second table scores it
as a health signal: the share of poses more than 1 m off that it did not report OK
(the higher the better), and of poses within 0.3 m that it did not report OK (false
alarms, the lower the better). A pose with no status in the last second counts as
flagged.

The "lost" columns count only the scans where the localizer requests
reinitialization (or stopped reporting). A scan that NDT rejects while the odometry
bridge carries the pose is WARN, but not lost. The results below were measured
before the "tracked" and "lost" columns existed. See
[experiments/alignment_health](../experiments/alignment_health/README.md) for why
neither column alone separates lost poses from good ones.

## Results

### Summary

| dataset | runs | initialized | wrong init | median xy (m) | p95 xy (m) |
|---|---|---|---|---|---|
| Koide outdoor_hard (01a-02b) | 12 | 12 | 0 | 0.08-0.12 | 0.21-0.91 |
| Koide outdoor_kidnap (a, b) | 6 | 6 | 0 | 0.04-0.12 | 15.85-60.50 |
| JEPLO EIL_*, lidar_slam_ros2 map | 12 | 12 | 0 | 0.03 | 0.09-0.13 |
| JEPLO wide maps and EIL_*, rebuilt inputs, low-sensor settings | 18 | 18 | 0 | 0.02-0.03 | - |
| JEPLO cross-session (map from another run), low-sensor settings | 24 | 23 | 5 | 0.02-0.22 | 0.04-1.81 |

Ranges are over cases. The kidnap p95 comes from the poses while the sensor is
carried, before recovery. The cross-session medians and p95 are over the runs that
started at the right place. The sections below have the per-case tables and what went wrong.

### Per case

Measured 2026-10-06 on `main` (Jazzy, replay at 1x, three runs per case unless noted).

**Handheld MID-360 outdoors (Koide):** every run initialized, none wrongly.

| case | runs | initialized | wrong init | time to first pose (s) | median xy (m) | p95 xy (m) | max xy (m) | >1 m |
|---|---|---|---|---|---|---|---|---|
| outdoor_hard_01a | 3 | 3 | 0 | 15.8 | 0.12 | 0.91 | 1.82 | 0.032 |
| outdoor_hard_01b | 3 | 3 | 0 | 22.2 | 0.08 | 0.21 | 0.87 | 0.000 |
| outdoor_hard_02a | 3 | 3 | 0 | 15.3 | 0.10 | 0.68 | 1.45 | 0.023 |
| outdoor_hard_02b | 3 | 3 | 0 | 12.3 | 0.09 | 0.37 | 128.81 | 0.008 |
| outdoor_kidnap_a | 3 | 3 | 0 | 14.5 | 0.04 | 60.50 | 180.96 | 0.073 |
| outdoor_kidnap_b | 3 | 3 | 0 | 18.6 | 0.12 | 15.85 | 150.50 | 0.328 |

The kidnap cases are far off while the sensor is carried, until recovery. In one
outdoor_hard_02b run the RKO-LIO odometry dropped scans with too few ICP keypoints;
the recovery supervisor then reset on a single unconfirmed answer 128 m away
(the confirmation is waived while odometry is out).

How `/alignment_status` tracked the error on the Koide runs (two per case, with #201):

| case | runs with >1 m poses | flagged when >1 m off | flagged when <0.3 m |
|---|---|---|---|
| outdoor_hard_01a | 2 | 0.98 | 0.13 |
| outdoor_hard_01b | 2 | 0.60 | 0.33 |
| outdoor_hard_02a | 2 | 0.93 | 0.41 |
| outdoor_hard_02b | 2 | 0.41 | 0.20 |
| outdoor_kidnap_a | 2 | 0.61 | 0.06 |
| outdoor_kidnap_b | 2 | 0.17 | 0.02 |

It catches slow loss of tracking (01a, 02a), but it reports OK on most poses after a
wrong reset (01b, kidnap_b: a confident match at the wrong place), and it raises
false alarms on 20-41% of good poses in 01b-02b.

**Unitree Go2 (JEPLO), lidar_slam_ros2 map of EIL_Mix:** every run initializes by
itself and none at a wrong place, and tracking stays within 3 cm (median). Before the
NDT score could judge ambiguity on its own (#201), one run in three ran out of global
attempts on ambiguous answers in the symmetric capture room.

| case | runs | initialized | wrong init | time to first pose (s) | median xy (m) | p95 xy (m) | max xy (m) | >1 m |
|---|---|---|---|---|---|---|---|---|
| EIL_Box | 3 | 3 | 0 | 17.1 | 0.03 | 0.09 | 4.61 | 0.001 |
| EIL_Stairs | 3 | 3 | 0 | 10.9 | 0.03 | 0.13 | 0.94 | 0.000 |
| EIL_Mask1 | 3 | 3 | 0 | 9.8 | 0.03 | 0.10 | 0.94 | 0.000 |
| EIL_Mask2 | 3 | 3 | 0 | 9.8 | 0.03 | 0.13 | 1.19 | 0.000 |
| Long_Stairs_self_map | 3 | 0 | 0 | - | - | - | - | - |
| Outdoor1_self_map | 1 | 0 | 0 | - | - | - | - | - |
| Outdoor2_self_map | 1 | 0 | 0 | - | - | - | - | - |

On the large Long_Stairs and Outdoor maps, global search did not find the Go2 with the
2026-10-06 defaults (before the sparse-band search became the quickstart default in #225).
The cause, measured below: the 2D search saturates. An upside-down MID-360 0.45 m
above the ground leaves about 100 points in the default height band. On a grid
inflated by 0.6 m they fit perfectly at many places, and branch-and-bound returns
eight of those places in search order. The 1.7-2.3 m offsets seen then are corners
of that perfect-fit area around the robot.

**Unitree Go2 wide maps, rebuilt inputs (2026-10-11).** The bags and maps were
rebuilt from the raw bags as above. The EIL cases use an EIL_Mix map accumulated the
same way, not the lidar_slam_ros2 map, so they differ from the table above. Runs:
baseline (defaults) and the [Large maps](go2_mid360_quickstart.md#large-maps)
settings (`go2_jeplo_low_sensor.yaml`), one 4-core machine, Jazzy:

| case | defaults: runs / initialized / wrong / lost | low-sensor settings: runs / initialized / wrong / lost | first pose (s) | median xy (m) |
|---|---|---|---|---|
| Long_Stairs_self_map | 1 / 0 / 0 / - | 2 / 2 / 0 / 0 | 12.3-12.6 | 0.02 |
| Outdoor1_self_map | 1 / 0 / 0 / - | 2 / 2 / 0 / 0 | 11.1-12.6 | 0.02 |
| Outdoor2_self_map | 1 / 0 / 0 / - | 2 / 2 / 0 / 0 | 11.1-14.6 | 0.02 |
| EIL_Box | 4 / 4 / 1 / 1 | 3 / 3 / 0 / 0 | 9.1-10.1 | 0.03 |
| EIL_Stairs | 4 / 4 / 0 / 0 | 3 / 3 / 0 / 0 | 9.4 | 0.02 |
| EIL_Mask1 | 4 / 4 / 3 / 2 | 3 / 3 / 0 / 0 | 8.8-10.1 | 0.02 |
| EIL_Mask2 | 4 / 4 / 1 / 1 | 3 / 3 / 0 / 0 | 9.3-9.5 | 0.02 |

"Lost" counts runs whose median error is above 1 m. The first-pose times and medians are
for the low-sensor runs. On this EIL map the defaults started wrongly in 5 of 16 runs,
EIL_Mask1 three times 1.1-2.0 m off, EIL_Mask2 once turned 180 degrees, and EIL_Box once
2 m off, after which tracking settled 4.5 m along the aisle.

See [experiments/go2_wide_map_global_init](../experiments/go2_wide_map_global_init/README.md)
for the diagnosis, the compared settings, and the EIL_Box start that falls on the
Go2 stepping off a box.

### Cross-session (Go2, JEPLO)

Measured 2026-11-18 on `main` at `6303352`, Jazzy, replay at 1x,
two runs per pair, the [Large maps](go2_mid360_quickstart.md#large-maps) settings, no
initial pose ([`go2_jeplo_cross_session.yaml`](../tools/benchmark/suites/go2_jeplo_cross_session.yaml)).
Each map is built from one run only (RKO-LIO offline odometry,
`build_map_from_odometry.py`, `align_map_to_ground_truth.py`, residual 0.034-0.075 m)
and the scans come from another run. Ground truth covers only the capture room.
The table below is v1.11.0; its wrong starts no longer happen since the startup checks
NDT-refined fixes more strictly ([why they passed, and the fix](#why-the-wrong-starts-passed-and-the-fix)).

| map → scans | initialized / wrong / manual pose | first pose (s) | first-pose error (m) | queries | median / p95 / max xy (m) |
|---|---|---|---|---|---|
| Long_Stairs → Outdoor2 (0-165 s) | 2 / 0 / 0 | 13.3, 13.3 | 0.02, 0.02 | 2, 2 | 0.02 / 0.04 / 0.07 |
| Outdoor2 (0-165 s) → Long_Stairs | 2 / 0 / 0 | 12.6, 12.3 | 0.02, 0.02 | 2, 2 | 0.02 / 0.04 / 0.10-0.14 |
| EIL_Mix → EIL_Box | 2 / 0 / 0 | 8.8, 9.6 | 0.01, 0.01 | 2, 2 | 0.03 / 0.09 / 0.65 |
| EIL_Mix → EIL_Stairs | 2 / 0 / 0 | 7.6, 9.4 | 0.01, 0.03 | 2, 2 | 0.02-0.03 / 0.84 / 1.30 |
| EIL_Mix → EIL_Mask1 | 2 / 0 / 0 | 10.0, 8.8 | 0.01, 0.05 | 2, 2 | 0.02 / 0.06 / 0.15-0.20 |
| EIL_Mix → EIL_Mask2 | 2 / 0 / 0 | 9.8, 9.5 | 0.02, 0.02 | 2, 2 | 0.02 / 0.04 / 0.07-0.16 |
| EIL_Box → EIL_Mix | 2 / **2** / 0 | 14.6, 9.3 | **5.50, 1.72** | 6, 2 | 4.22 and 0.03 / 7.37 and 1.50 / 8.36 and 2.05 |
| EIL_Box → EIL_Stairs | 2 / **2** / 0 | 10.4, 10.4 | **1.05, 1.05** | 4, 3 | 1.94 / 3.92 / 5.24 |
| EIL_Box → EIL_Mask1 | 2 / 0 / 0 | 10.0, 10.0 | 0.03, 0.03 | 3, 4 | 0.03 / 0.08-0.10 / 0.28-0.46 |
| Long_Stairs → EIL_Box | 2 / 0 / 0 | 18.1, 11.1 | 0.16, 0.04 | 6, 2 | 0.15-0.19 / 1.54-1.72 / 1.77-2.55 |
| Long_Stairs → EIL_Stairs | 2 / 0 / 0 | 12.1, 12.1 | 0.01, 0.01 | 3, 3 | 0.11-0.22 / 0.75-1.81 / 1.05-2.24 |
| Long_Stairs → EIL_Mask1 | 1 / **1** / **1** | -, 18.0 | -, **1.73** | 6, 5 | 1.77 / 3.11 / 3.30 (run 2) |
| *controls, own map:* EIL_Box → EIL_Box | 1 / 0 / **1** | -, 8.8 | -, 0.02 | 6, 2 | 0.02 / 0.05 / 0.31 |
| EIL_Stairs → EIL_Stairs | 2 / 0 / 0 | 9.4, 9.4 | 0.03, 0.03 | 2, 2 | 0.02 / 0.06 / 0.56 |
| EIL_Mix → EIL_Mix | 2 / 0 / 0 | 7.1, 7.8 | 0.02, 0.02 | 2, 2 | 0.02 / 0.05-0.06 / 0.28 |

"Wrong" is a first pose more than 1 m from the truth; "manual pose" is a run that
used up its six global attempts and asked for a pose (`needs_operator`). Values are
per run (run 1, run 2) or the range over both. Ground truth covers the capture room
only; on the Long_Stairs pair the whole run, corridor and staircase included, was also
compared with the test run's own offline odometry fitted to its ground truth (a
reference, not ground truth): median 0.02-0.03 m, max 0.19-0.22 m.

What differs between the sessions, and what it did:

- **Long_Stairs and Outdoor2** were recorded 1.5 h apart on one day along the same
  route (room, corridor, long staircase). The room was not rearranged: 99-100% of the
  room points of each map lie within 0.1-0.2 m of the other's. Both directions
  localized in every run with the two queries ranked #1 at 0.01-0.08 m, and tracked
  down the staircase. This is the
  [README GIF](readme-media.md#global-localization-on-a-go2).
- **The EIL runs move props** (boxes, a stair module) around the same room, on other
  days (2026-08-23 to 08-26; EIL_Mix has no date). 73-85% of one room map's points lie
  within 0.1-0.2 m of another's. The EIL_Mix map holds two props in the middle of the
  room; the EIL_Box map a box platform about 3.8 m x 1.7 m.
- **EIL_Mask1 and EIL_Mask2 have part of the sensor covered**: in EIL_Mask2 a
  60 degree sector of the scan holds about a third of its usual points, in EIL_Mask1
  the front 60 degrees about four fifths.
- **On the EIL_Mix map** every start was right. EIL_Box jumped 0.65 m for about 1 s
  at 31 s, and EIL_Stairs ran 0.5-1.3 m off along the room for 18 s (111-129 s), in both
  runs. On their own maps neither happened (max 0.31 m and 0.56 m).
- **On the EIL_Box map** the global search was unreliable even for its own run: one of
  the two self-map runs saw top candidates 1.5-9.3 m off in six queries and asked for a
  manual pose. On the other runs it published four wrong first poses (1.05-5.50 m off,
  one of them after six queries) whose two queries had agreed; one EIL_Mix run then
  converged back to 0.03 m (median). In EIL_Mask1 the consensus check rejected the wrong
  answers and both runs started right.
- **On the Long_Stairs map** (a room without the EIL props, plus the corridor and the
  staircase), five of six starts were right, but tracking slid 1-2.5 m along the room's
  long axis (y within 0.1 m) for up to 40 s while the Go2 walked on the floor between
  the props it climbs in these runs, and came back when it was on them. EIL_Mask1 once
  asked for a manual pose after six ambiguous queries (top candidates 3-14 m off) and
  once started 1.73 m off and stayed there.

**Map-change probe** on the EIL_Mix map (edited copies, two runs each): deleting every
map point 0.2-1.6 m high in the room interior (the props, 12,730 of 579,717 points) or
adding a 3 m x 2 m wall across the room that the scans do not see.

| map edit → scans | initialized / wrong / manual pose | first pose (s) | first-pose error (m) | queries | median / p95 / max xy (m) |
|---|---|---|---|---|---|
| props deleted → EIL_Box | 2 / 0 / 0 | 10.1, 9.8 | 0.01, 0.01 | 2, 2 | 0.02 / 0.10 / 0.24-0.26 |
| props deleted → EIL_Stairs | 2 / 0 / 0 | 11.4, 11.4 | 0.02, 0.02 | 4, 4 | 0.02 / 0.60 / 0.81 |
| wall added → EIL_Box | 2 / 0 / 0 | 9.6, 9.8 | 0.01, 0.01 | 2, 2 | 0.03 / 0.09-0.11 / 0.38-0.39 |
| wall added → EIL_Stairs | 2 / 0 / 0 | 9.4, 9.4 | 0.03, 0.03 | 2, 2 | 0.03 / 0.72 / 0.98 |

Neither edit caused a wrong start. Without the EIL_Mix props, EIL_Box and EIL_Stairs
tracked closer than on the unedited map (max 0.26 m and 0.81 m against 0.65 m and
1.30 m), i.e. props that are somewhere else in the test run hurt more than missing ones.
With props deleted, EIL_Stairs needed four queries (one consensus mismatch rejected a
candidate 1.22 m off).

Not verified: two runs per pair, one machine. JEPLO has one Long_Stairs recording and
one other run on its route (Outdoor2, used up to 165 s, converted from the first 1.5 GB
of the bag). The EIL_Box map's failure on its own run suggests that map, not only the
session change, is a weak case; other room maps of the EIL runs were not tried. The
probe edits are synthetic.

#### Why the wrong starts passed, and the fix

Measured 2026-11-19, same suite and settings, with the topics and frames pinned on the
command line (one baseline run lost its sensor topics to a slow bag start; EIL_Box →
EIL_Mix run 1 of the baseline is a rerun). The Long_Stairs scans for
Outdoor2 → Long_Stairs were no longer on disk, so that pair was not rerun.

Every wrong start came from NDT-refined G2 fixes (the Large maps settings refine, and so
does the sparse-band search) that the startup accepted although they disagreed or
were not distinctive:

- **Two queries 1.6-1.8 m apart "agreed".** The agreement bound is 2 m plus 5% of the
  distance travelled, sized for the 5-degree heading steps of unrefined fixes. In EIL_Box
  → EIL_Mix the first fix was 0.07 m off, the second 1.83 m along the room (NDT fitness
  0.085 against 0.096), they matched within the 2 m, and the second, wrong one was
  published. Long_Stairs → EIL_Mask1 did the same (0.01 m, then 1.78 m).
- **The room turned by 180 degrees.** In EIL_Box → EIL_Stairs both fixes were the
  near-symmetric room the other way round, 0.1-1.9 m from the true position, and their
  NDT fitness was as good as or better than the true pose's (the EIL_Box map holds a box
  platform where EIL_Stairs has a stair module). The distinctiveness check compares the
  top only with candidates at least 5 m away, so neither the turned pose at the same
  place nor an aliased place 1-3 m along the room counted. NDT refinement makes such a
  nearby candidate a separate optimum rather than a duplicate.
- Not the occupancy band or the score margin: the truth was often not among the 32
  candidates at all (3 m non-maximum suppression), and when it was, the BBS scores were
  saturated near 1 for both.

Offline, `g2_offline_probe.py` asked G2 about one scan per second of each test run
(14 pairs, 1,057 queries, ground truth as odometry); 365 tops were wrong (> 1 m or
> 20 degrees). A refined candidate 1 m or more away or turned 20 degrees or more with
fitness within 2x of the top's flagged 315 of the wrong tops and 99 of the 692 right
ones; the existing 5 m check flagged 238 and 60. Replaying the answers through
`decide_startup` from every query as a start point:

| startup rule (simulated, 889 start points) | wrong | right | manual pose |
|---|---|---|---|
| before | 154 | 706 | 29 |
| refined fixes agree within 0.5 m and 10 degrees | 67 | 752 | 70 |
| + a refined alternative within 5 m counts (1 m or 20 degrees away) | **41** | **750** | 98 |
| both, with 1.5 m non-maximum suppression (not adopted; 6 of the pairs, 442 start points; both checks alone there: 36 / 329 / 77) | 4 | 278 | 160 |

Both checks are on by default and only tighten the existing ones (for unrefined fixes
nothing changes). The remaining simulated wrong starts are on EIL_Box → EIL_Stairs and
Long_Stairs → EIL_Stairs, where consecutive answers agree tightly on the turned room or
on a place 1.5-2 m along it. A narrower suppression radius removed most of them but
cost a sixth of the right starts.

Live, before → after (28 runs each):

| map → scans | before: initialized / wrong / manual | after | first pose (s), after | first-pose error (m), after | queries, after |
|---|---|---|---|---|---|
| Long_Stairs → Outdoor2 (0-165 s) | 2 / 0 / 0 | 2 / 0 / 0 | 14.3, 13.8 | 0.01, 0.02 | 2, 2 |
| EIL_Mix → EIL_Box | 2 / 0 / 0 | 1 / 0 / **1** | -, 10.3 | -, 0.01 | 6, 2 |
| EIL_Mix → EIL_Stairs | 2 / 0 / 0 | 1 / 0 / **1** | 8.4, - | 0.02, - | 2, 6 |
| EIL_Mix → EIL_Mask1 | 2 / 0 / 0 | 2 / 0 / 0 | 10.5, 10.3 | 0.01, 0.01 | 2, 2 |
| EIL_Mix → EIL_Mask2 | 1 / 0 / **1** | 1 / 0 / **1** | 7.8, - | 0.03, - | 2, 6 |
| EIL_Box → EIL_Mix | 2 / **1** / 0 | 2 / 0 / 0 | 11.3, 10.8 | 0.03, 0.03 | 3, 3 |
| EIL_Box → EIL_Stairs | 2 / **2** / 0 | 2 / 0 / 0 | 13.1, 11.4 | 0.01, 0.02 | 5, 4 |
| EIL_Box → EIL_Mask1 | 2 / 0 / 0 | 2 / 0 / 0 | 11.3, 8.8 | 0.02, 0.02 | 3, 2 |
| Long_Stairs → EIL_Box | 0 / 0 / **2** | 2 / 0 / 0 | 11.3, 12.1 | 0.05, 0.09 | 2, 3 |
| Long_Stairs → EIL_Stairs | 1 / 0 / **1** | 1 / 0 / **1** | 13.9, - | 0.02, - | 4, 6 |
| Long_Stairs → EIL_Mask1 | 2 / **1** / 0 | 2 / 0 / 0 | 15.5, 17.5 | 0.13, 0.07 | 4, 6 |
| *controls:* EIL_Box → EIL_Box | 2 / 0 / 0 | 2 / 0 / 0 | 4.6, 10.8 | 0.02, 0.02 | 2, 3 |
| EIL_Stairs → EIL_Stairs | 2 / 0 / 0 | 2 / 0 / 0 | 10.4, 12.6 | 0.03, 0.01 | 3, 5 |
| EIL_Mix → EIL_Mix | 2 / 0 / 0 | 2 / 0 / 0 | 10.1, 10.1 | 0.03, 0.04 | 2, 2 |
| **total** | 24 / 4 / 4 | 24 / **0** / 4 | | | |

Right starts went from 20 to 24 of 28 and wrong ones from 4 to 0; four runs still asked
for a manual pose (other runs than before). In each of the four the top candidate was
never right in its six queries (EIL_Mix → EIL_Box run 1 saw the room turned by 123-176
degrees, EIL_Mix → EIL_Stairs run 2 and Long_Stairs → EIL_Stairs run 2 places
1.1-2.8 m off), so none rejected a right answer. The first right pose came after a
median 11.0 s (10.3 s before, max 17.5 s against 15.4 s): one or two more queries where
the first answers were aliased. The README GIF pair (Long_Stairs → Outdoor2) localized
in 2 queries both times, as before. Tracking after a right start is unchanged; the
Long_Stairs map still lets EIL_Box and EIL_Stairs slide up to 2-2.4 m along the room,
which the startup cannot see ([why, and the opt-in fix](#why-tracking-slid-along-the-room-and-the-fix)).

#### Why tracking slid along the room, and the fix

Measured 2026-11-21 and 22 on `fix/corridor-slide` (from `60adda3`), same machine and
settings. To separate tracking from the startup, the slide was reproduced from a known
pose: quickstart with `--initial-pose` set to the motion-capture pose 1 s into the bag,
odometry and localizer started before the bag plays from there, and
`enable_registration_localizability_diagnostics` on (it only reports the NDT Hessian).

From the true pose, Long_Stairs → EIL_Box still slid to 1.8 and 2.7 m and
Long_Stairs → EIL_Stairs to 1.3 and 2.2 m, so the startup was not the cause. What the
runs show:

- **The room is weak along its long axis everywhere, not only during the slide.** The
  weakest direction of the NDT Hessian points along the room (x component 0.99 or more)
  in every scan. With the pose right, its eigenvalue is 4-5 times below the next one and
  about 1% of the strongest (median ratio 0.012, p5 0.006); once slid, 0.005. A
  degeneracy threshold on that ratio flags the whole run or nothing, so it cannot tell a
  slide from normal tracking.
- **Degraded scans move the pose a little at a time, each within the guards.** In
  EIL_Stairs at 15-17 s the Go2 walks past the stair module, which the Long_Stairs map
  lacks: the scans fit worse (fitness 0.05-0.09 instead of 0.02, inlier ratio 0.89-0.94
  instead of 0.98, 20-35% fewer NDT correspondences) and every one was accepted, each
  moving the pose 0.03-0.17 m beyond the odometry prediction, all the same way. That is
  under the 0.3 m seed correction guard and far under the fitness gate (6.0), and it
  added up to 0.8 m in 1.2 s and 1.25 m in 2 s. In EIL_Box at 31 s and 50 s a burst of
  scans with half the usual correspondences (23,000-30,000 instead of 52,000-66,000),
  fitness 0.07-0.39 and an inlier ratio of 0.77-0.90 did the same faster.
- **Then the guard holds the wrong pose.** The next well-fitting scans (fitness
  0.005-0.02, inlier ratio 0.98-1.0) asked for the 0.4-0.9 m back, and the seed
  correction guard rejected them as jumps; NDT then settled in a neighbouring optimum
  1.2-1.6 m along the room (fitness 0.04-0.05 instead of 0.01) and stayed there until the
  Go2 climbed a prop. When the slide was short (0.5 m at 31 s in EIL_Box) the guard
  released after 30 rejections and the pose came back.
- **The odometry mostly stayed right.** RKO-LIO's displacement over any 0.5, 1 or 3 s
  window differed from the motion capture by 0.05-0.08 m (median) and 0.2 m (p95), so
  it kept the pose that NDT left (also visible in the runs: while scans were rejected,
  the error stayed where it was). It jumps by itself, though: in EIL_Box it went 0.9 m
  along the room at 50 s, together with NDT, and about 1 m back at 62 s, and on the
  stair module it moves 0.3-0.5 m back and forth now and then.

**The fix** (opt-in): `enable_seed_correction_window_guard` bounds what the corrections
accepted within `seed_correction_window_guard_sec`, plus the current one, add up to
(horizontal norm of the map-frame sum, `seed_correction_window_guard_translation_m`,
0.3 m). A pull back toward the odometry cancels earlier corrections and passes. It
shares the seed correction guard's warmup and its release after 30 rejections in a row,
and the quickstart turns it on with `--seed-correction-window-sec` when it seeds from
odometry. The window length trades the two failure modes: a short one lets a sustained
slide through at 0.3 m per window, a long one keeps NDT from following the odometry's
own jumps.

From the true pose (max / p95 xy, m; two runs, one for 3 s):

| setting | Long_Stairs → EIL_Box | Long_Stairs → EIL_Stairs |
|---|---|---|
| before | 2.71, 1.82 / 1.76, 1.59 | 2.24, 1.31 / 1.68, 0.84 |
| window 1 s | 0.69, 0.40 / 0.22, 0.21 | 1.02, 0.99 / 0.70, 0.65 |
| **window 2 s** | **0.40, 0.40 / 0.21, 0.21** | **0.90, 0.55 / 0.27, 0.27** |
| window 3 s | 0.97 / 0.61 | 0.55 / 0.29 |

At 3 s EIL_Box could not follow the odometry's 1 m jump at 62 s. The remaining errors
with 2 s are odometry jumps the per-scan guard holds for a moment.

Live with global initialization and `--seed-correction-window-sec=2.0`, against the
"after" runs of the previous section (median / p95 / max xy in m per run; "-" is a
run that asked for a manual pose):

| map → scans | before | window 2 s |
|---|---|---|
| Long_Stairs → EIL_Box | 0.09 / 0.70 / 0.94; 1.09 / 1.76 / 2.01 | 0.08 / 0.21 / 0.40; 0.08 / 0.21 / 0.40 |
| Long_Stairs → EIL_Stairs | 0.81 / 1.90 / 2.40; - | -; 0.05 / 0.26 / 0.60 |
| Long_Stairs → EIL_Mask1 | 0.08 / 0.39 / 0.80; 0.05 / 0.39 / 0.78 | 0.07 / 0.18 / 0.34; 0.05 / 0.27 / 0.36 |
| Long_Stairs → Outdoor2 (0-165 s) | 0.02 / 0.04 / 0.07; 0.02 / 0.04 / 0.09 | 0.02 / 0.04 / 0.08; 0.02 / 0.04 / 0.07 |
| EIL_Mix → EIL_Box | -; 0.03 / 0.09 / 0.65 | 0.03 / 0.09 / 0.45; 0.03 / 0.08 / 0.31 |
| EIL_Mix → EIL_Stairs | 0.02 / 0.84 / 1.30; - | 0.02 / 0.12 / 0.54; 0.02 / 0.12 / 0.51 |
| EIL_Mix → EIL_Mask1 | 0.02 / 0.06 / 0.17; 0.02 / 0.06 / 0.17 | 0.02 / 0.07 / 0.17; 0.02 / 0.06 / 0.17 |
| EIL_Mix → EIL_Mask2 | 0.02 / 0.05 / 0.49; - | 0.02 / 0.04 / 0.23; 0.02 / 0.05 / 0.07 |
| EIL_Box → EIL_Mix | 0.03 / 0.15 / 0.49; 0.03 / 0.17 / 0.75 | 0.03 / 0.10 / 0.43; 0.03 / 0.08 / 0.49 |
| EIL_Box → EIL_Stairs | 0.02 / 0.60 / 0.79; 0.02 / 0.14 / 0.75; 0.02 / 0.24 / 0.69; 0.02 / 0.20 / 0.54 | 0.02 / 0.34 / 0.47; 0.03 / 2.87 / 3.12; 0.02 / 0.13 / 0.45; 0.02 / 0.14 / 0.45 |
| EIL_Box → EIL_Mask1 | 0.03 / 0.11 / 0.79; 0.03 / 0.10 / 0.46 | 0.03 / 0.10 / 0.46; 0.03 / 0.08 / 0.17 |
| *controls:* EIL_Box → EIL_Box | 0.02 / 0.05 / 0.20; 0.02 / 0.05 / 0.19 | 0.02 / 0.05 / 0.52; 0.02 / 0.05 / 0.33 |
| EIL_Stairs → EIL_Stairs | 0.02 / 0.06 / 0.59; 0.02 / 0.06 / 0.30 | 0.02 / 0.06 / 0.35; 0.02 / 0.06 / 0.23 |
| EIL_Mix → EIL_Mix | 0.02 / 0.06 / 0.27; 0.02 / 0.06 / 0.28 | 0.02 / 0.06 / 0.28; - |

EIL_Box → EIL_Stairs has two more runs each way, run on the same day as the others. Every
slide over 1 m is gone except one: in EIL_Box → EIL_Stairs run 2 RKO-LIO jumped 0.5 m
at 105 s, the per-scan seed correction guard rejected 30 scans in a row while the
window guard rejected one, and its release then accepted a 2.4 m aliased answer on the
EIL_Box map (box platform where EIL_Stairs has the stair module). The other three runs
of that pair stayed within 0.45-0.47 m (before: 0.54-0.79 m). On the own maps the
window guard rejected no scan of EIL_Box and 16-19 of about 1,260 of EIL_Stairs; the
EIL_Box control's max of 0.33-0.52 m against 0.19-0.20 m is therefore not the guard.
Startups were not affected: 26 of 28 runs initialized, none wrong, two asked for a
manual pose (four before, in other runs).

Not verified: the window was chosen on the two Long_Stairs pairs it is measured on; two
runs per pair, one machine, one robot and one odometry. It is off by default and only
measured with RKO-LIO odometry seeding. The guard does not help when the odometry itself
jumps. The guards' release after 30 rejections in a row accepted an aliased answer in
EIL_Box → EIL_Stairs run 2; [the next section](#why-a-release-locked-onto-an-aliased-answer-and-the-fix)
makes the release check its answer.
To reproduce the live table, add `--seed-correction-window-sec=2.0` to the suite's
`quickstart_args`; the start-from-truth runs used a scratch harness (quickstart
`--dry-run --initial-pose`, then the printed launch command with the bag played from 1 s).

To reproduce, build the maps as in [Rebuilding the JEPLO bags and self maps](#rebuilding-the-jeplo-bags-and-self-maps)
(Outdoor2: `jeplo_to_pc2_bag.py ... bags_pc2_merged/Outdoor2_0-165s --duration 165`,
its map under `maps_loc/Outdoor2_0-165s`; the EIL_Mix, EIL_Box and EIL_Stairs maps from
their own runs), then run
`python3 tools/benchmark/run_benchmark.py tools/benchmark/suites/go2_jeplo_cross_session.yaml --out results/go2_cross`.
Query counts and manual-pose requests are read from each run's `quickstart.log`
(`query answered:` lines) and `status.jsonl` (`needs_operator`).

#### Why a release locked onto an aliased answer, and the fix

Measured 2026-11-22 on `fix/guard-release-alias` (from `0319d70`), same machine and
settings. **What happened in EIL_Box → EIL_Stairs run 2** (window 2 s, live), from its
`/alignment_status`: RKO-LIO jumped 0.6 m at 105.1 s. The next scans asked for
0.35-0.57 m back toward the truth (the pose was 0.34-0.62 m off), and the 0.3 m per-scan
guard rejected them; a few NDT answers near the jumped seed were accepted on the way and
reset the count. Then, at 110.2-110.8 s, NDT answered 1.9-2.5 m the other way, onto the
box platform of the EIL_Box map, five times in six scans. The 30th rejection in a row
fell on one of those (110.8 s): the release accepted it and the pose sat 2.4-3.1 m off
for 19 s, until 130 s. The alias belongs to the place and the map, not to the jump:
in run 1, tracking right, NDT also answered 2.0-2.5 m at 110.9-111.3 s, and the guard
rejected it as designed. The release takes whatever the 30th scan returns, so whether a
run ends 3 m off depends on where the count happens to end.

To reproduce it on demand, a relay published `odom -> odom_raw` under RKO-LIO's
`odom_raw -> livox_frame`, identity until a set time and then a fixed 0.7 m map-frame
offset toward the alias (quickstart `--initial-pose` at the true pose, bag played from
1 s). In the run with the jump at 107.5 s, the 30th rejection came at 115.2 s while NDT
answered 2.0-2.3 m onto the platform (fitness 0.15-0.21, against 0.04-0.07 for the
answers toward the truth): the old release would have taken 2.42 m from a pose 0.75 m
off.

**The fix** (opt-in): with `seed_correction_guard_release_agreement_scans` N > 0, a
released answer is accepted only if its map-frame correction (horizontal offset and yaw
from its seed) lies within `seed_correction_guard_release_agreement_translation_m`
(0.3 m) and `_yaw_deg` (5 degrees) of each of the last N corrections the guards
rejected, i.e. NDT has kept pointing at the same place. Otherwise the guards keep the
odometry pose and report `seed_correction_guard_release_disagreed`, and the next
agreeing answer is released. An odometry jump gives the same correction scan after
scan, so it passes; an alias that flickers in for a second does not. A hold that never
ends leaves the pose on the odometry until the rejection streak raises the
reinitialization request, as before. The quickstart sets N = 10 together with
`--seed-correction-window-sec`; 0 (the default) releases as before. N is the trade:
an alias that persists N scans is still taken, and a hold delays a right release while
the odometry drifts or until an outlier leaves the last N.

What the hold did in every run where it engaged (N = 10, `consecutive_rejected_updates`
at least 30):

| run | answer held at the 30th rejection | outcome |
|---|---|---|
| jump 107.5 s (from the true pose) | 2.42 m onto the platform, pose 0.75 m off | held 27 scans, then released 0.75 m, 0.01 m off |
| jump 111.0 s | 2.24 m onto the platform, pose 0.92 m off | held; released right 12 s later (a small accepted correction near the jump restarted the count) |
| jump 108.0 s, 108.5 s | 0.69, 0.73 m (right; NDT had answered onto the platform until 4 scans earlier) | held 7 scans until those left the last 10, then released, 0.01 m off |
| 3 runs started at 95 s (RKO-LIO initialized while walking) | 1.0-1.1 m (right) twice; 2.65 m onto the platform once | held 9-18 scans while the odometry drifted, then released right; the third ran 7.4 m off once RKO-LIO diverged (4-8 m in seconds) |
| EIL_Mix → EIL_Stairs, live | 1.03 m (right) | held 1 scan |
| EIL_Stairs → EIL_Box, live (wrong start, 1.5-2 m off) | 0.36 m | held 68 scans; neither release fixed the start |

From the true pose with the same jumps, the paired runs without the hold (six jump
times, 107.5-111.0 s) all happened to release on a right answer (max 0.57-1.08 m); with
it, max 0.71-1.08 m (0.85 and 1.08 m in the two that held an alias). In the six runs
started at 95 s (jumps 107.6-108.4 s), two without the hold ended 3.5 m off (a release
onto a wrong answer) and 7.6 m off (RKO-LIO diverged); with it, one, the divergence
above. The hold changes nothing unless a streak reaches 30.

Live with global initialization, window 2 s, against #342's runs (median / p95 / max
xy in m per run; "-" asked for a manual pose):

| map → scans | window 2 s | window 2 s + release agreement |
|---|---|---|
| Long_Stairs → EIL_Box | 0.08 / 0.21 / 0.40; 0.08 / 0.21 / 0.40 | 0.08 / 0.21 / 0.44; 0.08 / 0.23 / 0.68 |
| Long_Stairs → EIL_Stairs | -; 0.05 / 0.26 / 0.60 | 0.06 / 0.46 / 0.78; 0.05 / 0.26 / 0.51 |
| Long_Stairs → EIL_Mask1 | 0.07 / 0.18 / 0.34; 0.05 / 0.27 / 0.36 | 0.09 / 0.25 / 0.33; 0.04 / 0.21 / 0.38 |
| Long_Stairs → Outdoor2 (0-165 s) | 0.02 / 0.04 / 0.08; 0.02 / 0.04 / 0.07 | 0.02 / 0.04 / 0.07; 0.02 / 0.04 / 0.07 |
| EIL_Mix → EIL_Box | 0.03 / 0.09 / 0.45; 0.03 / 0.08 / 0.31 | 0.03 / 0.08 / 0.36; - |
| EIL_Mix → EIL_Stairs | 0.02 / 0.12 / 0.54; 0.02 / 0.12 / 0.51 | 0.02 / 0.47 / 1.40; 0.02 / 0.19 / 0.52 |
| EIL_Mix → EIL_Mask1 | 0.02 / 0.07 / 0.17; 0.02 / 0.06 / 0.17 | 0.02 / 0.06 / 0.17; 0.02 / 0.06 / 0.17 |
| EIL_Mix → EIL_Mask2 | 0.02 / 0.04 / 0.23; 0.02 / 0.05 / 0.07 | 0.02 / 0.05 / 0.24; 0.02 / 0.04 / 0.22 |
| EIL_Box → EIL_Mix | 0.03 / 0.10 / 0.43; 0.03 / 0.08 / 0.49 | -; 0.03 / 0.16 / 0.44 |
| EIL_Box → EIL_Stairs | 0.02 / 0.34 / 0.47; 0.03 / 2.87 / 3.12; 0.02 / 0.13 / 0.45; 0.02 / 0.14 / 0.45 | 0.02 / 0.13 / 0.46; 0.02 / 0.15 / 0.49; 0.02 / 0.13 / 0.38; 0.02 / 0.13 / 0.41 |
| EIL_Box → EIL_Mask1 | 0.03 / 0.10 / 0.46; 0.03 / 0.08 / 0.17 | 0.03 / 0.11 / 0.89; - |
| *controls:* EIL_Box → EIL_Box | 0.02 / 0.05 / 0.52; 0.02 / 0.05 / 0.33 | 0.02 / 0.05 / 0.35; 0.02 / 0.05 / 0.26 |
| EIL_Stairs → EIL_Stairs | 0.02 / 0.06 / 0.35; 0.02 / 0.06 / 0.23 | 0.02 / 0.06 / 0.42; 0.02 / 0.06 / 0.31 |
| EIL_Mix → EIL_Mix | 0.02 / 0.06 / 0.28; - | 0.02 / 0.05 / 0.35; 0.02 / 0.06 / 0.30 |
| EIL_Stairs → EIL_Box | (not run) | 1.75 / 2.84 / 3.13 (wrong start); 0.06 / 2.18 / 2.81 (wrong start); 0.04 / 0.13 / 1.17; 0.04 / 0.17 / 0.31 |

The hold engaged in two live runs (above); elsewhere two streaks reached 30 and their
answers agreed, and no other streak got there, so those runs differ from #342's only by
run-to-run variation. That includes EIL_Box → EIL_Stairs:
its four new runs never had more than 10 rejections in a row, so they say nothing about
the release (the failure was 1 run in 4 before). EIL_Stairs → EIL_Box is new
(the pair the other way round): 2 of 4 runs started wrong with and without the
agreement (4 more runs with it forced off: 5.15 and 4.94 m wrong starts, two right at
0.32-0.49 m), so that is G2 on the EIL_Stairs map, not the release. Startups: 25 of the
28 runs of #342's cases initialized, none wrong, three asked for a manual pose (#342:
26, none wrong, two); the startup does not reach the release.

Not verified: one alias, one map pair and one odometry; N = 10 and 0.3 m were chosen
from the alias seen here (1-2 s long, 2.4 m away), not swept. The jump runs are single
runs per setting and RKO-LIO does not repeat exactly, so the paired runs differ by more
than the release. It does not release earlier: a right correction still waits for 30
rejections (EIL_Mix → EIL_Stairs run 1 rode a 1 m odometry jump for 6 s). To reproduce
the live table, run the suite with `--seed-correction-window-sec=2.0` in
`quickstart_args` (the agreement comes with it).

#### EIL_Stairs → EIL_Box: why starts were wrong, and the fix

Measured 2026-11-22 on `main` at `9973ee4`. The pair is now a case of the suite. The
eight earlier live runs (four with the release agreement, four without) had four wrong
starts, for three different reasons:

| run | what happened |
|---|---|
| with agreement, runs 1 and 2 (1.76, 1.79 m off) | identical answers: query 1 a place 1.6 m along the room (rejected as ambiguous, kept as an unconfirmed fix), query 2 right (fitness 0.027), query 4 the same place 1.8 m along the room as query 1 (fitness 0.15, passed the gates), confirmed by query 1 through odometry and published |
| without agreement, run 4 (4.62 m off) | G2 was right: the published pose was 0.05 m from the ground truth. The localizer's first two scans corrected it by 0.60 m and 1.88 m (fitness 0.13, 0.42) while the robot walked at 0.7 m/s and turned, onto the room turned around 4.6 m away, and the seed correction guard, still in its warm-up after the initial pose, let them through |
| without agreement, run 1 (5.15 m off) | quickstart's discovery missed the odometry, so the search used the raw scan (roll 0 instead of 177 degrees: the MID-360 hangs upside down, so the scan was mirrored) and the startup had no odometry; two answers on the room turned around agreed while the robot walked 1.4 m. Verification rejected the pose 8 s later and the run handed over to the operator, but one wrong pose had been published |

The first two are the G2 search. Offline (`experiments/3d_bbs_eval/eval_global_search.py`, one
scan per 0.5 s over the first 60 s, ground truth for levelling) the top was within 1 m
for 55% of the scans; for 48 of the 120 no candidate was within 1 m of the truth, and in
42 of those the true pose registered better than the returned top (NDT fitness 0.033
against 0.149, median), the top being a place 1-2 m along the room at the same heading.
The G2 search suppresses candidates within `nms_radius_m` (quickstart: 3 m) of a
better one by the 2D grid score, before NDT refinement, so where the aliased place
scores higher on the grid the true pose is never refined. With 1.5 m the top was right
for 89% of the scans and the truth was a candidate for 99%. Agreement over motion does
not help: the alias is the room shifted along itself, so it moves with the robot like
the right answer, and query 1 and 4 above agree through odometry. Neither does a
fitness comparison across views: a startup rule that refused a fix contradicted by an
earlier, clearly better registered fix (query 2 above) still left 19 of the 24
simulated wrong starts of this pair: mostly the truth was never a candidate at all.

All 15 pairs, one scan per second (1,175 scans; `quickstart_model.decide_startup`
replayed from every scan as a start, ground truth as odometry, 995 start points):

| G2 / startup | top right | truth a candidate | query median | wrong | right | manual |
|---|---|---|---|---|---|---|
| quickstart (`nms_radius_m` 3 m) | 64% | 67% | 1.16 s | 52 | 835 | 108 |
| `nms_radius_m` 2 m | 73% | 78% | 0.58 s | 11 | 847 | 137 |
| `nms_radius_m` 1.5 m | 72% | 78% | 0.46 s | **5** | 804 | 186 |
| 1.5 m + `global_consensus_near_alias_samples` 3 | | | | **5** | **833** | 157 |
| 1.5 m + `global_consensus_near_alias_samples` 4 | | | | 5 | 814 | 176 |
| 3 m + `global_consensus_near_alias_samples` 3 | | | | 52 | 835 | 108 |

2 m left 3 wrong starts on EIL_Box → EIL_Mask1, which had none, and 8 on EIL_Box →
EIL_Stairs. With 1.5 m the five left are on the two pairs that had the most (EIL_Box →
EIL_Stairs 2, was 20; Long_Stairs → EIL_Stairs 3, was 16); EIL_Stairs → EIL_Box went
from 9 / 73 / 10 to 0 / 90 / 2. The cost is manual poses where the room repeats: in the
capture room the place about 1.8 m along x registers within 2x of the right one in
many views, and with 1.5 m that alternative is a candidate, so the nearby-alternative
check of #341 rejects the right answer that 3 m let through (Long_Stairs → EIL_Stairs:
right 64 → 22). The near-alias rule accepts such a fix once three views agree through
odometry, if the alternative has the same heading (the room turned around, or an
alternative 5 m away, still rejects): Long_Stairs → EIL_Stairs right 22 → 33, no new
wrong start anywhere. With 3 m it never applies. The opt-in 3D search
(`global_search_engine: bbs3d`) also answers this pair: top right for 79% of the
scans (3 m: 55%, 1.5 m: 86%), replay 0 wrong / 92 right / 0 manual, 1.8 s per query;
it was not run on the other pairs or live here.

Live, the 14 cases of #342 (two runs each; before = #345's runs on `main`, quickstart's
3 m; after = `--global-nms-radius-m=1.5 --global-consensus-near-alias-samples=3` added
to the same suite, `--seed-correction-window-sec=2.0` in both):

| map → scans | before: initialized / wrong / manual | after | after: first-pose error (m): median / p95 / max xy (m) |
|---|---|---|---|
| Long_Stairs → Outdoor2 (0-165 s) | 2 / 0 / 0 | 2 / 0 / 0 | 0.01: 0.02 / 0.04 / 0.07; 0.02: 0.02 / 0.04 / 0.07 |
| EIL_Mix → EIL_Box | 1 / 0 / 1 | 2 / 0 / 0 | 0.01: 0.03 / 0.07 / 0.32; 0.01: 0.03 / 0.07 / 0.28 |
| EIL_Mix → EIL_Stairs | 2 / 0 / 0 | 2 / 0 / 0 | 0.02: 0.02 / 0.12 / 0.39; 0.03: 0.02 / 0.15 / 0.48 |
| EIL_Mix → EIL_Mask1 | 2 / 0 / 0 | 2 / 0 / 0 | 0.04: 0.02 / 0.07 / 0.17; 0.04: 0.02 / 0.07 / 0.16 |
| EIL_Mix → EIL_Mask2 | 2 / 0 / 0 | 2 / 0 / 0 | 0.03: 0.02 / 0.04 / 0.07; 0.06: 0.02 / 0.07 / 0.48 |
| EIL_Box → EIL_Mix | 1 / 0 / 1 | 2 / 0 / 0 | 0.03: 0.03 / 0.08 / 0.38; 0.02: 0.03 / 0.10 / 0.38 |
| EIL_Box → EIL_Stairs | 2 / 0 / 0 | 1 / 0 / **1** | 0.02: 0.02 / 0.12 / 0.38; manual |
| EIL_Box → EIL_Mask1 | 1 / 0 / 1 | 2 / 0 / 0 | 0.09: 0.03 / 0.12 / 0.30; 0.04: 0.03 / 0.09 / 0.16 |
| Long_Stairs → EIL_Box | 2 / 0 / 0 | 2 / 0 / 0 | 0.13: 0.08 / 0.21 / 0.44; 0.06: 0.08 / 0.21 / 0.37 |
| Long_Stairs → EIL_Stairs | 2 / 0 / 0 | 1 / 0 / 1 | manual (no odometry); 0.05: 0.05 / 0.26 / 0.60 |
| Long_Stairs → EIL_Mask1 | 2 / 0 / 0 | 1 / 0 / **1** | 0.17: 0.10 / 0.27 / 0.59; manual |
| *controls:* EIL_Box → EIL_Box | 2 / 0 / 0 | 2 / 0 / 0 | 0.02: 0.02 / 0.05 / 0.52; 0.02: 0.02 / 0.04 / 0.19 |
| EIL_Stairs → EIL_Stairs | 2 / 0 / 0 | 2 / 0 / 0 | 0.02: 0.02 / 0.06 / 0.48; 0.02: 0.02 / 0.10 / 0.97 |
| EIL_Mix → EIL_Mix | 2 / 0 / 0 | 2 / 0 / 0 | 0.04: 0.02 / 0.06 / 0.27; 0.03: 0.02 / 0.06 / 0.27 |
| **total** | 25 / 0 / 3 | 25 / 0 / 3 | |
| EIL_Stairs → EIL_Box (new) | 8 runs: 8 / **4** / 0 | 8 runs: 7 / 0 / 1 | 0.01-0.02 m first-pose error, max 0.31-0.59 m |

The three manual poses before all came from runs that started without odometry: the
benchmark starts quickstart 3 s after the bag, and in some runs its discovery did not
see RKO-LIO yet (it logs `Discovery: odometry=none`), so the localizer ran without
odometry and the search on the raw, upside-down scan. After, one run did (Long_Stairs →
EIL_Stairs run 1); the other two asked for a pose in views that stayed ambiguous:
EIL_Box → EIL_Stairs run 2 saw the room turned around in two views and the place 1.2 m
along it next to the right answer in two more, and Long_Stairs → EIL_Mask1 run 2 (part
of the sensor covered) placed four of six views 5-22 m away. The first right pose came
after a median 9.3 s (11.6 s before, max 13.3 s against 20.1 s), and tracking in the
right runs stayed within p95 0.27 m (0.47 m) and max 0.97 m (1.40 m). EIL_Stairs → EIL_Box
after combines six runs with 1.5 m alone (five right, one without odometry asked for a
pose) and two with both settings. With 1.5 m alone on the 14 cases (28 runs): 21 / 0 / 7,
five of the seven without odometry and both Long_Stairs → EIL_Stairs runs ambiguous in all
six views, which the near-alias rule is for; with it, 3 of the 4 runs of that pair
started right (the fourth had no odometry).

Not verified: one site (the JEPLO capture room and its corridor); the rule's 3 and the
1.5 m were picked from these replays (1.5, 2 and 3 m on all pairs, 1 m only on the first
60 s of EIL_Stairs → EIL_Box: 0 wrong, 100 right, 8 manual of 108); two runs per pair. Not
run: the other suites (the Large maps settings on self maps, Koide, Boreas, HDL), which
keep quickstart's 3 m. The tracking failure of run 4 (a right start pulled 1.9 m onto
the turned room in the first two scans) is addressed in the next section; the odometry
discovery race is fixed since (see [Running it](#running-it)). The tables before this section
used 3 m: add `--global-nms-radius-m=3.0 --global-consensus-near-alias-samples=0` to
the suite to reproduce them.

#### EIL_Stairs → EIL_Box: a right start pulled onto the turned room, and the fix

Measured 2026-11-23 on `main` at `6f6cf9a` (v1.12.0). In live run 4 above (without
the release agreement) G2 published a pose 0.05 m from the ground truth, and the
first two scans moved it by 0.60 m (fitness 0.13) and 1.88 m (0.42); the track
ended on the room turned around, 4.6 m off. Replayed deterministically (bag from the
start, RKO-LIO, the localizer alone, and query 4's answer published on `/initialpose`
at the same bag time, moved by odometry as the startup does; quickstart's settings with
the window guard and the release agreement):

| start at 230.52 s (bag time) | runs | ended 4.6 m off | max xy of the others (m) |
|---|---|---|---|
| G2's answer, as published (`main`) | 6 | **2** | 1.16 (all four) |
| the ground-truth pose (`main`) | 1 | 0 | 1.17 |
| G2's answer, trusted (`seed_correction_guard_trusted_start_position_std_m`) | 4 | **0** | 0.26, 0.31, 0.32, 0.35 |
| the ground truth + 0.5 m, `main` / trusted | 1 / 2 | 1 / 0 | - / 0.58, 0.61 |
| the ground truth + 1.5 m, std 0.5 m (a manual pose): `main` / with the setting | 1 / 1 | 1 / 1 | - |

Why: the fix arrived as the Go2 stepped down: in 0.5 s the ground truth pitched from
10 to 45 degrees and dropped 0.5 m, and was level again 0.7 s later. The scans of
that second register badly against the map (corrections of 0.4-2.1 m, some with a fitness of 0.02), while
RKO-LIO carried the right pose through it within 0.2 m of the ground truth. Started
from the ground truth itself the localizer was pulled 1.2 m too, so it is neither the
G2 pose nor a stale seed: the published pose's pitch was the query scan's (0 degrees
against 15), but taking roll and pitch from the odometry, or the ground truth, gave
similar first corrections. The guards did not help because of their warmup: they
hold only after 5 accepts within 0.3 m, and the first correction (0.5 m) already
restarted the count, so every jump of that second was accepted; whether the track
returned or locked onto the turned room depended on which scans were processed. A
fitness gate cannot tell them apart either: the wrong corrections came with fitness
0.02-0.45, and in the earlier live runs a right first correction of 0.76 m had a
fitness of 0.12 (99 starts, the first six scans after each: 16 corrected the pose by
more than 0.3 m at least once, by 0.3-0.8 m from fixes 0.26-0.59 m off in the right
cases).

The fix trusts the fix the startup has already confirmed: with
`--trusted-global-start-std-m 0.1` (only with odometry prediction) the startup
publishes an NDT-refined global fix that odometry moved from its scan to now with a
0.1 m position standard deviation (`global_pose_position_std_m`), and the localizer
(`seed_correction_guard_trusted_start_position_std_m: 0.1`) treats an `/initialpose`
claiming at most that as trusted: when the first scan after it is seeded from
odometry, the guards hold from that scan. Anything else, including RViz's 2D Pose
Estimate (0.5 m), an unrefined fix, a fix odometry could not move, or a saved pose
(the localizer's covariance floor is 0.2 m), keeps the warmup. The cost is a fix that
is off: it stays off until the guards' release, 30 rejected scans (3 s) later, and is
then corrected toward what the rejected scans agreed on.

Live, the cross-session suite with #346's settings and `--seed-correction-window-sec=2.0`
as in the table above, two runs per case (before = that table's "after" column; after
= the same plus `--trusted-global-start-std-m=0.1`; median / p95 / max xy in m per
run, "-" asked for a manual pose):

| map → scans | before: initialized / wrong / manual | after | after: first-pose error: median / p95 / max |
|---|---|---|---|
| Long_Stairs → Outdoor2 (0-165 s) | 2 / 0 / 0 | 2 / 0 / 0 | 0.02: 0.02 / 0.04 / 0.07; 0.02: 0.02 / 0.04 / 0.07 |
| EIL_Mix → EIL_Box | 2 / 0 / 0 | 2 / 0 / 0 | 0.02: 0.03 / 0.11 / 0.29; 0.01: 0.03 / 0.09 / 0.35 |
| EIL_Mix → EIL_Stairs | 2 / 0 / 0 | 2 / 0 / 0 | 0.03: 0.02 / 0.12 / 0.53; 0.02: 0.02 / 0.11 / 0.44 |
| EIL_Mix → EIL_Mask1 | 2 / 0 / 0 | 2 / 0 / 0 | 0.03: 0.02 / 0.07 / 0.20; 0.03: 0.02 / 0.07 / 0.17 |
| EIL_Mix → EIL_Mask2 | 2 / 0 / 0 | 2 / 0 / 0 | **0.67**: 0.02 / **0.73** / 0.80; 0.02: 0.02 / 0.05 / 0.07 |
| EIL_Box → EIL_Mix | 2 / 0 / 0 | 2 / 0 / 0 | 0.03: 0.03 / 0.11 / 0.36; 0.03: 0.03 / 0.08 / 0.37 |
| EIL_Box → EIL_Stairs | 1 / 0 / 1 | 2 / **1** / 0 | **2.23 (wrong)**; 0.01: 0.02 / 0.12 / 0.38 |
| EIL_Box → EIL_Mask1 | 2 / 0 / 0 | 2 / 0 / 0 | 0.08: 0.03 / 0.13 / 0.33; 0.03: 0.03 / 0.14 / 0.36 |
| Long_Stairs → EIL_Box | 2 / 0 / 0 | 2 / 0 / 0 | 0.09: 0.08 / 0.21 / 0.49; 0.03: 0.08 / 0.21 / 0.36 |
| Long_Stairs → EIL_Stairs | 1 / 0 / 1 | 1 / 0 / 1 | 0.04: 0.05 / 0.31 / 0.62; - |
| Long_Stairs → EIL_Mask1 | 1 / 0 / 1 | 2 / 0 / 0 | 0.09: 0.08 / 0.21 / 0.39; 0.10: 0.08 / 0.20 / 0.33 |
| *controls:* EIL_Box → EIL_Box | 2 / 0 / 0 | 2 / 0 / 0 | 0.02: 0.02 / 0.05 / 0.19; 0.02: 0.02 / 0.05 / 0.19 |
| EIL_Stairs → EIL_Stairs | 2 / 0 / 0 | 2 / 0 / 0 | 0.03: 0.02 / 0.06 / 0.32; 0.02: 0.02 / 0.06 / 0.35 |
| EIL_Mix → EIL_Mix | 2 / 0 / 0 | 2 / 0 / 0 | 0.05: 0.02 / 0.06 / 0.39; 0.03: 0.02 / 0.06 / 0.27 |
| **total** | 25 / 0 / 3 | 27 / 1 / 1 | |
| EIL_Stairs → EIL_Box | 8 runs: 7 / 0 / 1 | 4 runs: 4 / 0 / 0 | 0.02 m first-pose error, p95 0.13-0.26, max 0.37-0.68 |

The wrong start and the two manual poses are G2's, not the localizer's: the manual
poses came from runs in which no view was unambiguous, so no pose was published, and
the wrong one (EIL_Box → EIL_Stairs run 1) published the room turned around, 2.2 m
off, whose first five scans corrected it by 0.03-0.21 m: the guards were in the state
they reach after a warmup either way (in #346's runs this pair saw the room turned
around in two views of one run, which then asked for a manual pose). EIL_Mix →
EIL_Mask2 run 1 is the cost: G2's answer was a place 0.7 m along the room (fitness
0.028), the guards held it for 30 scans and then took the
correction the rejected scans agreed on; tracking was right from 3 s after the fix.
Before, the registration corrected such fixes on the first scans (0.3-0.8 m in the
99 starts above). In the runs that started right, the first pose came after a median
9.1 s (9.3 s before) and tracking stayed within p95 0.31 m and max 0.68 m apart from
that run (before: 0.27 m and 0.97 m). The live EIL_Stairs → EIL_Box runs did not hit
the failure on `main` either (4 more runs: max 0.35-0.49 m): with #346's settings G2
answers earlier, before the step down, so the case is in the replay above.

Two cases were run again after a change: in one run of EIL_Mix → EIL_Mix and one of
EIL_Mix → EIL_Mask2, odometry had not yet covered the queried scan when the fix was
published, so the startup could not move it to the present (it was 0.48 and 0.42 m
off), and the guards held it. The startup now publishes such a fix, and any
unrefined one, with its default 0.5 m, so it is not trusted; the table has the new
runs of both cases (one of their four fixes was published untrusted that way).

`go2_jeplo.yaml` could not run as is here (it needs the lidar_slam_ros2 map of EIL_Mix
and the Long_Stairs, Outdoor1 and Outdoor2 bags). Its four EIL cases ran on the
room-only EIL_Mix map (`maps_loc/EIL_Mix`) with quickstart's defaults (no window
guard), one run each with and without the option and two more of EIL_Box (median /
p95 / max xy in m):

| case | `main` | with `--trusted-global-start-std-m=0.1` |
|---|---|---|
| EIL_Box | 0.03 / 0.09 / 0.35; wrong start (1.58 m); 0.03 / 0.08 / 1.67 | 1.33 / 4.61 / 5.39 (slid at 42 s); 0.03 / 0.25 / 0.48; 0.03 / 0.10 / 0.36 |
| EIL_Stairs | 0.02 / 0.67 / 1.17 | 0.03 / 0.76 / 1.02 |
| EIL_Mask1 | 0.02 / 0.07 / 0.41 | - (no view unambiguous, no pose published) |
| EIL_Mask2 | 0.02 / 0.05 / 0.07 | 0.02 / 0.05 / 0.17 |

The EIL_Box slide is the default settings' slide along the room (see [Why tracking
slid along the room](#why-tracking-slid-along-the-room-and-the-fix)), 42 s after a
start whose first five scans corrected the fix by 0.03-0.16 m, after which the guards
are warm with or without the option; `main` had a wrong start and a 1.67 m excursion
in the same three runs.

Not verified: one site and one robot; one case where the option was needed, and that
one only in the replay (5 and 4 runs). 0.1 m is a value the localizer's own
covariance (floor 0.2 m) and RViz (0.5 m) do not reach, not a measured accuracy of G2:
in the 31 live starts one refined, odometry-moved fix was 0.67 m off (and one 0.43 m
off in a run that was repeated), and such a fix is held for 3 s. Not run: the Large
maps settings (`go2_jeplo_low_sensor.yaml`), Koide, Boreas and HDL, where the option
is off; their defaults do not change. The stale roll and pitch
of a published fix (the query scan's, moved by planar odometry only) are not changed.

### Defaults with odometry prediction

After v1.12.0, quickstart turns on three settings that were opt-in whenever odometry
prediction is on, under every profile
([quickstart](quickstart.md#defaults-with-odometry-prediction)):
`--seed-correction-window-sec 2.0` (it brings the release agreement of 10 scans),
`--global-nms-radius-m 1.5` (3.0 before) and `--global-consensus-near-alias-samples 3`
(0 before). Without odometry prediction all three keep the old values. This is the A/B
evaluation behind that choice. Every run had odometry prediction on and used the
default (standalone) profile, which is why the defaults follow odometry prediction and
not a profile.

Measured on v1.12.0, Jazzy, replay at 1x, RKO-LIO odometry with odometry prediction on,
two runs per case and configuration, with the suites' default (standalone) profile and
the settings passed as flags. **A** is the old defaults, **B** the three settings; **C** was B with
`--global-search-engine bbs3d`. Cross-session is
[`go2_jeplo_cross_session.yaml`](../tools/benchmark/suites/go2_jeplo_cross_session.yaml)
(the Large maps settings; A used 3.0 m and no near-alias rule), "Go2 EIL" the four EIL
cases of `go2_jeplo.yaml` on the room-only EIL_Mix map with quickstart's defaults, and
Koide `outdoor_hard_01a` and `02b` of `koide_outdoor.yaml`.

| suite | config | runs | initialized / wrong / manual pose | first pose, median (max) |
|---|---|---|---|---|
| cross-session | A | 32 | 31 / 3 / 1 | 10.8 s (19.8 s) |
| cross-session | **B** | 32 | 29 / 1 / 3 | 9.1 s (14.9 s) |
| cross-session | C (B + bbs3d) | 32 | 31 / 1 / 1 | 16.6 s (38.3 s) |
| Go2 EIL | A | 8 | 8 / 0 / 0 | 11.4 s (13.8 s) |
| Go2 EIL | **B** | 8 | 8 / 0 / 0 | 8.9 s (11.6 s) |
| Koide outdoor_hard | A | 4 | 4 / 0 / 0 | 12.9 s (13.3 s) |
| Koide outdoor_hard | **B** | 4 | 4 / 0 / 0 | 12.6 s (14.8 s) |

"Wrong" is a first pose more than 1 m off (counted among the initialized runs),
"manual pose" a run that used up its global attempts. Per case (run 1; run 2),
cross-session and the cases that changed elsewhere:

| case (map → scans) | A initialized / wrong / manual | B initialized / wrong / manual | A first pose (s) | B first pose (s) | A median / p95 / max xy (m) | B median / p95 / max xy (m) |
|---|---|---|---|---|---|---|
| EIL_Box → EIL_Box | 2 / 0 / 0 | 2 / 0 / 0 | 9.8; 9.6 | 8.8; 8.6 | 0.02/0.05/0.26; 0.02/0.05/0.47 | 0.02/0.04/0.49; 0.02/0.05/0.26 |
| EIL_Box → EIL_Mask1 | 2 / 0 / 0 | 2 / 0 / 0 | 8.0; 9.0 | 9.0; 10.3 | 0.03/0.08/0.81; 0.03/0.11/0.38 | 0.03/0.10/0.46; 0.03/0.10/0.19 |
| EIL_Box → EIL_Mix | 2 / **1** / 0 | 2 / **1** / 0 | 8.8; 10.8 | 8.8; 8.3 | 0.03/1.09/2.07; 0.03/0.14/0.48 | 0.03/0.08/0.39; 0.03/1.67/2.15 |
| EIL_Box → EIL_Stairs | 2 / **1** / 0 | 2 / 0 / 0 | 9.4; 11.9 | 12.1; 11.6 | 1.87/3.88/5.55; 0.02/0.20/1.16 | 0.02/0.15/0.48; 0.02/0.14/0.56 |
| EIL_Mix → EIL_Box | 2 / 0 / 0 | 2 / 0 / 0 | 11.6; 9.6 | 9.1; 9.1 | 0.03/0.08/0.31; 0.03/0.09/0.73 | 0.03/0.15/0.65; 0.03/0.12/1.44 |
| EIL_Mix → EIL_Mask1 | 2 / 0 / 0 | 2 / 0 / 0 | 9.0; 9.5 | 8.5; 9.5 | 0.02/0.07/0.17; 0.02/0.07/0.18 | 0.02/0.06/0.17; 0.02/0.06/0.17 |
| EIL_Mix → EIL_Mask2 | 2 / 0 / 0 | 2 / 0 / 0 | 7.8; 9.8 | 10.0; 8.8 | 0.02/0.05/0.49; 0.02/0.05/0.26 | 0.02/0.05/0.14; 0.02/0.05/0.48 |
| EIL_Mix → EIL_Mix | 2 / 0 / 0 | 2 / 0 / 0 | 9.8; 10.3 | 8.3; 8.8 | 0.02/0.05/0.29; 0.02/0.05/0.27 | 0.02/0.06/0.28; 0.02/0.05/0.29 |
| EIL_Mix → EIL_Stairs | 2 / 0 / 0 | 2 / 0 / 0 | 12.1; 11.9 | 10.4; 7.6 | 0.02/0.61/1.02; 0.02/0.76/1.14 | 0.02/0.12/0.66; 0.02/0.12/0.91 |
| EIL_Stairs → EIL_Box | 2 / **1** / 0 | 2 / 0 / 0 | 12.1; 14.6 | 8.8; 8.6 | 0.06/1.74/2.20; 0.04/0.30/0.39 | 0.04/0.12/0.38; 0.04/0.14/0.39 |
| EIL_Stairs → EIL_Stairs | 2 / 0 / 0 | 2 / 0 / 0 | 8.4; 9.9 | 7.4; 6.9 | 0.02/0.06/0.32; 0.02/0.06/0.49 | 0.02/0.06/0.49; 0.02/0.06/0.53 |
| Long_Stairs → EIL_Box | 2 / 0 / 0 | 2 / 0 / 0 | 19.8; 18.3 | 9.6; 10.8 | 0.26/1.66/2.02; 0.10/1.35/1.72 | 0.08/0.21/0.42; 0.08/0.22/0.36 |
| Long_Stairs → EIL_Mask1 | 1 / 0 / **1** | 0 / 0 / **2** | 15.3; - | -; - | 0.36/0.64/0.76 | - |
| Long_Stairs → EIL_Stairs | 2 / 0 / 0 | 1 / 0 / **1** | 12.4; 15.6 | 14.9; - | 0.23/1.79/2.24; 0.25/1.83/2.24 | 0.05/0.27/0.60 |
| Long_Stairs → Outdoor2 (0-165 s) | 2 / 0 / 0 | 2 / 0 / 0 | 15.3; 12.8 | 12.3; 11.3 | 0.02/0.04/0.16; 0.02/0.04/0.07 | 0.02/0.04/0.07; 0.02/0.04/0.07 |
| Outdoor2 (0-165 s), own map | 2 / 0 / 0 | 2 / 0 / 0 | 13.3; 13.1 | 10.8; 11.3 | 0.02/0.04/0.06; 0.02/0.04/0.06 | 0.02/0.04/0.06; 0.02/0.04/0.08 |
| *Go2 EIL:* EIL_Stairs | 2 / 0 / 0 | 2 / 0 / 0 | 12.4; 12.9 | 11.6; 8.9 | 0.02/0.78/1.17; 0.02/0.67/1.14 | 0.02/0.12/0.42; 0.02/0.12/0.53 |
| *Koide:* outdoor_hard_01a | 2 / 0 / 0 | 2 / 0 / 0 | 12.5; 13.3 | 12.0; 13.0 | 5.10/5.61/7.51; 4.43/4.66/5.17 | 0.11/0.92/1.61; 0.61/1.16/1.86 |
| *Koide:* outdoor_hard_02b | 2 / 0 / 0 | 2 / 0 / 0 | 12.8; 13.0 | 14.8; 12.3 | 0.09/0.45/1.75; 0.09/0.43/1.45 | 0.08/0.32/1.45; 0.09/0.56/2.57 |

What changed:

- **Wrong starts: 3 to 1.** A's three were G2 answers 1.6-1.9 m along the room
  (EIL_Box → EIL_Mix, EIL_Box → EIL_Stairs, EIL_Stairs → EIL_Box), which the 1.5 m
  suppression and the near-alias rule fix. B had no wrong G2 answer: its one wrong
  start (EIL_Box → EIL_Mix run 2, 2.0 m) was a right fix pulled away by the first scans
  after it, the case `--trusted-global-start-std-m` addresses
  ([why](#eil_stairs--eil_box-a-right-start-pulled-onto-the-turned-room-and-the-fix)).
- **Slides along the room** stopped: p95 0.6-1.8 m under A (EIL_Mix → EIL_Stairs,
  Long_Stairs → EIL_Box and EIL_Stairs, the Go2 EIL_Stairs case) against 0.12-0.27 m
  under B. On Koide 01a, A lost the pose at about 113 s in both runs (median 4.4-5.1 m
  from then on); B did not.
- **First pose sooner**: median 9.1 s against 10.8 s cross-session, 8.9 s against
  11.4 s on the Go2 EIL cases.
- **Costs.** On the Long_Stairs map, which lacks the EIL props, B asked for a manual
  pose 3 times in 6 runs (A once): with the 1.5 m suppression the startup sees the
  places along the room next to the right answer, and in those runs no three views
  agreed. The release agreement can hold the odometry for seconds where a release
  would have been right: Koide 02b run 2 reached 2.57 m (max) against 1.45-1.75 m
  while releases were held (48 held in that run; 10-509 per Koide run, none on the Go2
  cases). The window guard rejected
  0-11.6 s of scans per run.
- **bbs3d (C)**: 31 / 1 / 1 cross-session (two manual poses fewer than B; its wrong
  start was 3.3 m off), but the first pose came after a median 16.6 s (max 38.3 s), and
  it asked for a manual pose in 1 of 4 Go2 EIL runs and 1 of 2 Koide runs (A and B:
  none). It stays opt-in.

The Go2 suites and `koide_outdoor.yaml` run with odometry prediction and the default
profile, so they now get these defaults; `go2_jeplo_cross_session.yaml` no longer passes
the 1.5 m radius and near-alias flags (and gains the window guard). To compare with
results up to v1.12.0, add `--seed-correction-window-sec=0 --global-nms-radius-m=3
--global-consensus-near-alias-samples=0` to a suite's `quickstart_args` (for the
cross-session suite's earlier results, only `--seed-correction-window-sec=0`).

**Smoke check of the shipped suites** (this change, one run per case plus a rerun of two;
`go2_jeplo_cross_session.yaml` and `koide_outdoor.yaml` as shipped, standalone profile,
RKO-LIO): every run resolved the three settings from odometry prediction (`Settings:`
line and the head of `quickstart.yaml` say `default with odometry prediction`;
`enable_seed_correction_window_guard: true`, `seed_correction_window_guard_sec: 2.0`,
`seed_correction_guard_release_agreement_scans: 10`; launch `g2_nms_radius_m:=1.5`,
`global_consensus_near_alias_samples:=3`).

| case (map → scans) | initialized / wrong / manual | first pose (s) | median / p95 / max xy (m) |
|---|---|---|---|
| EIL_Stairs → EIL_Box | 1 / 0 / 0 | 6.8 | 0.04 / 0.15 / 0.35 |
| EIL_Box → EIL_Stairs | 1 / **1** / 0 | 7.9 | 0.02 / 1.24 / 2.49 |
| EIL_Box → EIL_Stairs, again | 1 / 0 / 0 | 11.1 | 0.02 / 0.14 / 0.40 |
| Long_Stairs → EIL_Box | 0 / 0 / **1** | - | - |
| Long_Stairs → EIL_Box, again | 1 / 0 / 0 | 13.8 | 0.08 / 0.21 / 0.42 |
| Koide outdoor_hard_02b | 1 / 0 / 0 | 13.8 | 0.09 / 0.55 / 1.89 |

The wrong run started 1.8 m off after two ambiguous retries, with an RKO-LIO
registration loop of 17 s in that run (a cold disk after a restart; EIL_Stairs → EIL_Box
had one of 12 s); its rerun without a stall started right. The manual pose on the
Long_Stairs map is the cost the A/B shows there (every view ambiguous, 6 attempts). Two
earlier Koide attempts were invalid (no odom TF within 30 s while the 2.5 GB bag was
read from a cold disk) and are not counted.

What this does not cover:

- **Sensors and odometry**: only Livox MID-360 sensors (a Go2 with the sensor upside
  down and a handheld rig) and RKO-LIO odometry were measured. The defaults apply to any
  setup with odometry prediction (other LiDARs, wheel or leg odometry, other LIO front
  ends), and those were not measured; odometry that drifts more than RKO-LIO did here
  may make the window guard reject more scans.
- **Profiles**: the A/B runs used the standalone profile. `--profile nav2` and
  `mid360` change other localizer settings (`mid360`: IMU preintegration,
  continuous-time deskew, its scan filter and ranges), and with odometry they get the
  same three defaults without an A/B of their own.
- **Without odometry prediction** nothing changes; the 1.5 m radius alone, without the
  near-alias rule and window guard, was not evaluated, which is why it is not a
  default there.
- Two runs per case and configuration, one machine, one site for the cross-session
  pairs.

## Running it

Build lidar_slam_ros2 (for `rko_lio_odometry.launch.py`) and this package, then:

```bash
export JEPLO_ROOT=/data/jeplo                       # Unitree Go2, upside-down MID-360
export KOIDE_ROOT=/data/koide_hard_localization     # handheld MID-360 outdoors
python3 tools/benchmark/run_benchmark.py tools/benchmark/suites/go2_jeplo.yaml --out results/go2
python3 tools/benchmark/run_benchmark.py tools/benchmark/suites/koide_outdoor.yaml --out results/koide
```

The suites run quickstart with the default (standalone) profile and record
`/pcl_pose`; a suite that runs `--profile nav2` or `mid360` records
`/localization/pose_with_covariance`, where those profiles publish the pose (a suite or
case can set `pose_topic:`).
Each case runs three times by default (`--repeats`, `--case` to select). The run
directories keep every log, the recorded trajectory (`est.tum`), the startup status
stream, and `score.json`. `summary.md` and `results.json` collect the cases.
`--evaluate-only` re-scores existing runs, and `--quickstart` runs a source checkout
instead of the installed package. Runs whose RKO-LIO odometry stalled (a registration
loop over 10 s in its exit profile, e.g. on a cold disk after a restart) are listed
under the tables: the startup then lacks odometry, so re-run them before comparing.

When a suite launches odometry, each run waits up to 30 s (`odometry_tf_timeout_sec`)
for its odom TF on /tf before starting quickstart (still 3 s after the bag at the
earliest), and passes `--odom-tf-prediction --base-frame <child of odom>` unless the
case's `quickstart_args` set them, so the run does not depend on quickstart's own
discovery. A run whose quickstart configuration still has `use_odom_tf_prediction: false`,
or whose odometry never published TF, is stopped and marked invalid (`invalid.txt` in
the run directory): it is not scored, `summary.md` lists it, and the command exits
with 1. Set `expect_odometry: false` in a suite or case to measure a launched odometry
without requiring it. Results before this check (up to #346) include runs that started
without odometry in about 1 in 10 runs; the tables above name them.

Datasets:

- JEPLO (Unitree Go2): <https://huggingface.co/datasets/ASIG-Lab/JEPLO-Dataset>
  (formerly `ASIG-X/JEPLO`, which returned 404 on 2026-10-11). The suite uses
  `bags_pc2_merged`, `gt` and, for Long_Stairs and Outdoor*, maps built from the same
  sequence (`maps_loc`). Ground truth covers the 10 m x 4 m capture area. The
  dataset now ships only the raw bags (`bags/`, Livox `CustomMsg` at 20 Hz) and `gt/`;
  rebuild the other two as below.
- Koide et al., hard localization: the outdoor MID-360 sequences with
  `map_outdoor_hard.ply` / `map_outdoor_kidnap.ply`. The kidnap sequences cover the
  sensor and carry it elsewhere.

### Rebuilding the JEPLO bags and self maps

`jeplo_to_pc2_bag.py` writes `PointCloud2` bags at 10 Hz, two sweeps per cloud,
stamped mid-sweep, with absolute per-point times (which `rko_lio_jeplo.yaml`
expects). The self maps accumulate the bag along RKO-LIO's own trajectory and move
it into the ground-truth frame. RKO-LIO's error on returning to the capture room
after 250-700 s is 2-4 cm on these sequences.

```bash
python3 tools/benchmark/jeplo_to_pc2_bag.py $RAW/bags/Long_Stairs $JEPLO_ROOT/bags_pc2_merged/Long_Stairs
# rko_lio_jeplo.yaml plus lidar_topic, imu_topic, base_frame: livox_frame,
# bag_path, dump_results: true, results_dir, under /**: ros__parameters
ros2 run rko_lio offline_node --ros-args --params-file long_stairs_rko.yaml
python3 tools/benchmark/build_map_from_odometry.py --bag $JEPLO_ROOT/bags_pc2_merged/Long_Stairs \
  --trajectory <results_dir>/Long_Stairs_0/Long_Stairs_tum_0.txt --out /tmp/Long_Stairs_odom.pcd
python3 tools/benchmark/align_map_to_ground_truth.py --map /tmp/Long_Stairs_odom.pcd \
  --trajectory <results_dir>/Long_Stairs_0/Long_Stairs_tum_0.txt \
  --ground-truth $JEPLO_ROOT/gt/Long_Stairs.txt --out $JEPLO_ROOT/maps_loc/Long_Stairs/map.pcd
```

These maps are not the ones the 2026-10-06 results above used.

### The Go2 map

The EIL_* cases localize the way a user would: on a lidar_slam_ros2 map of another
run, EIL_Mix, moved into the ground-truth frame by fitting its trajectory to the
ground truth (residual 0.09 m, the SLAM trajectory's own error):

```bash
bash <lidar_slam_ros2>/scripts/run_rko_lio_graph_autoware_dogfood.sh \
  --bag $JEPLO_ROOT/bags_pc2_merged/EIL_Mix --lidar-topic /livox/lidar --imu-topic /livox/imu \
  --lidarslam-param <lidar_slam_ros2>/lidarslam/param/lidarslam_mid360_rko_graph.yaml \
  --rko-param tools/benchmark/suites/rko_lio_jeplo.yaml \
  --output-dir maps/eil_mix --wait-for-offline-completion --skip-viewer --base-frame base_link
python3 tools/benchmark/align_map_to_ground_truth.py --map maps/eil_mix/map.pcd \
  --trajectory maps/eil_mix/traj_raw.tum --ground-truth $JEPLO_ROOT/gt/EIL_Mix.txt \
  --out maps/eil_mix_gt/map.pcd
export JEPLO_LIDARSLAM_MAP=$PWD/maps/eil_mix_gt/map.pcd
```

The dataset's own leave-one-out maps (`maps_loo_*`) are not used: their floor is
smeared over 0.7 m (-0.4 to +0.3 m), so the occupancy grid generated for global
search marks almost the whole room as occupied, and initialization fails or lands on
the wrong pose. A map from lidar_slam_ros2 of the same room has its floor within 0.1 m.

## Autoware Istanbul (vehicle, preset replay)

A second, narrower benchmark for a car at up to 21 m/s on grades: the localization bag
of the [Autoware Istanbul urban
evaluation](https://docs.autoware.org/main/simulation-evaluation/components_evaluation/localization_evaluation/urban-environment-evaluation/)
replayed through `param/istanbul_gtsam_smoother.yaml`, with the bag's twist and no
odometry. Unlike the runs above it does not go through the quickstart and does not
initialize globally: each run starts from the GNSS pose at the window start and is
scored against `/sensing/gnss/pose_with_covariance`, interpolated at each pose stamp.

```bash
gdown 1WPWmFCjV7eQee4kyBpmGNlX7awerCPxc -O "$ISTANBUL/map/"
gdown 1yEB5j74gPLLbkkf87cuCxUgHXTkgSZbn -O "$ISTANBUL/localization_bag/"
ros2 bag reindex "$ISTANBUL/localization_bag" -s sqlite3
python3 tools/benchmark/istanbul_prepare.py "$ISTANBUL/localization_bag" /tmp/istanbul_fixture
# one window through one workspace
python3 tools/benchmark/istanbul_replay.py --ws WS --fixture /tmp/istanbul_fixture \
  --bag "$ISTANBUL/localization_bag" --map "$ISTANBUL/map/pointcloud_map.pcd" \
  --start 2300 --duration 120 --out /tmp/istanbul_run
# a baseline and a candidate workspace, three windows, three runs each
python3 tools/benchmark/istanbul_compare.py --base-ws BASE_WS --cand-ws CAND_WS \
  --fixture /tmp/istanbul_fixture --bag "$ISTANBUL/localization_bag" \
  --map "$ISTANBUL/map/pointcloud_map.pcd" --out /tmp/istanbul_compare
```

`--set KEY=VALUE` overrides a preset parameter (`--cand-set` for the candidate only);
the preset is read from the checkout that holds the tools. Set `ROS2_CLI` and
`ROS_PYTHON` when `python3` is not the interpreter the ROS distro was built for.

| Window | Bag time | Why |
| --- | --- | --- |
| A | 0-60 s | parked for 15 s, then up to 10 m/s |
| B | 2820-2970 s | sustained 6-10 % grades |
| C | 2300-2420 s | the fastest climb, 4-5 % at up to 21 m/s |

Median horizontal RMSE of three runs on 2026-11-01 (4 vCPUs): 0.23, 0.75 and 0.46 m, at
8.9-10 poses/s (the scans arrive at 10 Hz; median alignment 11-12 ms); how they were
reached is in
[experiments/istanbul_ndt_tracking](../experiments/istanbul_ndt_tracking/README.md).
Results before then (0.25, 0.87 and 0.50 m at 1.6-3.6 poses/s on 2026-10-28) ran an
unoptimized NDT_OMP
([Unoptimized ndt_omp](../experiments/istanbul_ndt_tracking/README.md#unoptimized-ndt_omp)).
`score.json` and the `istanbul_compare.py` summary also report the alignment cost: the
share of the window's scans that were aligned, the alignment time and the registration
iterations.

`param/istanbul_small_gicp.yaml` is the same preset with `registration_method:
SMALL_GICP` (needs small_gicp, e.g. `ros-jazzy-small-gicp-vendor`; pass `--preset
istanbul_small_gicp.yaml`). On 2026-10-28 it gave 0.26, 0.76 and 0.45 m at 8.0-9.5
poses/s, about as the optimized NDT_OMP preset does now; the NDT_OMP preset it was
compared with then (0.26, 0.91 and 0.54 m at 1.3-3.1 poses/s) ran unoptimized. On the
online Koide `outdoor_hard_02b` check, SMALL_GICP failed all five runs (88-93%
coverage, 0.32-0.41 m RMSE) where NDT_OMP passed all four. It slows down off the map
and misses clouds there, so NDT_OMP stays the default. Most of its RMSE gap comes from
the off-map stretch the odometry carries, which says nothing about the backend; outside
it the gap is 0.192 against 0.177 m
([experiments/koide_backend_comparison](../experiments/koide_backend_comparison/README.md)).

What to keep in mind with this bag (`istanbul_prepare.py` and `istanbul_replay.py`
handle the first three):

- **Header stamps are 2,371,531.4 s (27 days) before the bag's receive time.** The bag
  carries its own `/clock` in the header time base: it is played instead of `--clock`,
  and window starts are mapped through the clouds' stamps.
- **The first GNSS message is stamped 6.3 s after the first cloud**, and the first
  messages repeat a stale pose 1.9 m and 146° off the next ones; they are dropped.
- **GNSS drops out during 750-1030 s and 3020-3215 s** (sigma up to 92 m); windows there
  have neither a usable start pose nor a usable reference.
- **The GNSS orientation is about 2° off the direction of travel** (median -2.26° over
  the bag), so yaw RMSE against it carries that bias, and dead reckoning from the GNSS
  initial yaw drifts sideways.
- The localizer reads `/localization/util/downsample/pointcloud` (1500 points,
  `base_link`) and `/localization/twist_estimator/twist_with_covariance`, which matches
  the GNSS-derived speed and yaw rate within 0.05 m/s and 0.002 rad/s. Grades reach
  9.6 % where GNSS is accurate (z from -65 to +177 m over the route).

## Boreas (vehicle, preset replay)

A third preset replay, with LiDAR, IMU and ground truth: the
[Boreas dataset](https://www.boreas.utias.utoronto.ca/) (Burnett et al., 2023), read
from the public bucket `s3://boreas`. A car drives the same Toronto route on many
days. One sequence is the localization run, and the map is built from another day's
scans at their ground-truth poses. Like Istanbul it starts from the ground-truth pose
and does not go through the quickstart. The published `/pcl_pose` is matched to the
ground-truth lidar pose with the same stamp, without alignment.

```bash
# the first 120 s of boreas-2021-09-02-11-42, a map from boreas-2020-11-26-13-58
# (~11 min, 1.7 GB; scans are downloaded and voxel-downsampled on the way)
python3 tools/benchmark/boreas_prepare.py /tmp/boreas_fixture --duration 120
python3 tools/benchmark/boreas_replay.py /tmp/boreas_fixture /tmp/boreas_run --ws WS
```

`boreas_prepare.py` writes the bag, `map.ply` and `reference.tum`:

- **The bag** holds `/velodyne_points`, voxel-downsampled to 0.25 m, and `/imu/data` at
  200 Hz. The IMU comes from `applanix/imu_raw.csv`, which keeps gravity, rotated
  into the lidar frame.
- **The map** holds the other run's scans within 60 m of the window's path, every 2 m,
  at their ground-truth poses, thinned to 0.3 m.
- **The fixture has no twist.** The post-processed ground-truth velocity is not a
  runtime input.

`boreas_replay.py` runs `param/boreas_ndt_velodyne.yaml` with the fixture's map, the
first ground-truth pose, and `base_frame_id: velodyne`. `--set KEY=VALUE` overrides a
preset parameter. It writes `summary.json` with the RMSE, its horizontal and z parts,
p95 and max error, the RMSE per 20 s, and the share of scans aligned and accepted.

The window covers parking-lot maneuvers, including a U-turn of 208° in 10 s, then city
streets at up to 9.7 m/s. On 2026-11-02, with 4 vCPUs:

| Preset | Runs | RMSE m | Horizontal / z RMSE m | p95 / max error m | Scans accepted |
| --- | --- | --- | --- | --- | --- |
| `boreas_ndt_velodyne.yaml` | 3 | **0.132** (all three) | 0.126 / 0.041 | 0.24 / 0.41 | **100%** |
| the same before this change | 1 | 53.2 | 53.2 / 0.65 | 92.9 / 95.9 | 1% |

The RMSE per 20 s stays between 0.07 and 0.17 m. The three runs are identical, because
every scan is aligned.

Over the first 600 s (`--duration 600`: 4.3 km, 5786 scans, a map of 20.9 M points from
3189 scans; 11 GB, about an hour to prepare), on 2026-11-02:

| Setting | Runs | RMSE m | Horizontal / z RMSE m | p95 / max error m | Scans processed |
| --- | --- | --- | --- | --- | --- |
| `boreas_ndt_velodyne.yaml` | 2 | **0.105** (both) | 0.097 / 0.042 | 0.20 / 0.41 | 99.9%, 100% |
| with `use_imu_preintegration: true` | 2 | **0.098**, 0.099 | 0.089-0.090 / 0.042 | 0.18 / 0.39 | 98.8%, 99.3% |

- **The preset holds over the longer window.** The RMSE per 20 s stays between 0.05 and
  0.17 m. The first 120 s, with the parking-lot maneuvers, hold the worst bins
  (0.07-0.17 m); after them no bin exceeds 0.15 m, so the whole scores lower than its
  start.
- **The IMU preintegration helps here too.** It lowers the RMSE in 27 of the 30 bins,
  only horizontally, and seeds all but 6 of the scans.
- **The unprocessed scans were dropped by the cloud queue, under load.** These runs
  overlapped a download.
  - **Where:** the dropped scans fall in the dense stretches, such as 84-96 s and
    104-116 s, where 4600-4900 points remain after filtering against 2700 elsewhere.
  - **Without the IMU** the node keeps up there on 4 vCPUs: over the first 120 s its
    pose lags `/clock` by 0.12 s at p95 and 0.23 s at most.
  - **With it** the lag was 0.46-0.70 s at p95 and 0.75-0.92 s at most, close to the
    10-scan `cloud_queue_depth` (about 1 s). Four such runs lost no scan. Since the
    smoother's anchor carries its residual, an update takes about half the time.
    Three runs since then lag 0.25-0.59 s at p95 and 0.56-0.83 s at most, so from run
    to run the lag varies about as much as the fix changes it
    ([experiments/imu_smoother_solve](../experiments/imu_smoother_solve/README.md#the-anchor-never-let-it-converge)).
  - **`ndt_num_threads: 8` on 4 vCPUs is not a loss.** With 4 threads the alignment
    takes 1-2 ms longer. With two cores kept busy, 36-43 more scans drop (1107
    against 1143 without the IMU, 1048 against 1091 with it), since more threads keep
    a larger share of the CPU.
  - **The deskew is off.** With the preintegration on, the default `relative_motion`
    deskew took the IMU smoother's poses and ran on about 45 000 points a scan. That
    accounted for the 3-4 ms longer scan preparation. It changed nothing: with it off
    the preintegration still scores 0.127 m, so the gain comes from the IMU seed. The
    preset now sets `use_continuous_time_deskew: false`. Without the IMU the deskew
    never ran (`continuous_time_deskew_imu_preintegration_disabled`), and the preset's
    trajectory is byte-identical.
  - **With two cores kept busy** the lag reaches 1.3 s, and the queue drops the oldest
    scans: 74 of 1158 with the IMU, 15 without.
  - **NDT_OMP's fix takes most of that lag away**
    ([ndt_omp_ros2 #15](https://github.com/rsasaki0109/ndt_omp_ros2/pull/15)). Two pairs
    of runs with two cores kept busy were alternated back to back, before and after it,
    on 2026-11-03:

    | Two cores kept busy | Lag p95 | Lag max | `alignment_time_sec` mean | Scans dropped |
    | --- | --- | --- | --- | --- |
    | with the IMU, before #15 | 0.58-0.77 s | 0.90-1.16 s | 35.8-36.2 ms | 0, 0 |
    | with the IMU, with #15 | 0.12-0.16 s | 0.34-0.41 s | 24.1-25.9 ms | 0, 0 |
    | without the IMU, before #15 | 0.17-0.18 s | 0.40 s | 36.9 ms | 0, 7 |
    | without the IMU, with #15 | 0.11-0.12 s | 0.23 s | 26.9-28.5 ms | 0, 0 |

    The RMSE did not change (0.127 m with the IMU, 0.132-0.133 m without). In this
    session even the runs before #15 dropped few scans, so it is the lag that shows the
    difference. The IMU runs no longer approach the 10-scan queue. Without load, the
    IMU run's lag is 0.06 s at p95 with #15 and 0.08 s before it.
  - **Not the transport:** a second subscriber with the same QoS received every cloud.
  - **The preset run that lost 6 scans** lost them the same way; its RMSE is the same.

What to keep in mind with this dataset (`boreas_prepare.py` handles the first three):

- **The ground-truth rotation is `(Rz(heading) Ry(pitch) Rx(roll))^T`.** With the active
  `Rz Ry Rx` of the removed converter, two scans 3 s apart overlap at 0.06 instead of
  0.54, and a map built from them is wrong.
- **The point `time` field is seconds from the scan stamp** (−0.052 to +0.052 s), which
  is mid-sweep, not microseconds.
- **The bag plays each cloud at its stamp, mid-sweep.** A driver publishes it after the
  sweep, about 52 ms later. So at a scan's callback the IMU covers only half its sweep:
  the gyro `imu_pose_history` deskew reports `pose_history_ends_too_early` on every
  scan and never runs.
- **Timings drift between sessions.** On the same 4-vCPU container the preset, with a
  byte-identical trajectory, aligned in 31.1 ms on average in one session and 20.1 ms in
  another. Compare timings only between runs made back to back.
- **NDT_OMP aligns faster since
  [ndt_omp_ros2 #15](https://github.com/rsasaki0109/ndt_omp_ros2/pull/15).** Run back
  to back, the preset aligned in 12.2-12.4 ms on average with it and 18.1-19.1 ms
  without, with a byte-identical trajectory. The runs above predate it
  ([experiments/ndt_alignment_cost](../experiments/ndt_alignment_cost/README.md#inside-align-false-sharing)).
- **`imu.csv` is post-processed and has gravity removed.** `imu_raw.csv` keeps it, in the
  Applanix frame turned by 180° about (1, −1, 0)/√2. In the lidar frame its yaw rate
  follows the ground-truth heading rate with correlation 1.0. Parked, it reads
  (0.094, 0.323, 9.794) m/s² against (0.092, 0.327, 9.801) from the ground-truth
  attitude.
- **The ~1 MB clouds need a reliable subscription.** With the best-effort sensor QoS,
  38 of 97 scans arrived (`qos_overrides` in the preset).
- **Gaps used to cost the most.** The seed advanced the last pose by one scan's
  motion. After a 0.6 s gap at 9.5 m/s it was 5 m behind, and NDT locked 4 m back
  along the road. It is now scaled to the time since the last pose
  (`previous_delta_max_extrapolation_sec`). With 0.6 s dropped every 10 s the window
  scores 0.132 m, as without gaps, against 0.8-6.3 m before
  ([experiments/previous_delta_time_scaling](../experiments/previous_delta_time_scaling/README.md)).
  The preset still reuses each 100 m target for 10 m and builds the next one in the
  background, with a 60 m scan range, so a scan does not wait for one.
- **The IMU preintegration helps a little; the legacy `use_imu` deskew broke the
  run.** With `--set use_imu_preintegration=true` the window scores 0.127 m (twice),
  with every scan processed; before the smoother's solve went sparse it skipped one
  scan in five. With
  `use_imu: true` it scored 1.7 m, with 6% of the scans accepted: that deskew
  integrated gravity, and was removed in 1.8.0. With both, the IMU callback used to wait behind every
  registration; it now has its own callback group
  ([imu_estimation.md](imu_estimation.md#validation-state-honest-limits)).

### Global initialization on a 1 km square map

This ROS 2 Jazzy run used the [Boreas dataset](https://www.boreas.utias.utoronto.ca/).
The map used 521 scans from `boreas-2020-11-26-13-58`, selected at 8 m spacing
along the route. Returns between 2 and 75 m were thinned to 0.35 m voxels and
clipped to the local ENU bounds x = −1350 to −350 m, y = 900 to 1900 m. The
binary PLY held 5,499,200 points along the surveyed roads. With the Boreas
preset, the node reached `active` 2.1 s after process start, with a peak RSS of
406 MiB. The 0.2 m occupancy grid was 5000 × 5000 cells.

Query scans came from the separate `boreas-2021-09-02-11-42` run, at 344.402,
368.979, 387.439, 415.854, 445.514, 470.506, and 491.351 s from its first LiDAR
scan. Returns within 80 m were thinned to 0.25 m voxels and levelled using
ground-truth roll and pitch. The local search used a 140 m square occupancy
window centred 10 m east, west, north, or south of the true position, clipped at the map
boundary. BBS used up to 512 points from −0.5 to 4 m in the levelled scan and
searched all headings at 5° steps. NDT refined and ranked 16 candidates with a
60 m scan range, and seeded their height at 1.66 m above the estimated ground.
The direct NDT comparison used the true height and heading with the same four
XY offsets.

Success here means the single-scan result had position error below 2 m and yaw
error below 0.05 rad (2.9°). The BBS rows use the highest-ranked candidate.

| Method | Success | Position error | Median query time |
| --- | ---: | --- | ---: |
| Direct NDT from a 10 m offset | 0/28 | 9.45 m median over all trials | |
| BBS near the 10 m offset, then NDT | 24/28 | 0.11 m median among successes | 8.3 s |
| Full-map BBS with no prior, then NDT | 6/7 | 0.11 m median among successes | 59.4 s |

All four local-search misses came from the same scan at 445.514 s. It was
about 41 m from the map boundary, and about 11% of the points within the 60 m
scoring range lay outside the map. BBS ranked a pose within 0.6 m horizontally
of the truth first for all four offsets. NDT refined that pose to within 3 cm, but its
fitness was about 13.2, so other poses ranked above it. The full-map query
ranked a pose about 810 m away first. At the true pose, removing the points
outside the map reduced the NDT fitness from 13.19 to 0.13. Rerunning the east
offset with the scan capped at 35 m put the correct candidate first with
0.12 m error.

In a separate Quickstart check, the east-offset window at 415.854 s initialized
from two distinct real scans with 0.08 m error. At the boundary site, Quickstart
rejected the candidates and requested manual initialization after six
queries. These checks used held scans and cover initialization. Continuous
tracking after initialization was outside this run.

### Soak and fault injection

`soak_replay.py` runs one localizer for an hour on the 120 s fixture, played lap after
lap at 1x. Each lap's stamps and `/clock` are shifted past the laps before it, and an
`/initialpose` at the ground-truth start pose begins every lap after the first. Every
5 s it samples the node's resident memory, CPU use and threads from `/proc`.
`--fault KIND@LAP:START[+DURATION]` injects a fault into one lap: `lidar` or `imu`
drops those messages, `stall` stops the node (SIGSTOP, then SIGCONT), `reinit` sends
`/initialpose` at the ground-truth pose there, while moving. `--wrap` runs the node
under a profiler such as heaptrack.

```bash
python3 tools/benchmark/soak_replay.py /tmp/boreas_fixture /tmp/boreas_soak --ws WS \
  --minutes 60 --set use_imu_preintegration=true
python3 tools/benchmark/soak_replay.py /tmp/boreas_fixture /tmp/boreas_faults --ws WS \
  --minutes 20.4 --set use_imu_preintegration=true \
  --fault lidar@1:50+1 --fault lidar@2:50+5 --fault stall@5:50+2 --fault reinit@7:60
```

`summary.json` holds each lap's RMSE, max error, accepted scans, alignment time and pose
latency, the memory trend, and per fault the largest error from its start to 10 s after its end, the
time from its end until the error stays under 0.5 m for the rest of the lap, and the
RMSE after that.

An hour with `use_imu_preintegration: true` on v1.8.0, with 4 vCPUs, on 2026-11-07:

- **Tracking does not wear down.** All 29 laps score 0.127 m RMSE, 0.36 m at most, and
  every scan is accepted. The alignment takes 7.8-9.2 ms at the median and 23-31 ms
  at p95 in every lap. CPU use averages 74% of one core, with 43 threads.
- **Resident memory climbs from 1.5 to 1.9 GB, 2.2 GB at most, but nothing leaks.**
  Under heaptrack the heap in use bottoms out at 403-426 MB in every half lap for
  12 minutes, and peaks at 1.1-1.3 GB. glibc keeps what is freed across its malloc
  arenas: with `MALLOC_ARENA_MAX=2` the same hour stays at 0.9-1.1 GB.

Since then the node keeps no older registration targets alive, sizes the local-map crop
to its points, and limits glibc to two malloc arenas. Five laps each, with
`use_imu_preintegration: true`, on 2026-11-07:

| Build | Resident memory, max | Lowest in each lap | Heap in use (heaptrack) | Alignment median / p95 |
| --- | --- | --- | --- | --- |
| v1.8.0 | 2.0 GB | 1.3-1.6 GB | 0.40-1.3 GB | 8.0-9.2 / 24-31 ms |
| now | **0.82 GB** | **0.62-0.66 GB** | 0.28-0.69 GB | 8.1-9.0 / 24-28 ms |

- **The heap peaked with the local-map crops.** Eight older targets were kept alive,
  each a crop of the dense map of up to 1.4 million points in a buffer grown to 2.5
  million (79 MB). Freed, they halve the heap's peak.
- **The resident memory follows the arenas.** The smaller heap alone left it at
  1.5-2.2 GB; two arenas take it down, for 2-6% more of one core (83% against
  85-89% in the same session).
- **`MALLOC_ARENA_MAX` in the environment still wins.** The limit is set in
  `lidar_localization_node`; a component loaded into another container needs
  `MALLOC_ARENA_MAX=2` in that container's environment.

Faults 50 s into a lap, at 5-8 m/s, on v1.8.0 with `use_imu_preintegration: true`:

| Fault | Poses published during it | Largest error | Back under 0.5 m after |
| --- | --- | --- | --- |
| none (laps 0 and 8) | | 0.36 m | |
| LiDAR gap, 1 s | 0 | 0.24 m | 0.02 s |
| LiDAR gap, 2 s | 0 | **8.6 m** | 1.6 s |
| LiDAR gap, 5 s | 0 | no pose for the rest of the lap | never |
| LiDAR gap, 30 s | 0 | no pose for the rest of the lap | never |
| IMU gap, 10 s | 96 | 0.29 m | 0.05 s |
| node stopped, 2 s | 19 | 1.5 m | 0.06 s |
| node stopped, 10 s | 19 | **54 m** | never |
| `/initialpose` while moving | | 0.28 m | 0.05 s |

- **After more than 1 s without an accepted scan the IMU seed is dropped.** Its window
  runs from the last accepted scan, and above 1 s the seed falls back to the previous
  scan's motion, which advances at most 1 s. After the 5 s gap the seed had moved
  6.2 m of the 33.4 m driven, every scan was rejected, and the reinitialization
  request came 19 s later.
- **A wrong alignment can pass `score_threshold: 6`.** The poses more than 1 m off
  were accepted at fitness 0.5-5.9 with 50-74% of the points within 0.5 m of the map,
  against 96% at the median when tracking (85% at the 1st percentile).
  `min_inlier_ratio: 0.8` rejects them, but then the laps with the 2 s and 5 s gaps
  and both stops publish no pose after the fault either.

Since then the preset sets `min_inlier_ratio: 0.8`, and a rejected scan past the IMU
window is aligned once more from the IMU prediction, up to 10 s. On 2026-11-07:

| Fault | Poses published during it | Largest error | Back under 0.5 m after |
| --- | --- | --- | --- |
| LiDAR gap, 2 s | 0 | 0.25 m | 0.06 s |
| LiDAR gap, 5 s | 0 | 0.28 m | 0.07 s |
| LiDAR gap, 30 s | 0 | no pose for the rest of the lap | never |
| node stopped, 2 s | 0 | no pose for the rest of the lap | never |
| node stopped, 10 s | 0 | no pose for the rest of the lap | never |

- **The gaps of 2 and 5 s recover on the first scan after them.** That scan's
  previous-scan seed is rejected, and the retry from the IMU, over a 2.1 and a 5.1 s
  window, is accepted with 98-99% inliers. Both laps score 0.126-0.127 m RMSE, like
  the laps without a fault.
- **No wrong pose is accepted in any lap.** The longer faults end in a
  reinitialization request 14-18 s later.
- **The stops lose IMU samples too.** In the second after the node resumes, 5-15
  samples arrive per scan instead of 21, so the prediction no longer covers the window
  and there is nothing to retry from.

Since then a stop of 2 s recovers too. On 2026-11-08, twice each:

| Fault | Poses published during it | Largest error | Back under 0.5 m after |
| --- | --- | --- | --- |
| node stopped 2 s, 50 s into the lap, in a turn | 17-18 | 0.24 m | 0.06 s |
| node stopped 2 s, 30 s into the lap | 19 | 0.27 m | 0.04 s |
| node stopped 10 s | 0 | no pose for the rest of the lap | never |

- **The IMU gap stayed in every window.** A window starts at the last accepted scan,
  and a gap of more than 0.5 s between IMU samples broke its coverage, so after a stop
  no IMU prediction was made again: `imu_integrated_sample_count` stayed 0 while 18-31
  samples arrived per scan. For the retry, the sample before a gap is now held across
  it, up to the retry's 10 s.
- **Held across a turn, that prediction is short of it.** The 2 s stop at 50 s falls in a
  turn tightening from 5 to 12 deg/s. Its first retry was 8 deg short in yaw and 0.73 m
  off, and aligned with 67% inliers; turned by -4 to -12 deg it aligned with 98%, 3 cm
  from the reference, while turned the other way it never passed 63%. A rejected
  bridged retry is now aligned again turned by -6, 6, -12 and 12 deg, within 3 s of
  the last accepted scan, and the best that passes the gate is taken.
- **After 10 s the prediction is metres and 20 deg off,** beyond that search. The node
  rejects every scan and requests reinitialization, as before. With the global
  localization recovery nodes beside it (`--companion`), it does not recover either:
  their map-wide search misses there and answers in 18-21 s
  ([global localization](global_localization.md#on-the-boreas-fixture-after-a-10-s-stop)).

#### Under a memory limit

With the node alone in a memory cgroup, `--wrap` set to a script that writes its PID
into the cgroup's `cgroup.procs` and runs the node, on 2026-11-08, with
`use_imu_preintegration: true` and the preset's former 100 m crop:

| Memory limit | Result |
| --- | --- |
| 1 GB | an hour, 29 laps at 0.127 m RMSE, every scan accepted; 890 MiB at most, and the limit was never reached |
| 768 MB | killed 105 s into the first lap |
| 640 MB | killed at 95 s |
| 512 MB | killed at 80 s |

- **The peak comes with the dense stretch.** The kills fall where 1.4 million map
  points lie within the crop, as the heap's peak did. With `local_map_async_rebuild`
  the node holds a second target while it builds the next one.
- **The crop radius sets the peak** (troubleshooting:
  [The Node Is Killed for Memory](troubleshooting.md#symptom-the-node-is-killed-for-memory)).
  With the 80 m crop the preset now uses, an hour under 800 MB passed: 29 laps at
  0.127 m, 765 MiB at most, pose latency 57 ms at p95 and 222 ms at most. The cgroup
  reached its limit 4070 times and gave up page cache each time; nothing was killed.
  Without the IMU the 80 m crop scores 0.132 m like the 100 m one and peaks 75 MiB
  lower; the trajectories differ by 1.8 cm at most (3.0 cm with it).
- **The lowest memory of each lap rises over the hour**, by 50-100 MB, from 600-610
  to 660-710 MB with the 100 m crop and from 574 to 609-628 MB with the 80 m one.

Since then the node frees a replaced registration target before it builds the next
one, and returns the freed memory to the OS:

- **Three crops were alive during a rebuild.** heaptrack over one lap with the 80 m crop
  put 110 MB of crops in the 547 MiB peak: the one in use, the replaced one, still held
  by the spare registration objects with its search tree, and the new one. The next
  target is now built on fresh objects, and the replaced one is freed before the crop.
- **glibc kept what was freed.** It returns only the top of each heap, and a target is
  freed in the middle of it. With `malloc_trim` every 5 s (an `LD_PRELOAD` shim) the
  lowest memory of each lap stayed at 378-400 MB for an hour. The node now trims once
  a target change freed memory, at the end of that scan's callback: a trim takes 1.4 ms
  at the median and 15 ms at most.
- **An hour under 800 MB:** 29 laps at 0.127 m, 576 MiB at most (765 MiB before), the
  lowest memory of each lap 329-339 MB (580-632 MB before), and the limit never reached
  (4070 times before).
- **One lap each, alternated with the build before, three times:** 525-539 against
  613-652 MiB at the peak, the same alignment time (9.5-9.8 against 9.4-10.0 ms at the
  median) and RMSE, and pose latency 58-59 against 55-57 ms at p95: once per target
  change, the next scan waits for the trim.

`summary.json` also holds each lap's pose latency: from publishing a cloud to receiving
its pose, at the median, p95 and most, and how many poses came over 200 ms late.


## Boreas with the small-computer knobs

The knobs of [the low-resource preset](mid360_legged_jetson.md#low-resource-preset) that
need no odometry, on the vehicle case: the 120 s fixture of the Boreas section above,
`boreas_ndt_velodyne.yaml` (no IMU, 1.5 m voxel, 8 NDT threads, 25 iterations, no pose
timer), the localizer pinned to two cores with `taskset` as in the budget below, replays
at 1x on 2026-11-19, 1158 scans, all aligned. Boreas has no wheel odometry, so the
seed is the localizer's own motion model. CPU is CPU seconds per second of data and per
scan; latency is the median / p95 / largest from the control node receiving a cloud to
receiving its pose.

```bash
python3 tools/benchmark/boreas_replay.py FIXTURE OUT --ws WS \
  --wrap "budget.sh 2,3 0" --set voxel_leaf_size=2.0
```

| setting | RMSE / max (m) | cores | CPU ms/scan | peak RSS (MB) | latency (ms) |
|---|---|---|---|---|---|
| preset (2 runs) | 0.132 / 0.41 | 0.81, 0.84 | 83, 87 | 438, 448 | 27-28 / 84-88 / 228-244 |
| `ndt_num_threads: 2` | 0.132 / 0.41 | 0.88 | 91 | 432 | 31 / 119 / 299 |
| `ndt_num_threads: 1` | 0.132 / 0.41 | 0.90 | 94 | 441 | 37 / 148 / 384 |
| `ndt_max_iterations: 15` | 0.132 / 0.40 | 0.77 | 80 | 457 | 28 / 71 / 227 |
| 2 threads, 15 iterations (2 runs) | 0.132 / 0.40 | 0.88, 0.93 | 91, 96 | 427-428 | 32-34 / 105-122 / 338-346 |
| `voxel_leaf_size: 2.0` (2 runs) | 0.135 / 0.47 | 0.64, 0.76 | 67, 79 | 457, 466 | 21-25 / 55-73 / 203-269 |
| 2 threads, 15 iterations, voxel 2.0 (2 runs) | 0.135 / 0.47 | 0.78, 0.81 | 81, 84 | 425-431 | 25-26 / 84-88 / 214-287 |
| `voxel_leaf_size: 1.0` | 0.134 / 0.69 | 1.05 | 109 | 451 | 41 / 137 / 392 |
| `enable_timer_publishing` (30 Hz) | 0.551 / 2.04 | 0.74 | 77 | 452 | |

- **Only the scan voxel saved CPU**: 2.0 m took 10-20% off (the two runs differ by
  more than the 5% of run-to-run noise on the shared host, but both are below both
  baselines) for 2% RMSE. Threads and iterations saved none and fewer threads cost
  latency: the localizer with 8 threads on two cores finished scans sooner than with 2
  or 1.
- **The pose timer is unsafe here**: with the timer, 4009 poses are published for the
  1158 scans, the extrapolated ones scored against the ground truth (likely the cause),
  RMSE 0.551 m. The preset's timer is off.
- Within the 10% accuracy bound: voxel 2.0 (+2%), 1.0 (+2%), and threads or iterations
  (0%). The latency columns are not comparable across hosts; the Orin has not been
  measured.

## Low-resource budget (Go2, MID-360)

For a Jetson Orin Nano/NX on a Unitree Go2 or G1, shared with perception: how much
CPU and memory the localizer needs, and what can be cut
([the preset](mid360_legged_jetson.md#low-resource-preset)). Measured on 2026-11-18
on a 4-vCPU x86 host (Xeon, 2.3 GHz), the localizer pinned to two cores
(`taskset`) in its own cgroups, the replay harness and the odometry on the other
two. The Orin's Cortex-A78AE cores are slower for this code, so the CPU figures are a
lower bound. CPU is in cores: CPU seconds per second of data, the mean and the p95 of
1 s intervals. Memory is the resident set (VmRSS, with about 53 MB of shared-library
pages) and its anonymous part (RssAnon). Latency runs from the replay's control node
receiving a cloud to receiving its pose.

```bash
python3 tools/benchmark/boreas_replay.py FIXTURE OUT --ws WS \
  --preset param/mid360_legged_low_resource.yaml \
  --points-topic /livox/lidar --imu-topic /livox/imu --base-frame livox_frame \
  --wrap "budget.sh 2,3 256" \
  --companion "rko_lio online_node --ros-args --params-file rko_jeplo_online.yaml"
python3 scripts/run_koide_public_bag.py --data-dir DATA --output OUT \
  --preset param/mid360_legged_low_resource.yaml --set base_frame_id=livox_frame \
  --set imu_accel_scale=1.0 --wrap "budget.sh 2,3 0"
```

`budget.sh CPUS MEM_MB` writes its PID into a memory cgroup limited to MEM_MB (and,
for the quota runs, a cpu cgroup with `cpu.cfs_quota_us`), then
`exec taskset -c CPUS "$@"`. The JEPLO fixture is a bag converted with
`jeplo_to_pc2_bag.py`, a map of EIL_Mix built from RKO-LIO's trajectory and moved
into the motion-capture frame (580k points), and the ground truth as
`reference.tum`; the localizer starts from the first ground-truth pose. The Koide
runs also pass `use_odom_tf_prediction`, `publish_bridge_pose_when_lost`,
`use_sim_time`, `point_timestamp_unit: auto`, no pose timer and the reliable
cloud QoS; that bag is already in m/s^2.

### EIL_Stairs with odometry

RKO-LIO publishes `odom -> livox_frame`; `use_odom_tf_prediction` and
`publish_bridge_pose_when_lost` are on. Each row is `mid360_legged.yaml` with one
change, then the combinations. 137 s, 1370 scans:

| setting | RMSE / max (m) | aligned | cores mean / p95 | CPU ms/scan | peak RSS (MB) | latency median / p95 / max (ms) |
|---|---|---|---|---|---|---|
| `mid360_legged.yaml` | 0.077 / 0.82 | 1359/1370 | 0.27 / 0.36 | 27.2 | 132 | 24 / 34 / 80 |
| `mid360_legged.yaml`, odometry in m/s^2 | 0.066 / 0.27 | 1370/1370 | 0.24 / 0.33 | 24.5 | 123 | 22 / 30 / 125 |
| `ndt_num_threads: 2` | 0.066 / 0.25 | 1363/1370 | 0.26 / 0.38 | 26.7 | 123 | 24 / 36 / 86 |
| `ndt_num_threads: 1` | 0.066 / 0.27 | 1370/1370 | 0.24 / 0.33 | 24.1 | 133 | 26 / 35 / 90 |
| `ndt_max_iterations: 15` | 0.066 / 0.27 | 1370/1370 | 0.23 / 0.31 | 23.7 | 133 | 22 / 30 / 47 |
| `voxel_leaf_size: 0.4` | 0.067 / 0.29 | 1370/1370 | 0.18 / 0.25 | 18.7 | 122 | 20 / 26 / 42 |
| `voxel_leaf_size: 0.5` | 0.067 / 0.26 | 1370/1370 | 0.16 / 0.22 | 16.4 | 133 | 19 / 24 / 38 |
| `scan_max_range: 30` | 0.066 / 0.26 | 1370/1370 | 0.23 / 0.34 | 23.2 | 133 | 22 / 30 / 70 |
| `use_continuous_time_deskew: false` | 0.066 / 0.26 | 1370/1370 | 0.22 / 0.32 | 22.8 | 122 | 21 / 30 / 74 |
| `use_imu_preintegration: false` | 0.066 / 0.33 | 1361/1370 | 0.35 / 0.48 | 35.6 | 121 | 22 / 32 / 93 |
| `imu_smoother_window_size: 20` | 0.066 / 0.26 | 1370/1370 | 0.21 / 0.30 | 21.3 | 132 | 19 / 27 / 68 |
| `ndt_resolution: 2.0` | 0.293 / 1.22 | 1365/1370 | 0.24 / 0.33 | 24.0 | 124 | 22 / 31 / 71 |
| map voxelized at 0.1 m (133k points) | 0.081 / 0.32 | 1370/1370 | 0.22 / 0.30 | 22.0 | 88 | 22 / 29 / 52 |
| map voxelized at 0.2 m (28k points) | 0.103 / 0.38 | 1361/1370 | 0.21 / 0.29 | 21.4 | 79 | 22 / 28 / 41 |
| `min_scan_interval_sec: 0.19` (5 Hz) | 0.076 / 0.78 | 685/1370 | 0.15 / 0.22 | 15.1 | 133 | 24 / 34 / 64 |
| `enable_timer_publishing` (20 Hz) | 0.084 / 0.62 | 1370/1370 | 0.25 / 0.36 | 25.1 | 133 | 22 / 32 / 80 |
| threads 2, voxel 0.5, 15 iterations | 0.068 / 0.28 | 1370/1370 | 0.18 / 0.24 | 18.4 | 132 | 20 / 26 / 34 |
| the same, window 20 | 0.067 / 0.27 | 1370/1370 | 0.15 / 0.21 | 15.2 | 132 | 17 / 22 / 32 |
| the same, window 10 | 0.068 / 0.27 | 1370/1370 | 0.13 / 0.17 | 13.2 | 132 | 15 / 19 / 27 |
| the same (window 50), 5 Hz | 0.067 / 0.25 | 685/1370 | 0.12 / 0.16 | 12.4 | 121 | 21 / 27 / 37 |
| `mid360_legged_low_resource.yaml` | 0.067 / 0.26 | 1370/1370 | 0.21 / 0.28 | 21.7 | 132 | 21 / 28 / 38 |
| the same, repeat | 0.067 / 0.26 | 1370/1370 | 0.20 / 0.26 | 20.2 | 121 | 20 / 27 / 38 |
| the same, in 256 MB | 0.067 / 0.26 | 1370/1370 | 0.19 / 0.26 | 19.6 | 121 | 20 / 27 / 34 |
| the same, window 30 | 0.067 / 0.26 | 1363/1370 | 0.16 / 0.20 | 16.5 | 132 | 18 / 22 / 59 |
| the same, window 20, 5 Hz | 0.127 / 1.26 | 682/1370 | 0.10 / 0.13 | 9.7 | 132 | 17 / 22 / 33 |
| the same, 5 Hz | 0.072 / 0.52 | 685/1370 | 0.14 / 0.18 | 13.9 | 132 | 22 / 29 / 35 |

- **The scan voxel is the largest single cut**: 0.5 m leaves about 550 of 1550 points
  and a third of the CPU, at the same error. The NDT resolution is not: 2.0 m
  quadrupled the error.
- **The IMU smoother is the next largest cost.** perf with the 0.5 m voxel: the
  smoother's sparse solves took 25% of the node's CPU, the ROS executor and DDS 18%,
  NDT 16%, the kernel 13%, malloc and copies 11%, the fitness kd-tree 6%, scan
  conversion, deskew and voxel filter 4%. It re-optimizes all 50 scans of its window
  on every update. `imu_smoother_window_size` (new) at 20 or 10 cuts 18-29% here
  with the same error, but lost the Koide bag (below).
- **Without the IMU, more CPU, not less** (0.35 against 0.24 cores): NDT then needs
  more iterations from a worse seed.
- **Threads do not save CPU**, they spread it: one thread costs the same and adds
  4 ms of latency. Deskew (0.02 cores), the range (0.01) and the pose timer (0.01)
  cost little.
- **The map's point count sets the memory**: 580k points, 133k (0.1 m) and 28k
  (0.2 m) peaked at 123-133, 88 and 79 MB, but the error rose by 23% and 56%.
- **Five Hertz is marginal**: 0.12-0.15 cores and 0.067-0.076 m RMSE, but up to
  0.52-1.26 m off.
- The first two rows differ in RKO-LIO's acceleration unit: `g`, as the MID-360
  publishes it, in the first; m/s^2 in the second, where its initialization absorbed
  gravity into an 8.8 m/s^2 accelerometer bias. Run to run, the CPU varies by about
  0.02 cores and the resident memory between 121 and 133 MB.

RKO-LIO itself took 0.06-0.07 cores and 40 MB (13 MB anonymous).

### EIL_Stairs without odometry

| setting | RMSE / max (m) | aligned | cores mean / p95 | CPU ms/scan | peak RSS (MB) | latency median / p95 / max (ms) |
|---|---|---|---|---|---|---|
| `mid360_legged.yaml` | 0.130 / 1.76 | 1354/1370 | 0.29 / 0.43 | 29.4 | 132 | 15 / 29 / 68 |
| `voxel_leaf_size: 0.5` | 0.300 / 1.53 | 1207/1370 | 0.17 / 0.25 | 18.6 | 132 | 61 / 107 / 125 |
| threads 2, voxel 0.5, 15 iterations | 0.266 / 1.38 | 1201/1370 | 0.19 / 0.25 | 19.3 | 132 | 62 / 107 / 126 |
| threads 2, 15 iterations, window 20 | 1.018 / 13.18 | 1091/1370 | 0.31 / 0.74 | 31.2 | 132 | 64 / 109 / 126 |

Without odometry the localizer seeds from its own motion model. The 0.5 m scan then
locked onto the wrong stair level (1.5 m up) for seconds, with a good fitness, and
two threads, 15 iterations and a 20-scan window lost the robot (13 m off at 19 s).
Do not cut below `mid360_legged.yaml` without odometry.

### IMU acceleration units

`livox_ros_driver2` publishes the MID-360 acceleration in g, and the localizer does not
detect the unit: `imu_accel_scale` multiplies it. `mid360_legged.yaml` shipped
`1.0` (as the low-resource preset shipped `9.80665`) until 2026-11-19. With
`mid360_legged.yaml` on the JEPLO EIL_Stairs bag (IMU in g), RKO-LIO running as the
odometry (its TF unused, `use_odom_tf_prediction` off), pinned to two cores, two runs each:

| `imu_accel_scale` | RMSE / max (m) |
|---|---|
| `1.0` (before) | 0.197 / 1.64, 0.271 / 1.52 |
| `9.80665` (after) | 0.117 / 0.33, 0.139 / 1.78 |

CPU is the same (0.19-0.21 cores, 100 MB). Without the odometry's `odom` frame in the TF
tree (the first attempt, which had no companion) both values did worse (5.4 and 1.0 m)
because `enable_map_odom_tf` needs the frame; that is a harness issue, not the unit.
On the Koide `outdoor_hard_02b` bag, whose IMU is in m/s^2, `mid360_legged.yaml` with
odometry seeding and no pose timer gave 0.242 m at `1.0` and 0.873 m at `9.80665` (on the
map 0.193 and 0.195 m, off it 0.32 and 1.55 m): a driver in m/s^2 needs `1.0`, so
set it to match the driver, with `--set imu_accel_scale=1.0` for the Koide replays.

### Under a CPU quota and a memory limit

The same two cores with a CFS quota (`cpu.cfs_quota_us` per 100 ms period); the rows
"window 20" are an earlier version of the preset with a 20-scan IMU window:

| run | quota (cores) | aligned | RMSE / max (m) | latency median / p95 / max (ms) |
|---|---|---|---|---|
| `mid360_legged.yaml` | 0.25 | 1253/1370 | 0.067 / 0.26 | 74 / 261 / 562 |
| `mid360_legged.yaml` | 0.25 | 1190/1370 | 0.067 / 0.25 | 153 / 252 / 765 |
| low resource | 0.25 | 1370/1370 | 0.067 / 0.26 | 20 / 97 / 113 |
| low resource | 0.15 | 1049/1370 | 0.069 / 0.26 | 112 / 212 / 408 |
| low resource, window 20 | 0.25 | 1370/1370 | 0.067 / 0.27 | 16 / 21 / 119 |
| low resource, window 20 | 0.15 | 1360/1370 | 0.068 / 0.27 | 26 / 129 / 233 |
| low resource, window 20 | 0.15 | 1349/1370 | 0.067 / 0.24 | 16 / 113 / 212 |
| low resource, window 20 | 0.10 | 1070/1370 | 0.068 / 0.24 | 136 / 238 / 437 |
| low resource, window 20, 5 Hz | 0.10 | 680/1370 | 0.066 / 0.23 | 17 / 116 / 310 |

- **Short of CPU, the node drops scans** (`cloud_queue_depth: 1`) instead of
  building up lag, and the odometry carries the pose: no quota run came more than
  0.27 m off. The preset needs about 0.2 cores here; at 0.15 it aligns 77% of the
  scans.
- **Memory**: `mid360_legged.yaml` and the preset passed in a 256 MB cgroup, the
  preset with window 20 also in 128 and 96 MB, at the unlimited error. The node's anonymous memory is
  67-78 MB with this map, and the cgroup reclaims page cache first.

### Koide outdoor_hard_02b

A handheld MID-360 outdoors, 298 s, about 100 s of it off the 2.0M-point map;
`run_koide_public_bag.py` with its own RKO-LIO:

| setting | RMSE / on map / p95 (m) | localizer cores mean / p95 | CPU ms/scan | peak / steady RSS (MB) | anonymous peak / steady (MB) | latency median / p95 / max (ms) | RKO-LIO cores, peak RSS |
|---|---|---|---|---|---|---|---|
| `koide_outdoor_hard_02b_online.yaml` (75 m crop, no IMU) | 0.240 / 0.171 / 0.54 | 0.44 / 0.76 | 60 | 361 / 297 | 289 / 244 | 28 / 95 / 456 | 0.31, 168 MB |
| `mid360_legged.yaml` | 0.245 / 0.193 / 0.58 | 0.70 / 1.57 | 95 | 360 / 357 | 306 / 302 | 43 / 183 / 551 | 0.31, 170 MB |
| `mid360_legged_low_resource.yaml` | 0.228 / 0.203 / 0.46 | 0.50 / 1.01 | 68 | 354 / 350 | 300 / 296 | 36 / 119 / 320 | 0.31, 173 MB |
| the same, repeat | 0.227 / 0.203 / 0.43 | 0.45 / 0.88 | 62 | 354 / 351 | 300 / 297 | 30 / 109 / 268 | 0.32, 171 MB |
| the same, IMU window 20 | 4.340 / 5.228 / 12.50 | 0.53 / 0.99 | 72 | 356 / 350 | 302 / 296 | 37 / 125 / 354 | 0.34, 172 MB |
| the same, IMU window 20, 30 iterations | 3.221 / 3.884 / 11.51 | 0.53 / 1.09 | 72 | 352 / 351 | 298 / 297 | 35 / 144 / 318 | 0.32, 170 MB |
| the same, IMU window 30 | 22.204 / 19.085 / 32.03 | 0.70 / 1.13 | 95 | 350 / 348 | 296 / 294 | 55 / 150 / 364 | 0.32, 170 MB |

- **The preset takes 0.45-0.50 cores instead of 0.70**, at the same error.
  `mid360_legged.yaml` dropped 3% of the scans (coverage 96.8%, a FAIL of the
  replay's 97% threshold).
- **A shorter IMU smoother window lost the pose**: from 67 s on, with 20 scans for
  about 40 s (up to 14-24 m off, both runs), with 30 scans for the rest of the bag
  (up to 38 m). With 50 the same settings held. The cause was one IMU retry seed, not
  the window; it is fixed ([below](#memory-and-cpu-in-code)).
- **Memory is the map's**: about 350 MB resident and 300 MB anonymous whatever the
  preset. In 256 MB and in 192 MB the node was killed while it built the NDT target
  (192 MB: at 194 MB anonymous). heaptrack put the heap's peak at 292 MB: the map
  points (XYZI, 32 bytes each) 63 MB, the NDT voxel grid 58 MB, the map file as read
  24 MB, and about 100 MB in FLANN kd-trees, mostly the one over the map that scores
  the fitness. Since then the node fits in 224 MB ([below](#memory-and-cpu-in-code)).
- **RKO-LIO costs as much as the localizer outdoors**: 0.31-0.34 cores and 170 MB
  (136 MB anonymous) here, against 0.07 cores and 40 MB indoors.

### Memory and CPU in code

The same setup on 2026-11-18, `main` before these changes against after: the fitness
kd-tree and map loading (memory), the IMU smoother's block solve (CPU) and no IMU
retry after an odometry seed. CPU is CPU seconds per second of data and the p95 of
1 s intervals; memory is peak / steady, anonymous then resident.

| run | RMSE / largest (m) | aligned | CPU s/s, p95 | ms/scan | anonymous MB | resident MB | latency median / p95 (ms) |
|---|---|---|---|---|---|---|---|
| EIL_Stairs, low resource, before | 0.067 / 0.24-0.26 | 1363-1370 | 0.20-0.22, 0.26-0.28 | 20-22 | 68-77 | 121-132 | 21 / 28-29 |
| EIL_Stairs, low resource, after | 0.067 / 0.26 | 1370 | 0.145, 0.18 | 14.5 | 44 | 98 | 15.5 / 20 |
| EIL_Stairs, `mid360_legged.yaml`, before | 0.066 / 0.25 | 1370 | 0.29, 0.42 | 29 | 78 | 132 | 24 / 36 |
| EIL_Stairs, `mid360_legged.yaml`, after | 0.066 / 0.25 | 1364 | 0.21, 0.28 | 21 | 46 | 100 | 18 / 25 |
| Koide, low resource, before | 0.228-0.236 (0.201-0.203 on the map) | | 0.51-0.52, 1.01 | 69-70 | 300-302 / 295 | 354-356 | 35 / 119-124 |
| Koide, low resource, after (3 runs) | 0.227-0.235 (0.201-0.203) | | 0.40-0.41, 0.70-0.71 | 54-55 | 218-221 / 215 | 272-275 | 26-27 / 90-93 |
| Koide, `mid360_legged.yaml`, before | 0.245 (0.193) | | 0.70, 1.57 | 95 | 306 / 303 | 360 | 43 / 183 |
| Koide, `mid360_legged.yaml`, after | 0.238 (0.193) | | 0.59, 0.96 | 80 | 225 / 219 | 280 | 37 / 137 |
| Boreas 120 s, before | 0.132 / 0.41 | 1158 | 0.79, 1.37 | 82 | 496 / 389 | 554 / 442 | 28 / 75 |
| Boreas 120 s, after | 0.132 / 0.41 | 1158 | 0.77, 1.41 | 80 | 397 / 272 | 456 / 326 | 26 / 77 |
| Boreas, `use_imu_preintegration`, before (2 runs) | 0.127 / 0.36 | 1158 | 0.73, 1.47-1.48 | 75-76 | 468-482 / 383-384 | 538-540 / 436-437 | 25 / 76-83 |
| Boreas, `use_imu_preintegration`, after (2 runs) | 0.127 / 0.36 | 1158 | 0.73-0.78, 1.55-1.59 | 76-81 | 375-377 / 305-313 | 434-438 / 359-366 | 23-25 / 90-110 |

- **The fitness kd-tree was the largest single block of memory.** PCL's FLANN tree
  copies the target's coordinates twice and adds an index mapping: 98 MB of heap for
  the 2.0M-point Koide map in a micro-benchmark. `CompactKdTree` keeps a 4-byte
  index per point and its nodes, 13 MB, and reads the points from the target; it
  returns the same squared distances (all 200 000 test queries bit-identical,
  near and up to 20 m off the map) and answered 23% faster. Freeing the map file as
  read (24 MB) and the xyz-to-XYZI temporary (32 MB) before the target is built took
  the load off the peak: the node's anonymous memory is now highest at the end of the
  run, not while it builds the target.
- **The 2.0M-point map now runs in a 224 MB cgroup** (low resource, 0.235 m); it was
  killed at 256 MB before. 192 MB, tried with the kd-tree change only, was killed
  during the run. The anonymous memory still grows by about 30 MB over the 298 s,
  likely glibc keeping freed memory: fixed `MALLOC_MMAP_THRESHOLD_` and
  `MALLOC_TRIM_THRESHOLD_` (4 and 8 MB) took 10-15 MB of it off; not adopted.
- **The IMU smoother is about four times cheaper.** Its normal equations are block
  tridiagonal over the poses plus the bias rows; they are now solved by blocks with a
  6 x 6 Schur complement, instead of triplets and a general sparse LDLT. On a
  synthetic 1500-scan walk an update took 0.46 instead of 2.0 ms (window 50), an IMU
  sample 0.76 instead of 0.99 us, with the same poses and biases to 1e-15. On the
  replays that is 0.055-0.06 cores, a quarter of the low-resource preset's CPU on
  EIL_Stairs (0.20 with the kd-tree change alone, 0.145 with this). On Koide the
  changes together took 0.51 to 0.40 cores.
- **Why a shorter IMU window lost the Koide pose**: it did not, by itself. At 435.7 s
  of the ground truth, 9.9 s into a streak of rejected scans whose odometry seeds were
  0.2-0.3 m off, the node aligned the scan again from the IMU prediction dead-reckoned
  over those 9.9 s. That alignment came 10-35 m off with a fitness of 2.9-4.3 (the
  threshold is 6) and was accepted, in 8 of 13 runs: three of the study's four with a
  20- or 30-scan window, and all five with the kd-tree change alone (their scans were
  processed about 7 ms sooner, which likely changed the IMU data the retry saw; not
  traced). A scan seeded from odometry no longer gets that retry; since then all six
  Koide runs held, two of them with a 20-scan window (0.227-0.228 m). With the
  smoother this cheap, 20 scans save only about 0.01 cores (0.134 against 0.145 on
  EIL_Stairs, the same on Koide), so the preset keeps 50.
- **Boreas** (`boreas_ndt_velodyne.yaml`, local map crops) gave a byte-identical
  trajectory without the IMU and one within 1.5 cm with it; its memory fell by the
  crops' kd-trees, about 100 MB. Its CPU did not change: the smoother is a small part
  of its 75-80 ms per scan, and there is no odometry seed.
