# Quickstart Validation Record

This page records the Phase 4 validation boundary for the guarded quickstart feature.
It distinguishes tests that ran from datasets that were inspected but could not run.

## Automated coverage

On ROS 2 Jazzy, the package was built with `colcon build --symlink-install` and the full
package CTest suite passed. The focused quickstart coverage includes:

- ROS-free map identity, atomic pose persistence, malformed/expired state, topic
  discovery, startup policy, global-query timeout, and distinct-scan consensus tests;
- CLI generation and invalid-input tests, including the saved-pose-only command;
- launch argument and installed-file contract tests;
- a real rclpy graph with a fake G2 service, two scan timestamps, `/initialpose`,
  tracking diagnostics, state transitions, and verified pose persistence.

## Koide real-bag validation

Run date: 2026-07-22. The mounted public Koide assets contained the 380.39 s
`outdoor_hard_01a` ROS 2 bag, `map_outdoor_hard.ply`, and its generated BBS occupancy
map. They are external validation data and are not shipped in this repository.

Cold-start replay used the installed `quickstart.py`, the standalone profile,
`/livox/points`, simulation time, `--global-seed-z -11.04`, no RViz, and 0.1x bag
playback to approximate the documented stationary initialization condition.

Observed result:

- the compiled G2 backend and `g2_ndt_score` loaded successfully;
- guarded queries completed in 10.865 s, 12.344 s, and 11.228 s;
- the accepted 3D-ranked candidate was near `(-86.56, -8.41, -11.04, -80 deg)`;
- distinct-scan consensus published `/initialpose`, fresh localizer diagnostics reached
  `global_pose_verified`, and the startup state reached `active`;
- the atomic state record contained the exact map SHA-256 and a verified pose near
  `(-86.10, -8.86, -11.01)`.

A second launch omitted the occupancy map and reused that state record. The current scan
passed pre-publication NDT verification at fitness `3.326`, then the localizer reached
`saved_pose_verified` and `active`. This exercises both automatic startup sources on
real sensor messages.

An earlier 1.0x moving replay intentionally failed consensus and published no pose. That
is the expected boundary: cold-start global search requires a stationary robot, and a
weak, ambiguous, stale, inconsistent, or over-time result falls back to RViz.

## Dataset availability boundary

The mounted filesystem was searched for ROS bags and matching pointcloud maps:

| Dataset | Available assets | Phase 4 result |
| --- | --- | --- |
| Koide | bag, matching 3D map, occupancy map | cold start and saved restore passed |
| HDL | 120 per-frame `cloud.pcd` snapshots only; no HDL bag or matching map | replay skipped: incomplete input pair |
| Istanbul | no Istanbul bag or pointcloud map | replay skipped: dataset absent |

The HDL and Istanbul rows are not pass claims. Re-run their normal package replay tools
and the same quickstart workflow when a matching bag/map pair is mounted.
