/* lidar_localization_core's C API from C: a map of a room with a pillar, a
 * scan of it from a pose the guess is off from, and the alignment that finds
 * the pose. */
#undef NDEBUG
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lidar_localization/core_c_api.h"

/* The room: a 20 x 14 m floor, walls 3 m high and a 1 m square pillar off
 * centre, sampled every 0.2 m. Returns the point count; xyz may be NULL. */
static size_t room(float * xyz)
{
  size_t n = 0;
  const float step = 0.2f;
#define PUT(X, Y, Z) do { if (xyz) { xyz[3 * n] = (X); xyz[3 * n + 1] = (Y); xyz[3 * n + 2] = (Z); } \
    ++n; } while (0)
  for (float x = -10.0f; x <= 10.0f; x += step) {
    for (float y = -7.0f; y <= 7.0f; y += step) {
      PUT(x, y, 0.0f);
    }
  }
  for (float z = step; z <= 3.0f; z += step) {
    for (float x = -10.0f; x <= 10.0f; x += step) {
      PUT(x, -7.0f, z);
      PUT(x, 7.0f, z);
    }
    for (float y = -7.0f; y <= 7.0f; y += step) {
      PUT(-10.0f, y, z);
      PUT(10.0f, y, z);
    }
    for (float s = 0.0f; s <= 1.0f; s += step) {
      PUT(3.0f + s, 2.0f, z);
      PUT(3.0f + s, 3.0f, z);
      PUT(3.0f, 2.0f + s, z);
      PUT(4.0f, 2.0f + s, z);
    }
  }
#undef PUT
  return n;
}

static void pose(double x, double y, double yaw, double m[16])
{
  const double c = cos(yaw), s = sin(yaw);
  const double p[16] = {c, -s, 0, x, s, c, 0, y, 0, 0, 1, 0, 0, 0, 0, 1};
  for (int k = 0; k < 16; ++k) {
    m[k] = p[k];
  }
}

/* Aligns the scan of the map seen from truth, from guess, with the method; the
 * pose must come within tolerance_m and tolerance_rad of the truth. */
static void check(
  const char * method, const float * map, const float * scan, size_t n, const double truth[16],
  const double guess[16], double tolerance_m, double tolerance_rad)
{
  lidarloc_config config = lidarloc_default_config();
  config.registration_method = method;
  config.ndt_num_threads = 2;
  /* The node's 0.01 stops NDT early on this small room. */
  config.transform_epsilon = 1e-4;
  char error[256];
  lidarloc_core * core = lidarloc_create(&config, error, sizeof error);
  assert(core);
  assert(lidarloc_set_map(core, map, n) == 0);
  lidarloc_result result;
  assert(lidarloc_align(core, scan, n, guess, &result) == 0);
  printf(
    "%s: converged %d after %d iterations at (%.3f, %.3f), fitness %.4f, inliers %.2f\n",
    method, result.has_converged, result.iterations, result.pose[3], result.pose[7],
    result.fitness_score, result.inlier_ratio);
  fflush(stdout);
  assert(result.target_ready && result.has_converged);
  assert(hypot(result.pose[3] - truth[3], result.pose[7] - truth[7]) < tolerance_m);
  assert(
    fabs(atan2(result.pose[4], result.pose[0]) - atan2(truth[4], truth[0])) < tolerance_rad);
  assert(result.inlier_ratio > 0.9);
  /* Only NDT_OMP reports its Hessian: symmetric, and the NDT score at its
   * maximum (negative definite: a negative diagonal). */
  assert(result.localizability_valid == (method[0] == 'N'));
  assert(result.score_hessian_valid == (method[0] == 'N'));
  if (result.score_hessian_valid) {
    for (int r = 0; r < 6; ++r) {
      assert(result.score_hessian[7 * r] < 0.0);
      for (int c = 0; c < 6; ++c) {
        assert(fabs(result.score_hessian[6 * r + c] - result.score_hessian[6 * c + r]) <=
          1e-6 * fabs(result.score_hessian[7 * r]) + 1e-9);
      }
    }
  }
  lidarloc_destroy(core);
}

/* The scan of the map seen from pose m. */
static void scan_from(const float * map, size_t n, const double m[16], float * scan)
{
  for (size_t i = 0; i < n; ++i) {
    const double dx = map[3 * i] - m[3], dy = map[3 * i + 1] - m[7];
    scan[3 * i] = (float)(m[0] * dx + m[4] * dy);
    scan[3 * i + 1] = (float)(m[1] * dx + m[5] * dy);
    scan[3 * i + 2] = map[3 * i + 2];
  }
}

/* lidarloc_localize along a drive: from an initial pose, scans every 0.1 s
 * at 2 m/s turning 0.05 rad a scan, seeded by the previous delta; every scan
 * is accepted and the track stays on the truth. Without the prefilter, a seed
 * 3 m off with the seed correction guard on is then rejected and the track
 * holds; with it, the prefilter applies from the fourth scan and keeps every
 * point. */
static void check_tracking(const float * map, size_t n, int filter)
{
  lidarloc_config config = lidarloc_default_config();
  config.registration_method = "GICP_OMP";
  config.ndt_num_threads = 2;
  /* With filter, the map-distance prefilter from the fourth accepted scan. */
  config.map_distance_filter_m = filter ? 1.0 : 0.0;
  char error[256];
  lidarloc_core * core = lidarloc_create(&config, error, sizeof error);
  assert(core);
  assert(lidarloc_set_map(core, map, n) == 0);
  lidarloc_tracking tracking = lidarloc_default_tracking();
  tracking.score_threshold = 0.5;
  tracking.enable_seed_correction_guard = 1;
  tracking.seed_correction_guard_translation_m = 0.5;
  tracking.seed_correction_guard_warmup_accepts = 3;
  tracking.map_distance_filter_warmup_accepts = 3;
  assert(lidarloc_set_tracking(core, &tracking) == 0);
  float * scan = malloc(3 * n * sizeof(float));
  lidarloc_update update;
  double truth[16];
  pose(-4.0, -3.0, 0.0, truth);
  /* Nothing before the initial pose. */
  scan_from(map, n, truth, scan);
  assert(lidarloc_localize(core, scan, n, 0.0, NULL, &update) != 0);
  assert(lidarloc_set_initial_pose(core, truth, 0.0) == 0);
  double x = -4.0, y = -3.0, yaw = 0.0;
  for (int k = 1; k <= 20; ++k) {
    x += 0.2 * cos(yaw);
    y += 0.2 * sin(yaw);
    yaw += 0.05;
    pose(x, y, yaw, truth);
    scan_from(map, n, truth, scan);
    assert(lidarloc_localize(core, scan, n, 0.1 * k, NULL, &update) == 0);
    const double error_m = hypot(update.pose[3] - x, update.pose[7] - y);
    if (!update.accepted || error_m > 0.05) {
      printf("scan %d: %s, %.3f m off\n", k, update.status, error_m);
    }
    assert(update.accepted && error_m < 0.05);
    /* Every point of this scan lies on the map. */
    assert(update.alignment.map_distance_filter_applied == (filter && k > 3));
    if (filter && k > 3) {
      assert(update.alignment.map_distance_filter_kept_points == update.alignment.source_points);
    }
  }
  printf("tracked 20 scans: %.3f m off, %s\n", hypot(update.pose[3] - x, update.pose[7] - y),
    update.status);
  /* A seed 3 m off: the guard rejects the 3 m correction and the track holds.
   * (Not with the prefilter, which needs a guess within about its distance:
   * from 3 m along the room the long walls and the floor still fit.) */
  if (filter) {
    lidarloc_destroy(core);
    free(scan);
    return;
  }
  double off[16];
  pose(x + 3.0, y, yaw, off);
  assert(lidarloc_localize(core, scan, n, 2.1, off, &update) == 0);
  printf("seed 3 m off: accepted %d, %s\n", update.accepted, update.status);
  assert(!update.accepted && update.consecutive_rejected_updates == 1);
  assert(hypot(update.pose[3] - x, update.pose[7] - y) < 0.05);
  lidarloc_destroy(core);
  free(scan);
}

/* Local re-acquisition after an odometry-bridged outage: tracking with
 * odometry seeds, the odometry then 4 m off. The guard rejects the scans;
 * after three, the search around the bridged seed proposes the true pose, and
 * the next scan, seeded from it, is accepted there. */
static void check_reacquisition(const float * map, size_t n)
{
  lidarloc_config config = lidarloc_default_config();
  config.registration_method = "GICP_OMP";
  config.ndt_num_threads = 2;
  char error[256];
  lidarloc_core * core = lidarloc_create(&config, error, sizeof error);
  assert(core);
  lidarloc_reacquisition reacquisition = lidarloc_default_reacquisition();
  assert(!reacquisition.enable);
  reacquisition.enable = 1;
  reacquisition.min_rejections = 3;
  reacquisition.attempt_interval_scans = 1;
  reacquisition.search_radius_m = 6.0;
  assert(lidarloc_set_reacquisition(core, &reacquisition) == 0);
  /* The occupancy grid is built when the map is set. */
  assert(lidarloc_set_map(core, map, n) == 0);
  lidarloc_tracking tracking = lidarloc_default_tracking();
  tracking.score_threshold = 0.5;
  tracking.enable_seed_correction_guard = 1;
  tracking.seed_correction_guard_translation_m = 0.5;
  tracking.seed_correction_guard_warmup_accepts = 3;
  assert(lidarloc_set_tracking(core, &tracking) == 0);
  float * scan = malloc(3 * n * sizeof(float));
  lidarloc_update update;
  double truth[16], seed[16];
  double x = -5.0, y = -3.0;
  pose(x, y, 0.0, truth);
  assert(lidarloc_set_initial_pose(core, truth, 0.0) == 0);
  int k = 1;
  for (; k <= 6; ++k) {
    x += 0.2;
    pose(x, y, 0.0, truth);
    scan_from(map, n, truth, scan);
    assert(lidarloc_localize(core, scan, n, 0.1 * k, truth, &update) == 0);
    assert(update.accepted && !update.reacquisition_attempted);
  }
  int proposed_at = 0;
  for (; k <= 12; ++k) {
    x += 0.2;
    pose(x, y, 0.0, truth);
    pose(x + 4.0, y + 1.0, 0.0, seed);
    scan_from(map, n, truth, scan);
    assert(lidarloc_localize(core, scan, n, 0.1 * k, seed, &update) == 0);
    printf(
      "bridged scan %d: accepted %d (%s), seeded from re-acquisition %d, attempted %d: %s\n",
      k, update.accepted, update.status, update.seeded_from_reacquisition,
      update.reacquisition_attempted, update.reacquisition_reason);
    if (update.seeded_from_reacquisition) {
      assert(update.accepted);
      assert(hypot(update.pose[3] - x, update.pose[7] - y) < 0.05);
      break;
    }
    assert(!update.accepted);
    if (update.reacquisition_proposed) {
      assert(hypot(update.reacquisition_pose[3] - x, update.reacquisition_pose[7] - y) < 0.1);
      proposed_at = k;
    }
  }
  assert(proposed_at == 9 && k == 10);
  lidarloc_destroy(core);
  free(scan);
}

/* The agreement release. Tracking with odometry seeds (the tracked pose moved
 * by the odometry's motion), then four scans that register 0.15 m off each
 * (a scene the map lacks): each is accepted and the pose slides 0.6 m away
 * from the odometry. The next scans register on the truth again; the guard
 * rejects the 0.6 m corrections back. With
 * seed_correction_guard_agreement_release_scans 3 the third agreeing one is
 * accepted, since it takes the pose back to the odometry; off (0), the guard
 * holds. An odometry that drifted 0.6 m instead (drift 1) is held either way:
 * that correction moves the pose away from the odometry. */
static void check_agreement_release(const float * map, size_t n, int scans, int drift)
{
  lidarloc_config config = lidarloc_default_config();
  config.registration_method = "GICP_OMP";
  config.ndt_num_threads = 2;
  char error[256];
  lidarloc_core * core = lidarloc_create(&config, error, sizeof error);
  assert(core);
  assert(lidarloc_set_map(core, map, n) == 0);
  lidarloc_tracking tracking = lidarloc_default_tracking();
  assert(tracking.seed_correction_guard_agreement_release_scans == 0);
  tracking.score_threshold = 0.5;
  tracking.enable_seed_correction_guard = 1;
  tracking.seed_correction_guard_translation_m = 0.3;
  tracking.seed_correction_guard_warmup_accepts = 3;
  tracking.seed_correction_guard_agreement_release_scans = scans;
  /* The drift horizon covers the slide but not the first scans, which the NDT
   * initializer refines a few centimetres off. */
  tracking.seed_correction_guard_agreement_release_drift_sec = 0.75;
  assert(lidarloc_set_tracking(core, &tracking) == 0);
  float * scan = malloc(3 * n * sizeof(float));
  lidarloc_update update;
  double truth[16], seen[16], seed[16];
  double x = -5.0;
  const double y = -3.0;
  pose(x, y, 0.0, truth);
  assert(lidarloc_set_initial_pose(core, truth, 0.0) == 0);
  /* The offset of the tracked pose from the truth. */
  double offset = 0.0;
  int k = 1;
  for (; k <= 10; ++k) {
    x += 0.2;
    pose(x, y, 0.0, truth);
    const double scan_offset = (k > 6 && !drift) ? offset + 0.15 : 0.0;
    pose(x, y + scan_offset, 0.0, seen);
    pose(x, y + offset, 0.0, seed);
    scan_from(map, n, seen, scan);
    assert(lidarloc_localize(core, scan, n, 0.1 * k, seed, &update) == 0);
    assert(update.accepted);
    offset = update.pose[7] - y;
  }
  if (drift) {
    /* The odometry jumps 0.6 m at once. */
    offset = 0.6;
  }
  assert(fabs(offset - 0.6) < 0.05);
  int released_at = 0;
  for (; k <= 16; ++k) {
    x += 0.2;
    pose(x, y, 0.0, truth);
    pose(x, y + offset, 0.0, seed);
    scan_from(map, n, truth, scan);
    assert(lidarloc_localize(core, scan, n, 0.1 * k, seed, &update) == 0);
    printf("offset scan %d (release after %d, drift %d): accepted %d, %s\n", k, scans, drift,
      update.accepted, update.status);
    if (update.accepted) {
      assert(strcmp(update.status, "seed_correction_guard_agreement_released") == 0);
      assert(hypot(update.pose[3] - x, update.pose[7] - y) < 0.05);
      released_at = k;
      break;
    }
    assert(strcmp(update.status, "seed_correction_guard_rejected") == 0);
  }
  assert(released_at == (scans > 0 && !drift ? 10 + scans : 0));
  lidarloc_destroy(core);
  free(scan);
}

int main(void)
{
  const size_t n = room(NULL);
  float * map = malloc(3 * n * sizeof(float));
  room(map);

  /* The sensor at (1.0, -0.5), turned 0.1 rad: the scan is the map in its
   * frame. */
  double truth[16];
  pose(1.0, -0.5, 0.1, truth);
  float * scan = malloc(3 * n * sizeof(float));
  for (size_t i = 0; i < n; ++i) {
    const double dx = map[3 * i] - truth[3], dy = map[3 * i + 1] - truth[7];
    scan[3 * i] = (float)(truth[0] * dx + truth[4] * dy);
    scan[3 * i + 1] = (float)(truth[1] * dx + truth[5] * dy);
    scan[3 * i + 2] = map[3 * i + 2];
  }
  double guess[16];
  pose(0.6, -0.2, 0.05, guess);
  /* GICP finds the pose exactly. NDT's 1 m cells average this room's regular
   * 0.2 m samples, and it stops a few centimetres off. */
  check("GICP_OMP", map, scan, n, truth, guess, 0.02, 0.005);
  check("NDT_OMP", map, scan, n, truth, guess, 0.1, 0.02);
  check_tracking(map, n, 0);
  check_tracking(map, n, 1);
  check_reacquisition(map, n);
  check_agreement_release(map, n, 0, 0);
  check_agreement_release(map, n, 3, 0);
  check_agreement_release(map, n, 3, 1);

  lidarloc_config config = lidarloc_default_config();
  char error[256];
  lidarloc_result result;
  /* The map-distance prefilter keeps this scan of the map whole: every point lies on
   * the map at a guess 0.4 m off. */
  config.map_distance_filter_m = 1.0;
  config.ndt_num_threads = 2;
  lidarloc_core * filtering = lidarloc_create(&config, error, sizeof error);
  assert(filtering);
  assert(lidarloc_set_map(filtering, map, n) == 0);
  assert(lidarloc_align(filtering, scan, n, guess, &result) == 0);
  printf(
    "filtered: applied %d, kept %zu of %zu\n", result.map_distance_filter_applied,
    result.map_distance_filter_kept_points, result.source_points);
  assert(result.map_distance_filter_applied == 1);
  assert(result.map_distance_filter_kept_points == result.source_points);
  lidarloc_destroy(filtering);
  config = lidarloc_default_config();

  /* No map, no alignment. */
  lidarloc_core * empty = lidarloc_create(&config, error, sizeof error);
  assert(empty);
  assert(lidarloc_align(empty, scan, n, guess, &result) != 0);
  lidarloc_destroy(empty);

  /* An unknown method is refused with a reason. */
  config.registration_method = "NOT_A_METHOD";
  assert(lidarloc_create(&config, error, sizeof error) == NULL);
  printf("refused: %s\n", error);

  free(scan);
  free(map);
  return 0;
}
