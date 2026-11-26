/* lidar_localization_core's C API from C: a map of a room with a pillar, a
 * scan of it from a pose the guess is off from, and the alignment that finds
 * the pose. */
#undef NDEBUG
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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
  /* Only NDT_OMP reports its Hessian. */
  assert(result.localizability_valid == (method[0] == 'N'));
  lidarloc_destroy(core);
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
