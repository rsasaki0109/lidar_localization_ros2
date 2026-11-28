/* Replays scans through lidarloc_localize (core_c_api.h) with an odometry-like
 * seed and reports the position error against ground truth.
 *
 *   replay MAP.bin SCANS.bin GT.bin REMAP MIN_HC SCALE [TRACE.csv]
 *
 * MAP.bin:   uint64 n, then n float x, y, z (map frame).
 * SCANS.bin: per scan: double stamp, uint32 n, n float x, y, z (sensor frame).
 * GT.bin:    per scan: double stamp, 16 double pose (row-major map <- sensor).
 * REMAP:     0 or 1 (enable_degeneracy_remapping).
 * MIN_HC:    degeneracy_remapping_min_horizontal_constraint (0 = off).
 * SCALE:     the seed's odometry scale: each scan's seed is the tracked pose
 *            moved by the ground-truth motion since the previous scan with its
 *            translation multiplied by SCALE (a wheel that reads long), as the
 *            node is seeded from odometry.
 *
 * With the environment variable PER_SCAN_OFFSET_M above 0, each scan is judged
 * on its own instead: tracking restarts at the previous scan's ground truth and
 * the seed is the scan's ground truth moved that far horizontally.
 *
 * Prints one line: scans, accepted, remapped, RMSE and max position error (m),
 * final error (m), and with PER_SCAN_OFFSET_M the scans left more than 5 cm
 * farther from the truth than the seed. With TRACE.csv, writes stamp, error, accepted, remapped,
 * dropped (m), horizontal constraint (2) per scan. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lidar_localization/core_c_api.h"

static void mul(const double a[16], const double b[16], double out[16])
{
  double r[16];
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k) {
        s += a[4 * i + k] * b[4 * k + j];
      }
      r[4 * i + j] = s;
    }
  }
  memcpy(out, r, sizeof(r));
}

/* Inverse of a rigid transform. */
static void inv(const double a[16], double out[16])
{
  double r[16] = {0};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      r[4 * i + j] = a[4 * j + i];
    }
  }
  for (int i = 0; i < 3; ++i) {
    r[4 * i + 3] = -(r[4 * i + 0] * a[3] + r[4 * i + 1] * a[7] + r[4 * i + 2] * a[11]);
  }
  r[15] = 1.0;
  memcpy(out, r, sizeof(r));
}

static void * read_file(const char * path, size_t * size)
{
  FILE * f = fopen(path, "rb");
  if (!f) {
    perror(path);
    exit(1);
  }
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fseek(f, 0, SEEK_SET);
  void * data = malloc((size_t)n);
  if (fread(data, 1, (size_t)n, f) != (size_t)n) {
    fprintf(stderr, "short read: %s\n", path);
    exit(1);
  }
  fclose(f);
  *size = (size_t)n;
  return data;
}

int main(int argc, char ** argv)
{
  if (argc < 7) {
    fprintf(stderr, "usage: replay MAP.bin SCANS.bin GT.bin REMAP MIN_HC SCALE [TRACE.csv]\n");
    return 2;
  }
  size_t map_size, scans_size, gt_size;
  unsigned char * map = read_file(argv[1], &map_size);
  unsigned char * scans = read_file(argv[2], &scans_size);
  double * gt = read_file(argv[3], &gt_size);
  const int remap = atoi(argv[4]);
  const double min_hc = atof(argv[5]);
  const double scale = atof(argv[6]);
  FILE * trace = argc > 7 ? fopen(argv[7], "w") : NULL;
  const size_t gt_count = gt_size / (17 * sizeof(double));

  lidarloc_config config = lidarloc_default_config();
  config.ndt_resolution = 0.5;
  config.voxel_leaf_size = 0.1;
  config.filter_scan = 1;
  config.enable_local_map_crop = 0;
  config.ndt_num_threads = 1;
  config.scan_max_range = 10.0;
  char error[256] = {0};
  lidarloc_core * core = lidarloc_create(&config, error, sizeof(error));
  if (!core) {
    fprintf(stderr, "create: %s\n", error);
    return 1;
  }
  uint64_t map_points;
  memcpy(&map_points, map, sizeof(map_points));
  if (lidarloc_set_map(core, (const float *)(map + 8), (size_t)map_points) != 0) {
    fprintf(stderr, "set_map failed\n");
    return 1;
  }
  lidarloc_tracking tracking = lidarloc_default_tracking();
  tracking.enable_seed_correction_guard = 1;
  tracking.seed_correction_guard_translation_m = 0.5;
  /* As quickstart.py turns on with an odometry seed: at most 0.3 m of corrections
   * in all over 2 s. */
  tracking.enable_seed_correction_window_guard = 1;
  tracking.seed_correction_window_guard_sec = 2.0;
  tracking.seed_correction_window_guard_translation_m = 0.3;
  tracking.enable_degeneracy_remapping = remap;
  tracking.degeneracy_remapping_min_horizontal_constraint = min_hc;
  tracking.scan_period_sec = 0.2;
  lidarloc_set_tracking(core, &tracking);

  size_t offset = 0, k = 0, accepted = 0, remapped = 0;
  double sum_sq = 0.0, max_error = 0.0, last_error = 0.0;
  double tracked[16];
  double previous_stamp = 0.0;
  /* PER_SCAN_OFFSET_M > 0: judge each scan on its own (see below). */
  const char * per_scan = getenv("PER_SCAN_OFFSET_M");
  const double per_scan_offset_m = per_scan ? atof(per_scan) : 0.0;
  size_t worse = 0;
  while (offset + 12 <= scans_size && k < gt_count) {
    double stamp;
    uint32_t n;
    memcpy(&stamp, scans + offset, 8);
    memcpy(&n, scans + offset + 8, 4);
    const float * xyz = (const float *)(scans + offset + 12);
    offset += 12 + (size_t)n * 12;
    const double * truth = gt + 17 * k + 1;
    if (k == 0) {
      memcpy(tracked, truth, sizeof(tracked));
      previous_stamp = stamp;
      lidarloc_set_initial_pose(core, truth, stamp);
      ++k;
      continue;
    }
    /* Seed: the tracked pose moved by the scaled ground-truth motion. */
    double previous_inverse[16], motion[16], seed[16];
    inv(gt + 17 * (k - 1) + 1, previous_inverse);
    mul(previous_inverse, truth, motion);
    motion[3] *= scale;
    motion[7] *= scale;
    motion[11] *= scale;
    if (per_scan_offset_m > 0.0) {
      /* Every scan on its own: tracking restarts at the previous scan's ground
       * truth, and the seed is this scan's ground truth moved horizontally by
       * the offset, in a direction that turns by the golden angle per scan. */
      memcpy(tracked, gt + 17 * (k - 1) + 1, sizeof(tracked));
      lidarloc_set_initial_pose(core, tracked, previous_stamp);
      memcpy(seed, truth, sizeof(seed));
      const double angle = 2.399963 * (double)k;
      seed[3] += per_scan_offset_m * cos(angle);
      seed[7] += per_scan_offset_m * sin(angle);
    } else {
      mul(tracked, motion, seed);
    }
    previous_stamp = stamp;
    lidarloc_update update;
    if (lidarloc_localize(core, xyz, n, stamp, seed, &update) != 0) {
      memcpy(tracked, seed, sizeof(tracked));
    } else {
      memcpy(tracked, update.accepted ? update.pose : seed, sizeof(tracked));
      accepted += update.accepted ? 1 : 0;
      remapped += update.alignment.degeneracy_remapped ? 1 : 0;
    }
    const double dx = tracked[3] - truth[3], dy = tracked[7] - truth[7], dz = tracked[11] - truth[11];
    const double e = sqrt(dx * dx + dy * dy + dz * dz);
    sum_sq += e * e;
    worse += (per_scan_offset_m > 0.0 && e > per_scan_offset_m + 0.05) ? 1 : 0;
    max_error = e > max_error ? e : max_error;
    last_error = e;
    if (trace) {
      fprintf(
        trace, "%.6f,%.4f,%d,%d,%.4f,%.4f,%.4f\n", stamp, e, update.accepted,
        update.alignment.degeneracy_remapped, update.alignment.degeneracy_dropped_m,
        update.alignment.horizontal_constraint[0], update.alignment.horizontal_constraint[1]);
    }
    ++k;
  }
  const double rmse = sqrt(sum_sq / (double)(k > 1 ? k - 1 : 1));
  printf(
    "scans %zu accepted %zu remapped %zu rmse %.3f max %.3f final %.3f worse_than_seed %zu\n",
    k - 1, accepted, remapped, rmse, max_error, last_error, worse);
  if (trace) {
    fclose(trace);
  }
  lidarloc_destroy(core);
  return 0;
}
