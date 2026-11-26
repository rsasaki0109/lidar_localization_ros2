/* lidar_localization_core's C API: the registration engine
 * (registration_engine.hpp) for programs outside ROS 2 and outside C++, such
 * as simulators that call it through an FFI. The same code the node runs: a
 * map, a local target around the pose, the NDT initializer, one alignment
 * with its fitness, inlier ratio and localizability.
 *
 * Poses are 4x4 row-major homogeneous transforms (map <- sensor). Points are
 * x, y, z float triplets in the map frame (maps) or the sensor frame (scans).
 */
#ifndef LIDAR_LOCALIZATION_CORE_C_API_H_
#define LIDAR_LOCALIZATION_CORE_C_API_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct lidarloc_core lidarloc_core;

typedef struct lidarloc_config
{
  /* NDT_OMP, GICP_OMP, NDT, GICP, SMALL_GICP, SMALL_VGICP. Copied at creation. */
  const char * registration_method;
  double transform_epsilon;
  double ndt_resolution;
  double ndt_step_size;
  int ndt_max_iterations;
  /* 0 or less: all OpenMP threads. */
  int ndt_num_threads;
  int gicp_corr_randomness;
  double gicp_max_correspondence_distance;
  double vgicp_voxel_resolution;
  /* Voxel leaf for the filtered targets and, with filter_scan, the scans. */
  double voxel_leaf_size;
  int filter_scan;
  int enable_local_map_crop;
  double local_map_radius;
  size_t local_map_min_points;
  double local_map_update_distance;
  double scan_max_range;
  int ndt_init_scans_required;
  /* registration_fitness.hpp: a point is an inlier within this distance of the
   * map; scans of fewer points get an inlier ratio of 0. */
  double inlier_ratio_distance_m;
  size_t inlier_ratio_min_points;
  /* Report the NDT_OMP Hessian's localizability. */
  int localizability;
  /* map_distance_filter.hpp: above 0, align only the scan points within this
   * distance of the map at the guess (people and vehicles the map lacks), or the
   * whole scan when fewer than map_distance_filter_min_kept_ratio of them are. The
   * guess must then be within about this distance of the pose: leave it 0 for a
   * guess that is not a tracked pose (a start, a reset, a long gap). */
  double map_distance_filter_m;
  double map_distance_filter_min_kept_ratio;
} lidarloc_config;

typedef struct lidarloc_result
{
  /* Whether a registration target could be set around the guess. */
  int target_ready;
  int has_converged;
  /* -1 when the backend does not report it. */
  int iterations;
  /* The aligned pose; the guess when not converged. */
  double pose[16];
  double fitness_score;
  double inlier_ratio;
  double alignment_time_sec;
  /* Points of the scan given to the backend (after filter_scan). */
  size_t source_points;
  /* With localizability, for NDT_OMP: whether the Hessian was analysed, its
   * eigenvalues (ascending absolute value) and the weakest one's direction
   * (x, y, z, roll, pitch, yaw). */
  int localizability_valid;
  double eigenvalues[6];
  double weakest_eigenvector[6];
  double weak_ratio;
  size_t correspondence_count;
  /* With map_distance_filter_m: the source points within it of the map at the
   * guess, and whether only those were aligned. */
  size_t map_distance_filter_kept_points;
  int map_distance_filter_applied;
} lidarloc_result;

/* The node's parameter defaults, but NDT_OMP (the node's default is PCL's NDT)
 * and the localizability analysed. */
lidarloc_config lidarloc_default_config(void);

/* NULL on failure, with the reason in error (when error_size > 0). */
lidarloc_core * lidarloc_create(const lidarloc_config * config, char * error, size_t error_size);
void lidarloc_destroy(lidarloc_core * core);

/* Sets the map (count x, y, z triplets). 0 on success. */
int lidarloc_set_map(lidarloc_core * core, const float * xyz, size_t count);

/* Aligns a scan (count x, y, z triplets, sensor frame) to the map from the
 * guess: sets the target around the guess, refines the guess with the NDT
 * initializer while it runs, aligns, and fills result. 0 when the alignment
 * ran (converged or not), nonzero when it could not (no map, empty scan). */
int lidarloc_align(
  lidarloc_core * core, const float * xyz, size_t count, const double guess[16],
  lidarloc_result * result);

/* The tracking around each alignment in lidarloc_localize: the node's
 * measurement gate, its retry from the last accepted pose and its prediction
 * (the node's parameters of the same names; no IMU). */
typedef struct lidarloc_tracking
{
  /* The gate rejects a score above this (NDT fitness, or with
   * min_inlier_ratio the inlier ratio on that scale). */
  double score_threshold;
  double min_inlier_ratio;
  int enable_seed_correction_guard;
  double seed_correction_guard_translation_m;
  double seed_correction_guard_yaw_deg;
  int seed_correction_guard_release_rejections;
  int seed_correction_guard_warmup_accepts;
  int enable_seed_correction_window_guard;
  double seed_correction_window_guard_sec;
  double seed_correction_window_guard_translation_m;
  int enable_recovery_retry_from_last_pose;
  int recovery_retry_from_last_pose_min_rejections;
  double recovery_retry_from_last_pose_max_accepted_gap_sec;
  /* Without a seed from the caller: the last accepted motion extrapolated
   * (as predict_pose_from_previous_delta), else the tracked pose. */
  int predict_pose_from_previous_delta;
  double previous_delta_max_extrapolation_sec;
} lidarloc_tracking;

typedef struct lidarloc_update
{
  /* Whether the gate accepted the scan and the tracked pose moved to it. */
  int accepted;
  /* The node's status: level 0 ok, 1 warn, 2 error, and its message
   * (e.g. "ok", "score_threshold_rejected", "registration_not_converged"). */
  int status_level;
  char status[64];
  /* The tracked pose after this scan, and the seed the scan was aligned from. */
  double pose[16];
  double seed[16];
  int retried_from_last_pose;
  size_t consecutive_rejected_updates;
  /* The alignment the gate judged. */
  lidarloc_result alignment;
} lidarloc_update;

/* The node's defaults for the tracking. */
lidarloc_tracking lidarloc_default_tracking(void);
/* Sets the tracking; 0 on success. */
int lidarloc_set_tracking(lidarloc_core * core, const lidarloc_tracking * tracking);
/* Starts tracking at a pose (an initial pose, a reset); 0 on success. */
int lidarloc_set_initial_pose(lidarloc_core * core, const double pose[16], double stamp_sec);

/* One scan through the node's per-scan flow, without the IMU: the seed (the
 * caller's, e.g. predicted from odometry, or NULL for the prediction), the
 * NDT initializer, the alignment, a retry from the last accepted pose, the
 * measurement gate, and the update of the tracked pose and the prediction.
 * Needs a map and an initial pose. 0 when the scan was processed (accepted or
 * not), nonzero when it could not be (no map or initial pose, empty scan). */
int lidarloc_localize(
  lidarloc_core * core, const float * xyz, size_t count, double stamp_sec,
  const double seed[16], lidarloc_update * update);

/* The last message the engine logged (empty when none). */
const char * lidarloc_last_log(const lidarloc_core * core);

#ifdef __cplusplus
}
#endif

#endif  /* LIDAR_LOCALIZATION_CORE_C_API_H_ */
