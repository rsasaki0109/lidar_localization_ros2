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
  /* With localizability, for NDT_OMP: the Hessian of the NDT score at the
   * pose, row-major in (x, y, z, roll, pitch, yaw). Its inverse, negated,
   * approximates the pose covariance up to the scale of the NDT score. */
  int score_hessian_valid;
  double score_hessian[36];
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
  /* The map-distance prefilter on the primary alignment (config's
   * map_distance_filter_m above 0), as the node decides it
   * (map_distance_filter.hpp): from map_distance_filter_warmup_accepts
   * accepted scans after the initial pose, at map_distance_filter_m growing by
   * growth_m_per_s per second since the last accepted scan beyond one scan
   * period, up to max_distance_m. Retries align the whole scan. */
  double map_distance_filter_growth_m_per_s;
  double map_distance_filter_max_distance_m;
  int map_distance_filter_warmup_accepts;
  double scan_period_sec;
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
  /* Local re-acquisition (lidarloc_set_reacquisition): whether this scan was
   * aligned from the previous scan's proposal (accepted then says whether the
   * gate confirmed it), whether the search ran after this scan's rejection,
   * its outcome ("proposed", "ambiguous", "best_fitness_above_threshold", ...)
   * and, when proposed, the pose the next scan is seeded from. */
  int seeded_from_reacquisition;
  int reacquisition_attempted;
  int reacquisition_proposed;
  char reacquisition_reason[32];
  double reacquisition_pose[16];
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

/* Local re-acquisition after an odometry-bridged outage, as the node's
 * enable_local_reacquisition (local_reacquisition_policy.hpp), on an
 * occupancy grid generated from the map (occupancy_map_generation.hpp). It
 * runs in lidarloc_localize on scans given a seed (the caller's odometry
 * bridge: the last tracked pose moved by odometry) once min_rejections
 * consecutive scans were rejected, every attempt_interval_scans scans: a 2D
 * branch-and-bound search within search_radius_m of the seed, near its
 * heading, each candidate refined by the registration; the best is proposed
 * when it passes the score threshold and no other place fits nearly as well.
 * The next scan with a seed is aligned from the seed moved by the proposal's
 * correction, and the gate on that scan confirms it or not. */
typedef struct lidarloc_reacquisition
{
  int enable;
  int min_rejections;
  int attempt_interval_scans;
  double search_radius_m;
  double yaw_window_deg;
  int max_candidates;
  double candidate_separation_m;
  double min_fitness_ratio;
  double scan_min_z_m;
  double scan_max_z_m;
  double scan_min_range_m;
  double scan_voxel_m;
  int scan_max_points;
  double angular_resolution_deg;
  int pyramid_depth;
  /* The occupancy grid from the map: a cell is occupied when it holds
   * occupancy_min_points_per_cell points between occupancy_obstacle_height_m
   * and occupancy_max_obstacle_height_m (0: no limit) above its lowest
   * point, inflated by occupancy_inflate_radius_m. */
  double occupancy_resolution_m;
  double occupancy_obstacle_height_m;
  double occupancy_max_obstacle_height_m;
  int occupancy_min_points_per_cell;
  double occupancy_inflate_radius_m;
} lidarloc_reacquisition;

/* The node's defaults (disabled). */
lidarloc_reacquisition lidarloc_default_reacquisition(void);
/* Sets the re-acquisition; with enable, builds the occupancy grid from the
 * map (now, or when the map is set). 0 on success. */
int lidarloc_set_reacquisition(lidarloc_core * core, const lidarloc_reacquisition * reacquisition);

/* Global localization: where in the whole map a scan was taken, with no
 * prior (a start, a robot lost). The G2 engine "bbs3d" search
 * (bbs3d_topk_search.hpp, on 3D-BBS's voxel maps of the map) proposes top_k
 * poses at least min_separation_m apart; each is refined by the registration,
 * and the best is found when it passes the tracking's score threshold and no
 * candidate that refined farther than candidate_separation_m fits nearly as
 * well (min_fitness_ratio), as local re-acquisition decides
 * (global_localization.hpp). The voxel maps are built on the first call and
 * kept until the map or their parameters change. The caller then starts
 * tracking there (lidarloc_set_initial_pose). */
typedef struct lidarloc_global_config
{
  /* The map's voxel hierarchy: thinned to map_leaf_m voxel centroids, finest
   * voxels min_level_res_m, max_level coarser levels. */
  double map_leaf_m;
  double min_level_res_m;
  int max_level;
  /* The scan as the search sees it (base frame, levelled). */
  double scan_min_range_m;
  double scan_max_range_m;
  double scan_leaf_m;
  int top_k;
  double min_separation_m;
  /* Keep searching for candidates scoring at least this share of the best. */
  double relative_score_floor;
  int timeout_ms;
  size_t max_queue_nodes;
  int num_threads;
  double max_tilt_rad;
  double candidate_separation_m;
  double min_fitness_ratio;
} lidarloc_global_config;

typedef struct lidarloc_global_result
{
  int found;
  /* "proposed", "ambiguous", "best_fitness_above_threshold",
   * "no_converged_candidate", "no_candidates", "no_scan_points", ... */
  char reason[32];
  /* The pose found (map <- sensor) and its fitness; when none was found,
   * the best refined candidate's (fitness infinite when none converged). */
  double pose[16];
  double fitness;
  /* Candidates the search returned, and the best fitness among the others. */
  size_t candidates;
  double second_fitness;
  size_t scan_points;
  int timed_out;
  double voxelmap_build_ms;
  double search_ms;
  double total_ms;
} lidarloc_global_result;

/* The G2 bbs3d defaults (global_localization_query.py). */
lidarloc_global_config lidarloc_default_global_config(void);
/* Searches the map for the scan (count x, y, z triplets, sensor frame). 0 when
 * the search ran (found or not), nonzero when it could not (no map, an empty
 * scan, bad parameters). */
int lidarloc_global_localize(
  lidarloc_core * core, const float * xyz, size_t count, const lidarloc_global_config * config,
  lidarloc_global_result * result);

/* The last message the engine logged (empty when none). */
const char * lidarloc_last_log(const lidarloc_core * core);

#ifdef __cplusplus
}
#endif

#endif  /* LIDAR_LOCALIZATION_CORE_C_API_H_ */
