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

/* The last message the engine logged (empty when none). */
const char * lidarloc_last_log(const lidarloc_core * core);

#ifdef __cplusplus
}
#endif

#endif  /* LIDAR_LOCALIZATION_CORE_C_API_H_ */
