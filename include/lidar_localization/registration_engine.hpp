// The scan-to-map registration of lidar_localization, without ROS: the
// registration backend (NDT-OMP, GICP-OMP, PCL NDT/GICP, small_gicp), the map
// and its registration target (the whole map, or a local crop around the pose,
// optionally rebuilt in the background), the NDT initializer for the GICP
// backends, and one alignment with its fitness and localizability.
//
// PCLLocalization (the ROS 2 node) owns one; so can any other program that
// gives it a map and scans (lidar_localization_core, see
// include/lidar_localization/core_c_api.h). Messages go to the LogSink the
// owner provides.
#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>

#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>
#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
#include <small_gicp/pcl/pcl_registration.hpp>
#endif

#include "lidar_localization/diagnostic_ndt_omp.hpp"
#include "lidar_localization/map_distance_filter.hpp"
#include "lidar_localization/map_initialization_policy.hpp"
#include "lidar_localization/registration_backend_policy.hpp"
#include "lidar_localization/registration_cloud_keep_alive_policy.hpp"
#include "lidar_localization/registration_fitness.hpp"
#include "lidar_localization/registration_localizability_policy.hpp"

namespace lidar_localization
{

enum class LogLevel { kInfo, kWarn, kError };

// Where the engine's messages go (the node forwards them to its ROS logger).
using LogSink = std::function<void (LogLevel, const std::string &)>;

// printf-style formatting into a std::string, for LogSink messages.
std::string formatLog(const char * format, ...)
#if defined(__GNUC__)
__attribute__((format(printf, 1, 2)))
#endif
;

// The node's parameter defaults (component_lifecycle.cpp).
struct RegistrationEngineConfig
{
  // NDT_OMP, GICP_OMP, NDT, GICP, or a small_gicp backend
  // (registration_backend_policy.hpp).
  std::string registration_method{"NDT"};
  double transform_epsilon{0.01};
  double ndt_resolution{1.0};
  double ndt_step_size{0.1};
  int ndt_max_iterations{35};
  // 0 or less: all OpenMP threads.
  int ndt_num_threads{4};
  int gicp_corr_randomness{20};
  double gicp_max_correspondence_distance{2.0};
  double vgicp_voxel_resolution{1.0};
  double voxel_leaf_size{0.2};
  // Crop the map around the pose for each target instead of registering
  // against the whole map.
  bool enable_local_map_crop{false};
  double local_map_radius{150.0};
  std::size_t local_map_min_points{100};
  double local_map_update_distance{0.0};
  bool local_map_async_rebuild{false};
  double scan_max_range{100.0};
  std::size_t source_cloud_keep_alive_count{kDefaultRegistrationSourceCloudKeepAliveCount};
  std::size_t target_cloud_keep_alive_count{kDefaultRegistrationTargetCloudKeepAliveCount};
  int ndt_init_scans_required{5};
};

// One registration call: what the backend reports.
struct RegistrationAlignResult
{
  bool has_converged{false};
  // -1 when the backend does not report it.
  int iterations{-1};
  Eigen::Matrix4f final_transformation{Eigen::Matrix4f::Identity()};
  double alignment_time_sec{0.0};
};

// One map-distance prefilter of a source (map_distance_filter.hpp).
struct MapDistanceFilterResult
{
  // kApplied, or why the whole source was set (kTooFewKept, kNoTargetTree).
  MapDistanceFilterStatus status{MapDistanceFilterStatus::kDisabled};
  std::size_t input_points{0};
  // The points within the distance of the target, whether or not they were set.
  std::size_t kept_points{0};
  double distance_m{0.0};
  // The nearest-neighbour searches and the copy.
  double time_sec{0.0};
};

class RegistrationEngine
{
public:
  using PointT = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<PointT>;
  // A lock held while the backend aligns or its target is replaced, so a map
  // switch or a target swap never races an align(). The default is the
  // engine's own mutex.
  using ExecutionLock = std::unique_lock<std::mutex>;
  using ExecutionLocker = std::function<ExecutionLock()>;

  // The owner of one registration backend; only the member for the method is
  // set.
  struct RegistrationObjects
  {
    pcl::Registration<PointT, PointT>::Ptr pcl;
    boost::shared_ptr<DiagnosticNdtOmp<PointT, PointT>> ndt_omp;
    pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp_omp;
#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
    small_gicp::RegistrationPCL<PointT, PointT>::Ptr small_gicp;
#endif
    pcl::Registration<PointT, PointT> * get() const;
    // Iterations of the last align(); -1 when the backend does not report them.
    int finalIterationCount() const;
  };

  explicit RegistrationEngine(LogSink log = {}, ExecutionLocker locker = {});
  ~RegistrationEngine();
  RegistrationEngine(const RegistrationEngine &) = delete;
  RegistrationEngine & operator=(const RegistrationEngine &) = delete;

  // Builds the backend for config.registration_method; false (with an error
  // logged) when the method is invalid or unavailable. Drops the targets and
  // sources of a previous configuration.
  bool configure(const RegistrationEngineConfig & config);
  const RegistrationEngineConfig & config() const {return config_;}

  // A map loaded from a file: the registration target is the whole map, or,
  // with enable_local_map_crop, crops of it around the pose
  // (setTargetForPose). Builds the NDT initializer for the GICP backends that
  // crop. Takes the execution lock.
  void setMap(const Cloud::Ptr & map, const MapBounds3d & bounds);
  // A map from a topic: always the whole map (voxel-filtered for the
  // backends that filter their target), never cropped.
  void setMapFromTopic(const Cloud::Ptr & map);
  bool usesLocalMapCrop() const {return use_local_map_crop_;}

  // Consecutive failures to set a local target (scan_admission_policy.hpp's
  // crop failure guard reads them), and the guard itself.
  int consecutiveCropFailures() const {return consecutive_crop_failures_;}
  bool cropFailureGuardActive() const {return crop_failure_guard_active_;}
  void activateCropFailureGuard() {crop_failure_guard_active_ = true;}
  // An admitted initial pose clears the guard and its log throttles.
  void resetCropFailureGuard();

  // With local-map cropping, makes the crop around the pose the target
  // (reusing the current one within local_map_update_distance, or one built
  // in the background); false when no target can be set there. Without
  // cropping, true.
  bool setTargetForPose(const Eigen::Matrix4f & center_pose);
  void discardLocalMapRebuild();

  void setSource(const Cloud::Ptr & source);
  // Sets as the source the points of SOURCE within DISTANCE_M of the registration
  // target when moved by GUESS (map_distance_filter.hpp), or SOURCE whole when the
  // target has no search tree yet (a GICP backend before its first alignment) or
  // fewer than MIN_KEPT_RATIO of its points would be kept. Call it after
  // setTargetForPose(GUESS), so the distance is to the target that will be aligned to.
  MapDistanceFilterResult setSourceNearTarget(
    const Cloud::Ptr & source, const Eigen::Matrix4f & guess, double distance_m,
    double min_kept_ratio);
  // The seed refined by the NDT initializer while it runs (the first
  // ndt_init_scans_required scans after a map load, GICP backends with
  // cropping); otherwise the seed.
  Eigen::Matrix4f refineSeedWithNdtInitializer(
    const Cloud::Ptr & source, const Eigen::Matrix4f & seed);

  // Aligns the source to the target from the guess. The backend call runs
  // under the execution lock.
  RegistrationAlignResult align(const Eigen::Matrix4f & guess);
  // The last alignment's fitness (registration_fitness.hpp).
  RegistrationFitness fitness(double inlier_distance_m, std::size_t inlier_min_points) const;
  // The last NDT-OMP alignment's localizability; nullopt for the other
  // backends.
  std::optional<RegistrationLocalizabilityMetrics> localizability() const;

  // Whether a target change freed memory since the last call (the node then
  // returns it to the OS).
  bool takeHeapTrimPending();
  // Frees the backend, targets and map. With leak_target_clouds, the target
  // clouds are leaked instead of freed (a shutdown that must not wait for
  // their destructors).
  void release(bool leak_target_clouds = false);

  pcl::Registration<PointT, PointT> * registration() const {return registration_;}
  const RegistrationObjects & registrationObjects() const {return registration_objects_;}
  // Whether a local target is being built in the background, and waiting for it.
  bool localMapRebuildPending() const {return local_map_rebuild_.valid();}
  void waitForLocalMapRebuild() const
  {
    if (local_map_rebuild_.valid()) {
      local_map_rebuild_.wait();
    }
  }
  // The center of the current local target, once one is set.
  std::optional<Eigen::Vector2f> localMapTargetCenter() const
  {
    if (!local_map_target_cached_) {
      return std::nullopt;
    }
    return Eigen::Vector2f(local_map_target_center_x_, local_map_target_center_y_);
  }
  ExecutionLock lockExecution() const;

private:
  struct LocalMapRebuild
  {
    bool ready{false};
    float center_x{0.0f};
    float center_y{0.0f};
    Cloud::Ptr target;
    Cloud::Ptr warm_up_source;
  };

  RegistrationObjects makeRegistrationObjects(RegistrationBackend backend) const;
  void warmUpRegistrationTarget(const Cloud::Ptr & target);
  void startLocalMapRebuild(float center_x, float center_y);
  bool takeLocalMapRebuild(float center_x, float center_y, bool wait);
  void log(LogLevel level, const std::string & message) const;
  int threads() const;

  LogSink log_;
  ExecutionLocker locker_;
  mutable std::mutex execution_mutex_;
  RegistrationEngineConfig config_;

  pcl::Registration<PointT, PointT> * registration_{nullptr};
  RegistrationObjects registration_objects_;
  // local_map_async_rebuild: the next local target is built on this twin of
  // registration_objects_ in the background, then the two are swapped.
  RegistrationObjects spare_registration_objects_;
  std::future<LocalMapRebuild> local_map_rebuild_;
  float local_map_rebuild_center_x_{0.0f};
  float local_map_rebuild_center_y_{0.0f};
  std::atomic<bool> heap_trim_pending_{false};
  Cloud::Ptr registration_source_cloud_;
  pcl::VoxelGrid<PointT> voxel_grid_filter_;

  Cloud::Ptr full_map_cloud_ptr_;
  MapBounds3d map_bounds_{};
  std::deque<Cloud::Ptr> recent_source_clouds_;
  std::deque<Cloud::Ptr> recent_target_clouds_;
  bool use_local_map_crop_{false};
  bool local_map_target_cached_{false};
  float local_map_target_center_x_{0.0f};
  float local_map_target_center_y_{0.0f};
  int consecutive_crop_failures_{0};
  bool crop_failure_guard_active_{false};
  std::chrono::steady_clock::time_point last_crop_out_of_bounds_log_time_{};
  std::chrono::steady_clock::time_point last_crop_failure_streak_log_time_{};

  // NDT initializer for the GICP backends with cropping.
  bool use_ndt_initializer_{false};
  int ndt_init_scan_count_{0};
  pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt_initializer_;
};

}  // namespace lidar_localization
