#include "lidar_localization/registration_engine.hpp"

#include <omp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>

#include <cstdarg>
#include <cstdio>
#include <exception>
#include <utility>
#include <vector>

#include "lidar_localization/local_map_target_policy.hpp"
#include "lidar_localization/ndt_initializer_policy.hpp"
#include "lidar_localization/registration_target_search_tree.hpp"

namespace lidar_localization
{

std::string formatLog(const char * format, ...)
{
  va_list args;
  va_start(args, format);
  va_list copy;
  va_copy(copy, args);
  const int length = std::vsnprintf(nullptr, 0, format, copy);
  va_end(copy);
  std::string text(length > 0 ? static_cast<std::size_t>(length) : 0, '\0');
  if (length > 0) {
    std::vsnprintf(text.data(), text.size() + 1, format, args);
  }
  va_end(args);
  return text;
}

namespace
{

// About 100 points spread over the target, aligned once to build what
// pcl::Registration and the backends set up lazily in the first align() after
// setInputTarget(): the target search tree, GICP target covariances, the
// small_gicp VGICP voxel map.
RegistrationEngine::Cloud::Ptr sampleWarmUpSource(const RegistrationEngine::Cloud & target)
{
  RegistrationEngine::Cloud::Ptr probe(new RegistrationEngine::Cloud());
  const std::size_t stride = std::max<std::size_t>(1, target.size() / 100);
  for (std::size_t i = 0; i < target.size(); i += stride) {
    probe->push_back(target.points[i]);
  }
  return probe;
}

}  // namespace

pcl::Registration<RegistrationEngine::PointT, RegistrationEngine::PointT> *
RegistrationEngine::RegistrationObjects::get() const
{
  if (pcl) {return pcl.get();}
  if (ndt_omp) {return ndt_omp.get();}
  if (gicp_omp) {return gicp_omp.get();}
#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
  if (small_gicp) {return small_gicp.get();}
#endif
  return nullptr;
}

int RegistrationEngine::RegistrationObjects::finalIterationCount() const
{
  if (ndt_omp) {return ndt_omp->getFinalNumIteration();}
  if (const auto * ndt =
    dynamic_cast<const pcl::NormalDistributionsTransform<PointT, PointT> *>(pcl.get()))
  {
    return ndt->getFinalNumIteration();
  }
#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
  if (small_gicp) {
    return static_cast<int>(small_gicp->getRegistrationResult().iterations);
  }
#endif
  return -1;
}

RegistrationEngine::RegistrationEngine(LogSink log, ExecutionLocker locker)
: log_(std::move(log)), locker_(std::move(locker))
{
}

RegistrationEngine::~RegistrationEngine()
{
  discardLocalMapRebuild();
}

void RegistrationEngine::log(LogLevel level, const std::string & message) const
{
  if (log_) {
    log_(level, message);
  }
}

int RegistrationEngine::threads() const
{
  return resolveRegistrationThreadCount(config_.ndt_num_threads, omp_get_max_threads());
}

RegistrationEngine::ExecutionLock RegistrationEngine::lockExecution() const
{
  if (locker_) {
    return locker_();
  }
  return ExecutionLock(execution_mutex_);
}

RegistrationEngine::RegistrationObjects RegistrationEngine::makeRegistrationObjects(
  RegistrationBackend backend) const
{
  RegistrationObjects objects;
  if (backend == RegistrationBackend::kPclGicp) {
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(
      new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>());
    gicp->setTransformationEpsilon(config_.transform_epsilon);
    gicp->setCorrespondenceRandomness(config_.gicp_corr_randomness);
    gicp->setMaxCorrespondenceDistance(config_.gicp_max_correspondence_distance);
    objects.pcl = gicp;
  } else if (backend == RegistrationBackend::kPclNdt) {
    pcl::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(
      new pcl::NormalDistributionsTransform<PointT, PointT>());
    ndt->setStepSize(config_.ndt_step_size);
    ndt->setResolution(config_.ndt_resolution);
    ndt->setTransformationEpsilon(config_.transform_epsilon);
    objects.pcl = ndt;
  } else if (backend == RegistrationBackend::kNdtOmp) {
    boost::shared_ptr<DiagnosticNdtOmp<PointT, PointT>> ndt_omp(
      new DiagnosticNdtOmp<PointT, PointT>());
    ndt_omp->setStepSize(config_.ndt_step_size);
    ndt_omp->setResolution(config_.ndt_resolution);
    ndt_omp->setTransformationEpsilon(config_.transform_epsilon);
    ndt_omp->setNumThreads(threads());
    objects.ndt_omp = ndt_omp;
  } else if (backend == RegistrationBackend::kGicpOmp) {
    pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp_omp(
      new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());
    gicp_omp->setTransformationEpsilon(config_.transform_epsilon);
    gicp_omp->setCorrespondenceRandomness(config_.gicp_corr_randomness);
    gicp_omp->setMaxCorrespondenceDistance(config_.gicp_max_correspondence_distance);
    objects.gicp_omp = gicp_omp;
  }
#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
  else if (isSmallGicpBackend(backend)) {
    small_gicp::RegistrationPCL<PointT, PointT>::Ptr reg(
      new small_gicp::RegistrationPCL<PointT, PointT>());
    reg->setTransformationEpsilon(config_.transform_epsilon);
    reg->setCorrespondenceRandomness(config_.gicp_corr_randomness);
    reg->setMaxCorrespondenceDistance(config_.gicp_max_correspondence_distance);
    reg->setVoxelResolution(config_.vgicp_voxel_resolution);
    reg->setRegistrationType(smallGicpRegistrationType(backend));
    reg->setNumThreads(threads());
    objects.small_gicp = reg;
  }
#endif
  if (objects.get()) {
    objects.get()->setMaximumIterations(config_.ndt_max_iterations);
  }
  return objects;
}

bool RegistrationEngine::configure(const RegistrationEngineConfig & config)
{
  log(LogLevel::kInfo, "initializeRegistration");
  discardLocalMapRebuild();
  config_ = config;
  registration_ = nullptr;
  registration_objects_ = RegistrationObjects();
  spare_registration_objects_ = RegistrationObjects();
  registration_source_cloud_.reset();
  recent_source_clouds_.clear();
  recent_target_clouds_.clear();

  const auto registration_backend = parseRegistrationBackend(config_.registration_method);
  if (!isValidRegistrationBackend(registration_backend)) {
    log(LogLevel::kError, "Invalid registration method.");
    return false;
  }
#ifndef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
  if (isSmallGicpBackend(registration_backend)) {
    log(
      LogLevel::kError,
      "small_gicp backend requested but support is not available. Install small_gicp and rebuild.");
    return false;
  }
#endif
  registration_objects_ = makeRegistrationObjects(registration_backend);
  registration_ = registration_objects_.get();
  if (registration_ == nullptr) {
    log(LogLevel::kError, "Registration backend setup failed.");
    return false;
  }
  if (config_.local_map_async_rebuild) {
    spare_registration_objects_ = makeRegistrationObjects(registration_backend);
  }

  const auto leaf = static_cast<float>(config_.voxel_leaf_size);
  voxel_grid_filter_.setLeafSize(leaf, leaf, leaf);
  log(LogLevel::kInfo, "initializeRegistration end");
  return true;
}

void RegistrationEngine::warmUpRegistrationTarget(const Cloud::Ptr & target)
{
  // pcl::Registration builds its target search tree lazily inside the first
  // align() call (even for NDT, which does not use it).  On a large map that
  // takes about a second and stalls the first scan, dropping the following
  // ones while the robot keeps moving.  Pay that cost at map load instead by
  // aligning a small probe cloud once.
  if (!registration_ || !target || target->empty()) {
    return;
  }
  const auto probe = sampleWarmUpSource(*target);
  const auto start = std::chrono::steady_clock::now();
  registration_->setInputSource(probe);
  keepRegistrationCloudAlive(
    recent_source_clouds_, probe, config_.source_cloud_keep_alive_count);
  Cloud output;
  registration_->align(output, Eigen::Matrix4f::Identity());
  log(
    LogLevel::kInfo,
    formatLog(
      "Registration target warm-up took %.3f s",
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count()));
}

void RegistrationEngine::setMap(const Cloud::Ptr & map_cloud_ptr, const MapBounds3d & bounds)
{
  map_bounds_ = bounds;
  // The map content changed: drop the crop guards and the cached local target.
  consecutive_crop_failures_ = 0;
  crop_failure_guard_active_ = false;
  discardLocalMapRebuild();
  local_map_target_cached_ = false;

  // A runtime map switch must not race with an align() running outside the state lock.
  auto registration_execution_lock = lockExecution();
  const auto target_setup =
    planInitialMapTargetSetup(config_.enable_local_map_crop, config_.registration_method);
  use_local_map_crop_ = target_setup.use_local_map_crop;
  if (target_setup.builds_full_map_ndt_grid) {
    if (const auto error = ndtVoxelGridOverflowError(bounds, config_.ndt_resolution)) {
      log(LogLevel::kError, *error);
    }
  }
  if (use_local_map_crop_) {
    // Keep the raw full map and only voxel-filter the cropped local target per scan.
    full_map_cloud_ptr_ = map_cloud_ptr;
    log(
      LogLevel::kInfo,
      formatLog(
        "Local map cropping enabled. Full map: %zu pts, radius: %.0fm",
        full_map_cloud_ptr_->size(), config_.local_map_radius));
    // Avoid building a full-map NDT target here. It can overflow on city-scale maps.
    if (target_setup.create_ndt_initializer) {
      use_ndt_initializer_ = true;
      ndt_init_scan_count_ = 0;
      ndt_initializer_.reset(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      ndt_initializer_->setStepSize(config_.ndt_step_size);
      ndt_initializer_->setResolution(config_.ndt_resolution);
      ndt_initializer_->setTransformationEpsilon(config_.transform_epsilon);
      ndt_initializer_->setNumThreads(threads());
      setNdtTargetWithSearchTree<PointT>(*ndt_initializer_, map_cloud_ptr);
      log(
        LogLevel::kInfo,
        formatLog(
          "NDT initializer created (%d scans before GICP switch)",
          config_.ndt_init_scans_required));
    } else {
      use_ndt_initializer_ = false;
      ndt_initializer_.reset();
    }
  } else {
    full_map_cloud_ptr_.reset();
    if (target_setup.set_full_map_as_registration_target && registration_) {
      setRegistrationTarget<PointT>(
        *registration_, map_cloud_ptr, usesNdtVoxelGrid(config_.registration_method));
      keepRegistrationCloudAlive(
        recent_target_clouds_, map_cloud_ptr, config_.target_cloud_keep_alive_count);
      warmUpRegistrationTarget(map_cloud_ptr);
    }
  }
}

void RegistrationEngine::setMapFromTopic(const Cloud::Ptr & map_cloud_ptr)
{
  const auto map_target_choice =
    chooseMapSubscriptionTargetCloud(usesFilteredTarget(config_.registration_method));
  if (map_target_choice == LocalMapTargetCloud::kFilteredLocalMap) {
    Cloud::Ptr filtered_cloud_ptr(new Cloud());
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    setRegistrationTarget<PointT>(
      *registration_, filtered_cloud_ptr, usesNdtVoxelGrid(config_.registration_method));
    keepRegistrationCloudAlive(
      recent_target_clouds_, filtered_cloud_ptr, config_.target_cloud_keep_alive_count);
    warmUpRegistrationTarget(filtered_cloud_ptr);
  } else {
    setRegistrationTarget<PointT>(
      *registration_, map_cloud_ptr, usesNdtVoxelGrid(config_.registration_method));
    keepRegistrationCloudAlive(
      recent_target_clouds_, map_cloud_ptr, config_.target_cloud_keep_alive_count);
    warmUpRegistrationTarget(map_cloud_ptr);
  }

  // The map topic replaces the whole registration target.
  discardLocalMapRebuild();
  local_map_target_cached_ = false;
}

bool RegistrationEngine::setTargetForPose(const Eigen::Matrix4f & center_pose_matrix)
{
  if (!use_local_map_crop_ || !full_map_cloud_ptr_) {
    return true;
  }

  const float cx = center_pose_matrix(0, 3);
  const float cy = center_pose_matrix(1, 3);
  const LocalMapCropRequest crop_request{
    cx,
    cy,
    config_.local_map_radius,
    config_.local_map_min_points,
    LocalMapBounds2d{
      map_bounds_.valid,
      map_bounds_.min_point.x,
      map_bounds_.max_point.x,
      map_bounds_.min_point.y,
      map_bounds_.max_point.y}};
  const auto steady_now = std::chrono::steady_clock::now();

  auto handle_crop_failure = [&](LocalMapTargetFailure failure) {
      const bool has_last_streak_log =
        last_crop_failure_streak_log_time_ != std::chrono::steady_clock::time_point{};
      const auto elapsed_since_streak_log =
        has_last_streak_log ?
        steady_now - last_crop_failure_streak_log_time_ :
        std::chrono::steady_clock::duration::zero();
      const bool has_last_bounds_log =
        last_crop_out_of_bounds_log_time_ != std::chrono::steady_clock::time_point{};
      const auto elapsed_since_bounds_log =
        has_last_bounds_log ?
        steady_now - last_crop_out_of_bounds_log_time_ :
        std::chrono::steady_clock::duration::zero();
      const auto decision = handleLocalMapTargetFailure(
        LocalMapTargetFailureHandlingInput{
          failure,
          consecutive_crop_failures_,
          has_last_streak_log,
          elapsed_since_streak_log,
          has_last_bounds_log,
          elapsed_since_bounds_log});
      consecutive_crop_failures_ = decision.consecutive_crop_failures;
      if (decision.should_log_failure_streak) {
        last_crop_failure_streak_log_time_ = steady_now;
        log(
          LogLevel::kWarn,
          formatLog(
            "Crop failure streak reached %d while target setup keeps failing.",
            consecutive_crop_failures_));
      }
      if (decision.should_log_out_of_bounds) {
        last_crop_out_of_bounds_log_time_ = steady_now;
      }
      return decision;
    };

  const auto crop_validation = validateLocalMapCropRequest(crop_request);
  if (!crop_validation.can_crop) {
    const auto failure_decision = handle_crop_failure(crop_validation.failure);
    if (failure_decision.should_log_out_of_bounds) {
      log(
        LogLevel::kWarn,
        formatLog(
          "Crop center (%.1f, %.1f) is outside map bounds + radius, skipping alignment",
          static_cast<double>(cx), static_cast<double>(cy)));
    }
    return false;
  }

  auto reuse_target = [this]() {
      const auto reuse_decision = handleLocalMapTargetSuccess();
      consecutive_crop_failures_ = reuse_decision.consecutive_crop_failures;
      crop_failure_guard_active_ = reuse_decision.crop_failure_guard_active;
      return true;
    };
  const bool async_rebuild =
    config_.local_map_async_rebuild && spare_registration_objects_.get() != nullptr;
  if (async_rebuild && takeLocalMapRebuild(cx, cy, false)) {
    return reuse_target();
  }
  if (shouldReuseLocalMapTarget(
      local_map_target_cached_, local_map_target_center_x_, local_map_target_center_y_,
      cx, cy, config_.local_map_update_distance))
  {
    return reuse_target();
  }
  if (async_rebuild) {
    const bool rebuild_running = local_map_rebuild_.valid();
    const auto step = planLocalMapAsyncRebuild(
      local_map_target_cached_,
      planarDistance(local_map_target_center_x_, local_map_target_center_y_, cx, cy),
      localMapAsyncReuseLimit(
        config_.local_map_radius, config_.scan_max_range, config_.local_map_update_distance),
      rebuild_running,
      rebuild_running && planarDistance(
        local_map_rebuild_center_x_, local_map_rebuild_center_y_, cx, cy) <
      std::max(config_.local_map_update_distance, 1.0));
    switch (step) {
      case LocalMapAsyncRebuildStep::kReuseAndStartRebuild:
        startLocalMapRebuild(cx, cy);
        return reuse_target();
      case LocalMapAsyncRebuildStep::kReuse:
        return reuse_target();
      case LocalMapAsyncRebuildStep::kWaitForRebuild:
        if (takeLocalMapRebuild(cx, cy, true)) {
          return reuse_target();
        }
        break;
      case LocalMapAsyncRebuildStep::kBuildNow:
        break;
    }
  }

  Cloud::Ptr local_map = cropLocalMapByRadius(
    *full_map_cloud_ptr_, crop_request.center_x, crop_request.center_y,
    crop_request.radius_m);
  const auto crop_size_validation =
    validateLocalMapCropSize(local_map->size(), crop_request.min_points);
  if (!crop_size_validation.target_ready) {
    handle_crop_failure(crop_size_validation.failure);
    log(
      LogLevel::kWarn,
      formatLog(
        "Local map crop too small (%zu points, min %zu) around (%.3f, %.3f) with radius %.1fm.",
        local_map->size(),
        config_.local_map_min_points,
        static_cast<double>(cx),
        static_cast<double>(cy),
        config_.local_map_radius));
    return false;
  }

  const bool filter_target = usesFilteredTarget(config_.registration_method);
  Cloud::Ptr filtered_local_map(new Cloud());
  if (filter_target) {
    voxel_grid_filter_.setInputCloud(local_map);
    voxel_grid_filter_.filter(*filtered_local_map);
  }
  const auto target_selection = chooseTargetAfterLocalMapCrop(
    local_map->size(), filtered_local_map->size(), crop_request.min_points, filter_target);
  const bool ndt_target = usesNdtVoxelGrid(config_.registration_method);
  if (target_selection.target_cloud == LocalMapTargetCloud::kFilteredLocalMap) {
    setRegistrationTarget<PointT>(*registration_, filtered_local_map, ndt_target);
    keepRegistrationCloudAlive(
      recent_target_clouds_, filtered_local_map, config_.target_cloud_keep_alive_count);
  } else {
    setRegistrationTarget<PointT>(*registration_, local_map, ndt_target);
    keepRegistrationCloudAlive(
      recent_target_clouds_, local_map, config_.target_cloud_keep_alive_count);
  }
  local_map_target_cached_ = true;
  local_map_target_center_x_ = cx;
  local_map_target_center_y_ = cy;
  // The previous crop and NDT voxel grid were freed when the target was replaced.
  heap_trim_pending_.store(true, std::memory_order_relaxed);

  const auto success_decision = handleLocalMapTargetSuccess();
  consecutive_crop_failures_ = success_decision.consecutive_crop_failures;
  crop_failure_guard_active_ = success_decision.crop_failure_guard_active;
  return true;
}

void RegistrationEngine::startLocalMapRebuild(float center_x, float center_y)
{
  local_map_rebuild_center_x_ = center_x;
  local_map_rebuild_center_y_ = center_y;
  // The next target is built on fresh objects. The spare ones still hold the target
  // replaced at the last swap, with its crop and search tree; the task frees them before
  // it crops, so a rebuild holds the target in use and the new one, not a third.
  RegistrationObjects stale = std::move(spare_registration_objects_);
  spare_registration_objects_ =
    makeRegistrationObjects(parseRegistrationBackend(config_.registration_method));
  // The task owns what it reads: a map switch or cleanup swaps the members.
  local_map_rebuild_ = std::async(
    std::launch::async,
    [stale = std::move(stale), full_map = full_map_cloud_ptr_,
    objects = spare_registration_objects_,
    radius = config_.local_map_radius, min_points = config_.local_map_min_points,
    filter_target = usesFilteredTarget(config_.registration_method),
    ndt_target = usesNdtVoxelGrid(config_.registration_method),
    leaf = static_cast<float>(config_.voxel_leaf_size), center_x, center_y]() mutable {
      stale = RegistrationObjects();
      LocalMapRebuild rebuild;
      rebuild.center_x = center_x;
      rebuild.center_y = center_y;
      Cloud::Ptr local_map = cropLocalMapByRadius(*full_map, center_x, center_y, radius);
      Cloud::Ptr filtered_local_map(new Cloud());
      if (filter_target) {
        pcl::VoxelGrid<PointT> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(leaf, leaf, leaf);
        voxel_grid_filter.setInputCloud(local_map);
        voxel_grid_filter.filter(*filtered_local_map);
      }
      const auto target_selection = chooseTargetAfterLocalMapCrop(
        local_map->size(), filtered_local_map->size(), min_points, filter_target);
      rebuild.ready = target_selection.target_ready;
      if (!rebuild.ready) {
        return rebuild;
      }
      rebuild.target = target_selection.target_cloud == LocalMapTargetCloud::kFilteredLocalMap ?
        filtered_local_map : local_map;
      auto * registration = objects.get();
      setRegistrationTarget<PointT>(*registration, rebuild.target, ndt_target);
      rebuild.warm_up_source = sampleWarmUpSource(*rebuild.target);
      registration->setInputSource(rebuild.warm_up_source);
      Cloud output;
      registration->align(output, Eigen::Matrix4f::Identity());
      return rebuild;
    });
}

bool RegistrationEngine::takeLocalMapRebuild(float center_x, float center_y, bool wait)
{
  if (!local_map_rebuild_.valid()) {
    return false;
  }
  if (!wait &&
    local_map_rebuild_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    return false;
  }
  LocalMapRebuild rebuild;
  try {
    rebuild = local_map_rebuild_.get();
  } catch (const std::exception & error) {
    log(LogLevel::kWarn, formatLog("Background local map rebuild failed: %s", error.what()));
    return false;
  }
  // A target built around a center the pose has since left is not used.
  if (!rebuild.ready ||
    planarDistance(rebuild.center_x, rebuild.center_y, center_x, center_y) >=
    localMapAsyncReuseLimit(
      config_.local_map_radius, config_.scan_max_range, config_.local_map_update_distance))
  {
    return false;
  }
  auto registration_execution_lock = lockExecution();
  std::swap(registration_objects_, spare_registration_objects_);
  // The task freed the stale target before it cropped.
  heap_trim_pending_.store(true, std::memory_order_relaxed);
  registration_ = registration_objects_.get();
  if (registration_source_cloud_) {
    registration_->setInputSource(registration_source_cloud_);
  }
  keepRegistrationCloudAlive(
    recent_source_clouds_, rebuild.warm_up_source, config_.source_cloud_keep_alive_count);
  keepRegistrationCloudAlive(
    recent_target_clouds_, rebuild.target, config_.target_cloud_keep_alive_count);
  local_map_target_cached_ = true;
  local_map_target_center_x_ = rebuild.center_x;
  local_map_target_center_y_ = rebuild.center_y;
  return true;
}

void RegistrationEngine::discardLocalMapRebuild()
{
  if (local_map_rebuild_.valid()) {
    local_map_rebuild_.wait();
    local_map_rebuild_ = std::future<LocalMapRebuild>();
  }
}

void RegistrationEngine::setSource(const Cloud::Ptr & source_cloud)
{
  registration_->setInputSource(source_cloud);
  registration_source_cloud_ = source_cloud;
  keepRegistrationCloudAlive(
    recent_source_clouds_, source_cloud, config_.source_cloud_keep_alive_count);
}

Eigen::Matrix4f RegistrationEngine::refineSeedWithNdtInitializer(
  const Cloud::Ptr & source_cloud, const Eigen::Matrix4f & init_guess)
{
  const auto run_input = NdtInitializerRunInput{
    use_ndt_initializer_,
    static_cast<bool>(ndt_initializer_) && static_cast<bool>(source_cloud),
    ndt_init_scan_count_,
    config_.ndt_init_scans_required};
  if (!shouldRunNdtInitializer(run_input)) {
    return init_guess;
  }

  ndt_initializer_->setInputSource(source_cloud);
  Cloud::Ptr ndt_output(new Cloud);
  ndt_initializer_->align(*ndt_output, init_guess);
  const auto progress_decision = updateNdtInitializerProgress(
    NdtInitializerProgressInput{run_input, ndt_initializer_->hasConverged()});
  if (!progress_decision.should_accept_refined_seed) {
    return init_guess;
  }

  Eigen::Matrix4f refined_seed = ndt_initializer_->getFinalTransformation();
  ndt_init_scan_count_ = progress_decision.next_scan_count;
  log(
    LogLevel::kInfo,
    formatLog(
      "NDT init scan %d/%d fitness=%.3f",
      ndt_init_scan_count_, config_.ndt_init_scans_required,
      ndt_initializer_->getFitnessScore()));
  if (progress_decision.should_reset_initializer) {
    log(
      LogLevel::kInfo,
      formatLog("NDT init complete, switching to %s", config_.registration_method.c_str()));
    ndt_initializer_.reset();
  }
  return refined_seed;
}

RegistrationAlignResult RegistrationEngine::align(const Eigen::Matrix4f & guess)
{
  RegistrationAlignResult result;
  Cloud output_cloud;
  const auto start = std::chrono::steady_clock::now();
  {
    auto registration_execution_lock = lockExecution();
    registration_->align(output_cloud, guess);
  }
  result.alignment_time_sec =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  result.has_converged = registration_->hasConverged();
  result.iterations = registration_objects_.finalIterationCount();
  if (result.has_converged) {
    result.final_transformation = registration_->getFinalTransformation();
  }
  return result;
}

RegistrationFitness RegistrationEngine::fitness(
  double inlier_distance_m, std::size_t inlier_min_points) const
{
  return registrationFitness(*registration_, threads(), inlier_distance_m, inlier_min_points);
}

std::optional<RegistrationLocalizabilityMetrics> RegistrationEngine::localizability() const
{
  if (!registration_objects_.ndt_omp) {
    return std::nullopt;
  }
  Eigen::Matrix<double, 6, 6> hessian;
  std::size_t correspondence_count = 0;
  registration_objects_.ndt_omp->evaluateFinalScoreHessian(hessian, correspondence_count);
  return analyzeRegistrationHessian(hessian, correspondence_count);
}

void RegistrationEngine::resetCropFailureGuard()
{
  consecutive_crop_failures_ = 0;
  crop_failure_guard_active_ = false;
  last_crop_out_of_bounds_log_time_ = std::chrono::steady_clock::time_point{};
  last_crop_failure_streak_log_time_ = std::chrono::steady_clock::time_point{};
}

bool RegistrationEngine::takeHeapTrimPending()
{
  return heap_trim_pending_.exchange(false, std::memory_order_relaxed);
}

void RegistrationEngine::release(bool leak_target_clouds)
{
  discardLocalMapRebuild();
  {
    auto registration_execution_lock = lockExecution();
    ndt_initializer_.reset();
    use_ndt_initializer_ = false;
    registration_ = nullptr;
    registration_objects_ = RegistrationObjects();
    spare_registration_objects_ = RegistrationObjects();
  }
  registration_source_cloud_.reset();
  recent_source_clouds_.clear();
  // NOLINTBEGIN(clang-analyzer-cplusplus.NewDeleteLeaks): the target and full-map
  // clouds are intentionally leaked when leak_target_clouds is set. NDT_OMP can
  // still hold shutdown-path ownership relationships to them, so freeing them
  // during process teardown crashes.
  if (leak_target_clouds) {
    auto * leaked_target_clouds = new std::deque<Cloud::Ptr>();
    leaked_target_clouds->swap(recent_target_clouds_);
    (void)leaked_target_clouds;
    auto * leaked_full_map_cloud = new Cloud::Ptr(std::move(full_map_cloud_ptr_));
    (void)leaked_full_map_cloud;
  } else {
    recent_target_clouds_.clear();
    full_map_cloud_ptr_.reset();
  }
  map_bounds_ = MapBounds3d{};
  consecutive_crop_failures_ = 0;
  crop_failure_guard_active_ = false;
  last_crop_out_of_bounds_log_time_ = std::chrono::steady_clock::time_point{};
  last_crop_failure_streak_log_time_ = std::chrono::steady_clock::time_point{};
  // NOLINTEND(clang-analyzer-cplusplus.NewDeleteLeaks)
}

}  // namespace lidar_localization
