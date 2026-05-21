#ifndef LIDAR_LOCALIZATION_LOCAL_MAP_TARGET_POLICY_HPP_
#define LIDAR_LOCALIZATION_LOCAL_MAP_TARGET_POLICY_HPP_

#include <chrono>
#include <cstddef>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_localization
{

struct LocalMapBounds2d
{
  bool valid{false};
  float min_x{0.0f};
  float max_x{0.0f};
  float min_y{0.0f};
  float max_y{0.0f};
};

struct LocalMapCropRequest
{
  float center_x{0.0f};
  float center_y{0.0f};
  double radius_m{0.0};
  std::size_t min_points{1};
  LocalMapBounds2d bounds;
};

enum class LocalMapTargetCloud
{
  kRawLocalMap = 0,
  kFilteredLocalMap,
};

enum class LocalMapTargetFailure
{
  kNone = 0,
  kCropCenterOutsideBounds,
  kCropTooSmall,
};

struct LocalMapCropValidationDecision
{
  bool can_crop{true};
  LocalMapTargetFailure failure{LocalMapTargetFailure::kNone};
};

struct LocalMapTargetSelectionDecision
{
  bool target_ready{true};
  LocalMapTargetFailure failure{LocalMapTargetFailure::kNone};
  LocalMapTargetCloud target_cloud{LocalMapTargetCloud::kRawLocalMap};
};

struct LocalMapTargetFailureHandlingInput
{
  LocalMapTargetFailure failure{LocalMapTargetFailure::kNone};
  int consecutive_crop_failures{0};
  bool has_last_failure_streak_log{false};
  std::chrono::steady_clock::duration elapsed_since_failure_streak_log{
    std::chrono::steady_clock::duration::zero()};
  bool has_last_out_of_bounds_log{false};
  std::chrono::steady_clock::duration elapsed_since_out_of_bounds_log{
    std::chrono::steady_clock::duration::zero()};
  int failure_streak_log_threshold{100};
  std::chrono::steady_clock::duration min_log_interval{std::chrono::seconds(5)};
};

struct LocalMapTargetFailureHandlingDecision
{
  int consecutive_crop_failures{0};
  bool should_log_failure_streak{false};
  bool should_log_out_of_bounds{false};
};

struct LocalMapTargetSuccessDecision
{
  int consecutive_crop_failures{0};
  bool crop_failure_guard_active{false};
};

inline bool isCropCenterOutsideLocalMapBounds(const LocalMapCropRequest & request)
{
  if (!request.bounds.valid) {
    return false;
  }
  const float radius = static_cast<float>(request.radius_m);
  return request.center_x < request.bounds.min_x - radius ||
         request.center_x > request.bounds.max_x + radius ||
         request.center_y < request.bounds.min_y - radius ||
         request.center_y > request.bounds.max_y + radius;
}

inline LocalMapCropValidationDecision validateLocalMapCropRequest(
  const LocalMapCropRequest & request)
{
  if (isCropCenterOutsideLocalMapBounds(request)) {
    return {false, LocalMapTargetFailure::kCropCenterOutsideBounds};
  }
  return {};
}

inline bool isLocalMapCropTooSmall(std::size_t local_map_points, std::size_t min_points)
{
  return local_map_points < min_points;
}

inline LocalMapTargetSelectionDecision validateLocalMapCropSize(
  std::size_t local_map_points,
  std::size_t min_points)
{
  if (isLocalMapCropTooSmall(local_map_points, min_points)) {
    return {false, LocalMapTargetFailure::kCropTooSmall, LocalMapTargetCloud::kRawLocalMap};
  }
  return {};
}

inline std::size_t localMapReserveCapacity(std::size_t full_map_points)
{
  return full_map_points / 10;
}

inline pcl::PointCloud<pcl::PointXYZI>::Ptr cropLocalMapByRadius(
  const pcl::PointCloud<pcl::PointXYZI> & full_map,
  float center_x,
  float center_y,
  double radius_m)
{
  const float radius_squared = static_cast<float>(radius_m * radius_m);
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map(new pcl::PointCloud<pcl::PointXYZI>());
  local_map->reserve(localMapReserveCapacity(full_map.size()));
  for (const auto & point : full_map.points) {
    const float dx = point.x - center_x;
    const float dy = point.y - center_y;
    if (dx * dx + dy * dy <= radius_squared) {
      local_map->push_back(point);
    }
  }
  return local_map;
}

inline LocalMapTargetCloud chooseLocalMapTargetCloud(
  std::size_t filtered_local_map_points,
  std::size_t min_points)
{
  if (filtered_local_map_points >= min_points) {
    return LocalMapTargetCloud::kFilteredLocalMap;
  }
  return LocalMapTargetCloud::kRawLocalMap;
}

inline LocalMapTargetSelectionDecision chooseTargetAfterLocalMapCrop(
  std::size_t local_map_points,
  std::size_t filtered_local_map_points,
  std::size_t min_points)
{
  const auto crop_size_decision = validateLocalMapCropSize(local_map_points, min_points);
  if (!crop_size_decision.target_ready) {
    return crop_size_decision;
  }
  return {
    true,
    LocalMapTargetFailure::kNone,
    chooseLocalMapTargetCloud(filtered_local_map_points, min_points)};
}

inline LocalMapTargetCloud chooseMapSubscriptionTargetCloud(bool backend_uses_filtered_target)
{
  return backend_uses_filtered_target ?
         LocalMapTargetCloud::kFilteredLocalMap :
         LocalMapTargetCloud::kRawLocalMap;
}

inline int incrementCropFailureCount(int current_count)
{
  if (current_count == std::numeric_limits<int>::max()) {
    return current_count;
  }
  return current_count + 1;
}

inline bool shouldLogCropFailureStreak(
  int consecutive_crop_failures,
  bool has_last_log_time,
  std::chrono::steady_clock::duration elapsed_since_last_log,
  int failure_threshold = 100,
  std::chrono::steady_clock::duration min_log_interval = std::chrono::seconds(5))
{
  if (consecutive_crop_failures <= failure_threshold) {
    return false;
  }
  return !has_last_log_time || elapsed_since_last_log >= min_log_interval;
}

inline bool shouldLogCropOutOfBounds(
  bool has_last_log_time,
  std::chrono::steady_clock::duration elapsed_since_last_log,
  std::chrono::steady_clock::duration min_log_interval = std::chrono::seconds(5))
{
  return !has_last_log_time || elapsed_since_last_log >= min_log_interval;
}

inline LocalMapTargetFailureHandlingDecision handleLocalMapTargetFailure(
  const LocalMapTargetFailureHandlingInput & input)
{
  if (input.failure == LocalMapTargetFailure::kNone) {
    return {input.consecutive_crop_failures, false, false};
  }

  LocalMapTargetFailureHandlingDecision decision;
  decision.consecutive_crop_failures =
    incrementCropFailureCount(input.consecutive_crop_failures);
  decision.should_log_failure_streak =
    shouldLogCropFailureStreak(
    decision.consecutive_crop_failures,
    input.has_last_failure_streak_log,
    input.elapsed_since_failure_streak_log,
    input.failure_streak_log_threshold,
    input.min_log_interval);
  decision.should_log_out_of_bounds =
    input.failure == LocalMapTargetFailure::kCropCenterOutsideBounds &&
    shouldLogCropOutOfBounds(
    input.has_last_out_of_bounds_log,
    input.elapsed_since_out_of_bounds_log,
    input.min_log_interval);
  return decision;
}

inline LocalMapTargetSuccessDecision handleLocalMapTargetSuccess()
{
  return {};
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_LOCAL_MAP_TARGET_POLICY_HPP_
