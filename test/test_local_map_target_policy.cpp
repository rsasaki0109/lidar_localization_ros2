#include "lidar_localization/local_map_target_policy.hpp"

#include <cassert>
#include <chrono>
#include <limits>

namespace ll = lidar_localization;

pcl::PointXYZI make_point(float x, float y, float z = 0.0f)
{
  pcl::PointXYZI point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = 1.0f;
  return point;
}

void test_crop_bounds_are_optional_and_radius_expanded()
{
  ll::LocalMapCropRequest request;
  request.center_x = 1000.0f;
  request.center_y = 0.0f;
  request.radius_m = 10.0;
  request.bounds.valid = false;
  assert(!ll::isCropCenterOutsideLocalMapBounds(request));

  request.bounds = {true, 0.0f, 100.0f, -50.0f, 50.0f};
  request.center_x = 110.0f;
  assert(!ll::isCropCenterOutsideLocalMapBounds(request));
  request.center_x = 110.1f;
  assert(ll::isCropCenterOutsideLocalMapBounds(request));
  request.center_x = -10.0f;
  assert(!ll::isCropCenterOutsideLocalMapBounds(request));
  request.center_x = -10.1f;
  assert(ll::isCropCenterOutsideLocalMapBounds(request));

  request.center_x = 50.0f;
  request.center_y = 60.1f;
  assert(ll::isCropCenterOutsideLocalMapBounds(request));
}

void test_crop_local_map_by_horizontal_radius()
{
  pcl::PointCloud<pcl::PointXYZI> full_map;
  full_map.push_back(make_point(0.0f, 0.0f));
  full_map.push_back(make_point(3.0f, 4.0f, 100.0f));
  full_map.push_back(make_point(5.1f, 0.0f));
  full_map.push_back(make_point(-3.0f, -4.0f));

  const auto local_map = ll::cropLocalMapByRadius(full_map, 0.0f, 0.0f, 5.0);
  assert(local_map->size() == 3);
  assert(local_map->points[1].z == 100.0f);
}

void test_min_points_and_target_choice()
{
  assert(ll::isLocalMapCropTooSmall(9, 10));
  assert(!ll::isLocalMapCropTooSmall(10, 10));

  assert(ll::chooseLocalMapTargetCloud(10, 10) == ll::LocalMapTargetCloud::kFilteredLocalMap);
  assert(ll::chooseLocalMapTargetCloud(9, 10) == ll::LocalMapTargetCloud::kRawLocalMap);

  const auto too_small = ll::chooseTargetAfterLocalMapCrop(9, 9, 10);
  assert(!too_small.target_ready);
  assert(too_small.failure == ll::LocalMapTargetFailure::kCropTooSmall);
  assert(!ll::validateLocalMapCropSize(9, 10).target_ready);
  assert(ll::validateLocalMapCropSize(10, 10).target_ready);

  const auto filtered_ready = ll::chooseTargetAfterLocalMapCrop(20, 10, 10);
  assert(filtered_ready.target_ready);
  assert(filtered_ready.target_cloud == ll::LocalMapTargetCloud::kFilteredLocalMap);

  const auto raw_fallback = ll::chooseTargetAfterLocalMapCrop(20, 9, 10);
  assert(raw_fallback.target_ready);
  assert(raw_fallback.target_cloud == ll::LocalMapTargetCloud::kRawLocalMap);
}

void test_crop_request_validation()
{
  ll::LocalMapCropRequest request;
  request.center_x = 100.0f;
  request.center_y = 0.0f;
  request.radius_m = 10.0;
  request.bounds = {true, 0.0f, 50.0f, -10.0f, 10.0f};

  const auto invalid = ll::validateLocalMapCropRequest(request);
  assert(!invalid.can_crop);
  assert(invalid.failure == ll::LocalMapTargetFailure::kCropCenterOutsideBounds);

  request.center_x = 60.0f;
  const auto valid = ll::validateLocalMapCropRequest(request);
  assert(valid.can_crop);
  assert(valid.failure == ll::LocalMapTargetFailure::kNone);
}

void test_map_subscription_target_choice()
{
  assert(
    ll::chooseMapSubscriptionTargetCloud(true) == ll::LocalMapTargetCloud::kFilteredLocalMap);
  assert(ll::chooseMapSubscriptionTargetCloud(false) == ll::LocalMapTargetCloud::kRawLocalMap);
}

void test_crop_failure_count_and_log_throttles()
{
  assert(ll::incrementCropFailureCount(0) == 1);
  assert(
    ll::incrementCropFailureCount(std::numeric_limits<int>::max()) ==
    std::numeric_limits<int>::max());

  assert(!ll::shouldLogCropFailureStreak(100, false, std::chrono::seconds(0)));
  assert(ll::shouldLogCropFailureStreak(101, false, std::chrono::seconds(0)));
  assert(!ll::shouldLogCropFailureStreak(101, true, std::chrono::seconds(4)));
  assert(ll::shouldLogCropFailureStreak(101, true, std::chrono::seconds(5)));

  assert(ll::shouldLogCropOutOfBounds(false, std::chrono::seconds(0)));
  assert(!ll::shouldLogCropOutOfBounds(true, std::chrono::seconds(4)));
  assert(ll::shouldLogCropOutOfBounds(true, std::chrono::seconds(5)));
}

void test_target_failure_handling_decision()
{
  ll::LocalMapTargetFailureHandlingInput input;
  input.failure = ll::LocalMapTargetFailure::kCropTooSmall;
  input.consecutive_crop_failures = 100;
  input.has_last_failure_streak_log = false;
  input.has_last_out_of_bounds_log = false;

  const auto too_small = ll::handleLocalMapTargetFailure(input);
  assert(too_small.consecutive_crop_failures == 101);
  assert(too_small.should_log_failure_streak);
  assert(!too_small.should_log_out_of_bounds);

  input.failure = ll::LocalMapTargetFailure::kCropCenterOutsideBounds;
  input.consecutive_crop_failures = 1;
  input.has_last_failure_streak_log = true;
  input.elapsed_since_failure_streak_log = std::chrono::seconds(1);
  input.has_last_out_of_bounds_log = true;
  input.elapsed_since_out_of_bounds_log = std::chrono::seconds(5);

  const auto out_of_bounds = ll::handleLocalMapTargetFailure(input);
  assert(out_of_bounds.consecutive_crop_failures == 2);
  assert(!out_of_bounds.should_log_failure_streak);
  assert(out_of_bounds.should_log_out_of_bounds);

  input.failure = ll::LocalMapTargetFailure::kNone;
  input.consecutive_crop_failures = 7;
  const auto no_failure = ll::handleLocalMapTargetFailure(input);
  assert(no_failure.consecutive_crop_failures == 7);
  assert(!no_failure.should_log_failure_streak);
  assert(!no_failure.should_log_out_of_bounds);
}

void test_target_success_resets_runtime_guard_state()
{
  const auto success = ll::handleLocalMapTargetSuccess();
  assert(success.consecutive_crop_failures == 0);
  assert(!success.crop_failure_guard_active);
}

int main()
{
  test_crop_bounds_are_optional_and_radius_expanded();
  test_crop_local_map_by_horizontal_radius();
  test_min_points_and_target_choice();
  test_crop_request_validation();
  test_map_subscription_target_choice();
  test_crop_failure_count_and_log_throttles();
  test_target_failure_handling_decision();
  test_target_success_resets_runtime_guard_state();
  return 0;
}
