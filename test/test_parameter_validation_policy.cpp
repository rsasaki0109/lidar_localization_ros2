#include "lidar_localization/parameter_validation_policy.hpp"

#include <cassert>
#include <chrono>
#include <limits>

namespace ll = lidar_localization;

void test_positive_int_parameter_keeps_valid_values()
{
  const auto value = ll::normalizePositiveIntParameter(3);
  assert(value.value == 3);
  assert(!value.was_adjusted);
}

void test_positive_int_parameter_uses_valid_fallback()
{
  const auto zero = ll::normalizePositiveIntParameter(0, 4);
  assert(zero.value == 4);
  assert(zero.was_adjusted);

  const auto negative_fallback = ll::normalizePositiveIntParameter(-2, -10);
  assert(negative_fallback.value == ll::kMinimumPositiveIntParameter);
  assert(negative_fallback.was_adjusted);
}

void test_local_map_min_points_clamps_to_one()
{
  const auto valid = ll::normalizeLocalMapMinPoints(100);
  assert(valid.value == 100);
  assert(!valid.was_adjusted);

  const auto invalid = ll::normalizeLocalMapMinPoints(0);
  assert(invalid.value == 1);
  assert(invalid.was_adjusted);
}

void test_positive_finite_double_parameter_rejects_invalid_values()
{
  const auto valid = ll::normalizePositiveFiniteDoubleParameter(20.0, 10.0);
  assert(valid.value == 20.0);
  assert(!valid.was_adjusted);

  const auto zero = ll::normalizePositiveFiniteDoubleParameter(0.0, 5.0);
  assert(zero.value == 5.0);
  assert(zero.was_adjusted);

  const auto nan = ll::normalizePositiveFiniteDoubleParameter(
    std::numeric_limits<double>::quiet_NaN(), 5.0);
  assert(nan.value == 5.0);
  assert(nan.was_adjusted);

  const auto invalid_fallback = ll::normalizePositiveFiniteDoubleParameter(
    -1.0, std::numeric_limits<double>::infinity());
  assert(invalid_fallback.value == ll::kDefaultPosePublishFrequencyHz);
  assert(invalid_fallback.was_adjusted);
}

void test_pose_publish_period_uses_safe_frequency()
{
  const auto period_20hz = ll::posePublishPeriodFromFrequencyHz(20.0);
  assert(period_20hz == std::chrono::nanoseconds(50000000));

  const auto fallback_period = ll::posePublishPeriodFromFrequencyHz(0.0);
  assert(fallback_period == std::chrono::nanoseconds(100000000));
}

int main()
{
  test_positive_int_parameter_keeps_valid_values();
  test_positive_int_parameter_uses_valid_fallback();
  test_local_map_min_points_clamps_to_one();
  test_positive_finite_double_parameter_rejects_invalid_values();
  test_pose_publish_period_uses_safe_frequency();
  return 0;
}
