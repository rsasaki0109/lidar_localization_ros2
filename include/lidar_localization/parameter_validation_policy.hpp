#ifndef LIDAR_LOCALIZATION_PARAMETER_VALIDATION_POLICY_HPP_
#define LIDAR_LOCALIZATION_PARAMETER_VALIDATION_POLICY_HPP_

#include <chrono>
#include <cmath>
#include <cstddef>

namespace lidar_localization
{

constexpr int kMinimumPositiveIntParameter = 1;
constexpr double kDefaultPosePublishFrequencyHz = 10.0;

template<typename T>
struct NormalizedParameterValue
{
  T value{};
  bool was_adjusted{false};
};

inline NormalizedParameterValue<int> normalizePositiveIntParameter(
  int requested,
  int fallback = kMinimumPositiveIntParameter)
{
  const int valid_fallback = fallback > 0 ? fallback : kMinimumPositiveIntParameter;
  if (requested > 0) {
    return {requested, false};
  }
  return {valid_fallback, true};
}

inline NormalizedParameterValue<std::size_t> normalizeLocalMapMinPoints(int requested)
{
  const auto normalized = normalizePositiveIntParameter(requested);
  return {static_cast<std::size_t>(normalized.value), normalized.was_adjusted};
}

inline NormalizedParameterValue<double> normalizePositiveFiniteDoubleParameter(
  double requested,
  double fallback)
{
  const double valid_fallback =
    std::isfinite(fallback) && fallback > 0.0 ? fallback : kDefaultPosePublishFrequencyHz;
  if (std::isfinite(requested) && requested > 0.0) {
    return {requested, false};
  }
  return {valid_fallback, true};
}

inline NormalizedParameterValue<double> normalizePosePublishFrequencyHz(double requested)
{
  return normalizePositiveFiniteDoubleParameter(requested, kDefaultPosePublishFrequencyHz);
}

inline std::chrono::nanoseconds posePublishPeriodFromFrequencyHz(double frequency_hz)
{
  const auto normalized = normalizePosePublishFrequencyHz(frequency_hz);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / normalized.value));
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_PARAMETER_VALIDATION_POLICY_HPP_
