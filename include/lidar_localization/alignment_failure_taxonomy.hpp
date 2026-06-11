#ifndef LIDAR_LOCALIZATION_ALIGNMENT_FAILURE_TAXONOMY_HPP_
#define LIDAR_LOCALIZATION_ALIGNMENT_FAILURE_TAXONOMY_HPP_

#include <cmath>
#include <cstddef>
#include <string>

namespace lidar_localization
{

// Primary failure categories, ordered by diagnosis priority. The category names
// distinguish bad match, missing map/initial pose, weak overlap, stale prediction,
// and overload so downstream consumers can branch on a single key instead of
// reverse-engineering the status message.
inline constexpr const char * kAlignmentFailureCategoryMissingMap = "missing_map";
inline constexpr const char * kAlignmentFailureCategoryMissingInitialPose =
  "missing_initial_pose";
inline constexpr const char * kAlignmentFailureCategoryWeakOverlap = "weak_overlap";
inline constexpr const char * kAlignmentFailureCategoryBadMatch = "bad_match";
inline constexpr const char * kAlignmentFailureCategoryStalePrediction =
  "stale_prediction";
inline constexpr const char * kAlignmentFailureCategoryOverload = "overload";
inline constexpr const char * kAlignmentFailureCategoryHealthy = "healthy";

struct AlignmentFailureTaxonomyParams
{
  // Scan-side overlap proxy: fewer filtered points than this cannot constrain
  // the registration. Mirrors the local_map_min_points default.
  std::size_t weak_overlap_min_filtered_points{100};
  // Prediction is considered stale once no measurement has been accepted for
  // this long; the pose is coasting on prediction alone.
  double stale_prediction_min_gap_sec{2.0};
  // Alignment slower than this is falling behind the scan stream. Non-positive
  // disables the overload check.
  double overload_alignment_time_sec{0.3};
};

struct AlignmentFailureTaxonomyInput
{
  bool map_received{false};
  bool initialpose_received{false};
  std::string status_message;
  bool has_converged{false};
  double fitness_score{NAN};
  double effective_score_threshold{NAN};
  std::size_t filtered_point_count{0};
  double alignment_time_sec{0.0};
  double accepted_gap_sec{NAN};
};

struct AlignmentFailureTaxonomyResult
{
  std::string category{kAlignmentFailureCategoryHealthy};
  bool weak_overlap_active{false};
  bool bad_match_active{false};
  bool stale_prediction_active{false};
  bool overload_active{false};
};

inline AlignmentFailureTaxonomyResult classifyAlignmentFailure(
  const AlignmentFailureTaxonomyParams & params,
  const AlignmentFailureTaxonomyInput & input)
{
  AlignmentFailureTaxonomyResult result;

  // Secondary flags are computed independently so co-occurring conditions stay
  // visible even when a higher-priority category wins.
  result.weak_overlap_active =
    input.filtered_point_count < params.weak_overlap_min_filtered_points ||
    input.status_message == "local_map_crop_too_small";
  result.bad_match_active =
    !input.has_converged ||
    (std::isfinite(input.fitness_score) &&
    std::isfinite(input.effective_score_threshold) &&
    input.fitness_score > input.effective_score_threshold);
  result.stale_prediction_active =
    std::isfinite(input.accepted_gap_sec) &&
    input.accepted_gap_sec >= params.stale_prediction_min_gap_sec;
  result.overload_active =
    params.overload_alignment_time_sec > 0.0 &&
    input.alignment_time_sec >= params.overload_alignment_time_sec;

  if (!input.map_received) {
    result.category = kAlignmentFailureCategoryMissingMap;
  } else if (!input.initialpose_received) {
    result.category = kAlignmentFailureCategoryMissingInitialPose;
  } else if (result.weak_overlap_active) {
    result.category = kAlignmentFailureCategoryWeakOverlap;
  } else if (result.bad_match_active) {
    result.category = kAlignmentFailureCategoryBadMatch;
  } else if (result.stale_prediction_active) {
    result.category = kAlignmentFailureCategoryStalePrediction;
  } else if (result.overload_active) {
    result.category = kAlignmentFailureCategoryOverload;
  }
  return result;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_ALIGNMENT_FAILURE_TAXONOMY_HPP_
