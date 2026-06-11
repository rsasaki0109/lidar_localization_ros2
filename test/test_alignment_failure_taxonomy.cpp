#include "lidar_localization/alignment_failure_taxonomy.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <string>

namespace ll = lidar_localization;

ll::AlignmentFailureTaxonomyInput healthy_input()
{
  ll::AlignmentFailureTaxonomyInput input;
  input.map_received = true;
  input.initialpose_received = true;
  input.status_message = "ok";
  input.has_converged = true;
  input.fitness_score = 1.0;
  input.effective_score_threshold = 6.0;
  input.filtered_point_count = 1500;
  input.alignment_time_sec = 0.05;
  input.accepted_gap_sec = 0.1;
  return input;
}

void test_healthy_input_classifies_healthy()
{
  const auto result =
    ll::classifyAlignmentFailure(ll::AlignmentFailureTaxonomyParams{}, healthy_input());
  assert(result.category == ll::kAlignmentFailureCategoryHealthy);
  assert(!result.weak_overlap_active);
  assert(!result.bad_match_active);
  assert(!result.stale_prediction_active);
  assert(!result.overload_active);
}

void test_missing_map_outranks_everything()
{
  auto input = healthy_input();
  input.map_received = false;
  input.initialpose_received = false;
  input.has_converged = false;
  input.filtered_point_count = 0;

  const auto result =
    ll::classifyAlignmentFailure(ll::AlignmentFailureTaxonomyParams{}, input);
  assert(result.category == ll::kAlignmentFailureCategoryMissingMap);
  assert(result.weak_overlap_active);
  assert(result.bad_match_active);
}

void test_missing_initial_pose_when_map_present()
{
  auto input = healthy_input();
  input.initialpose_received = false;

  const auto result =
    ll::classifyAlignmentFailure(ll::AlignmentFailureTaxonomyParams{}, input);
  assert(result.category == ll::kAlignmentFailureCategoryMissingInitialPose);
}

void test_weak_overlap_from_low_point_count_and_crop_message()
{
  auto input = healthy_input();
  input.filtered_point_count = 99;

  auto result =
    ll::classifyAlignmentFailure(ll::AlignmentFailureTaxonomyParams{}, input);
  assert(result.category == ll::kAlignmentFailureCategoryWeakOverlap);
  assert(result.weak_overlap_active);

  input = healthy_input();
  input.status_message = "local_map_crop_too_small";
  result = ll::classifyAlignmentFailure(ll::AlignmentFailureTaxonomyParams{}, input);
  assert(result.category == ll::kAlignmentFailureCategoryWeakOverlap);

  // At exactly the minimum the scan still counts as constraining.
  input = healthy_input();
  input.filtered_point_count = 100;
  result = ll::classifyAlignmentFailure(ll::AlignmentFailureTaxonomyParams{}, input);
  assert(!result.weak_overlap_active);
}

void test_bad_match_from_over_threshold_or_no_convergence()
{
  auto input = healthy_input();
  input.fitness_score = 6.5;

  auto result =
    ll::classifyAlignmentFailure(ll::AlignmentFailureTaxonomyParams{}, input);
  assert(result.category == ll::kAlignmentFailureCategoryBadMatch);
  assert(result.bad_match_active);

  // Exactly at the threshold is not a bad match.
  input.fitness_score = 6.0;
  result = ll::classifyAlignmentFailure(ll::AlignmentFailureTaxonomyParams{}, input);
  assert(!result.bad_match_active);

  input = healthy_input();
  input.has_converged = false;
  result = ll::classifyAlignmentFailure(ll::AlignmentFailureTaxonomyParams{}, input);
  assert(result.category == ll::kAlignmentFailureCategoryBadMatch);

  // NaN fitness (no alignment ran) does not flag over-threshold by itself.
  input = healthy_input();
  input.fitness_score = std::numeric_limits<double>::quiet_NaN();
  result = ll::classifyAlignmentFailure(ll::AlignmentFailureTaxonomyParams{}, input);
  assert(!result.bad_match_active);
}

void test_weak_overlap_outranks_bad_match()
{
  auto input = healthy_input();
  input.filtered_point_count = 10;
  input.has_converged = false;
  input.fitness_score = 50.0;

  const auto result =
    ll::classifyAlignmentFailure(ll::AlignmentFailureTaxonomyParams{}, input);
  assert(result.category == ll::kAlignmentFailureCategoryWeakOverlap);
  assert(result.bad_match_active);
}

void test_stale_prediction_and_nan_gap()
{
  auto input = healthy_input();
  input.accepted_gap_sec = 2.0;

  auto result =
    ll::classifyAlignmentFailure(ll::AlignmentFailureTaxonomyParams{}, input);
  assert(result.category == ll::kAlignmentFailureCategoryStalePrediction);
  assert(result.stale_prediction_active);

  input.accepted_gap_sec = std::numeric_limits<double>::quiet_NaN();
  result = ll::classifyAlignmentFailure(ll::AlignmentFailureTaxonomyParams{}, input);
  assert(!result.stale_prediction_active);
  assert(result.category == ll::kAlignmentFailureCategoryHealthy);
}

void test_overload_and_disable_switch()
{
  auto input = healthy_input();
  input.alignment_time_sec = 0.5;

  auto result =
    ll::classifyAlignmentFailure(ll::AlignmentFailureTaxonomyParams{}, input);
  assert(result.category == ll::kAlignmentFailureCategoryOverload);
  assert(result.overload_active);

  ll::AlignmentFailureTaxonomyParams disabled;
  disabled.overload_alignment_time_sec = 0.0;
  result = ll::classifyAlignmentFailure(disabled, input);
  assert(!result.overload_active);
  assert(result.category == ll::kAlignmentFailureCategoryHealthy);
}

void test_bad_match_outranks_stale_and_overload()
{
  auto input = healthy_input();
  input.fitness_score = 9.0;
  input.accepted_gap_sec = 10.0;
  input.alignment_time_sec = 1.0;

  const auto result =
    ll::classifyAlignmentFailure(ll::AlignmentFailureTaxonomyParams{}, input);
  assert(result.category == ll::kAlignmentFailureCategoryBadMatch);
  assert(result.bad_match_active);
  assert(result.stale_prediction_active);
  assert(result.overload_active);
}

int main()
{
  test_healthy_input_classifies_healthy();
  test_missing_map_outranks_everything();
  test_missing_initial_pose_when_map_present();
  test_weak_overlap_from_low_point_count_and_crop_message();
  test_bad_match_from_over_threshold_or_no_convergence();
  test_weak_overlap_outranks_bad_match();
  test_stale_prediction_and_nan_gap();
  test_overload_and_disable_switch();
  test_bad_match_outranks_stale_and_overload();
  return 0;
}
