#include "lidar_localization/imu_queue_policy.hpp"

#include <cassert>
#include <cmath>
#include <vector>

namespace ll = lidar_localization;

ll::TimestampedImuSample sample(double stamp, double accel_x)
{
  return {stamp, Eigen::Vector3d::Zero(), Eigen::Vector3d(accel_x, 0.0, 0.0)};
}

void test_plan_stops_exactly_at_scan_time_without_future_sample()
{
  const std::vector<ll::TimestampedImuSample> samples{
    sample(0.0, 1.0), sample(0.05, 2.0), sample(0.10, 1000.0)};

  const auto plan = ll::planCausalImuIntervals(0.0, 0.075, samples, 0.06);

  assert(plan.input_order_valid);
  assert(plan.complete_coverage);
  assert(plan.intervals.size() == 2);
  assert(std::abs(plan.intervals[0].dt() - 0.05) < 1e-12);
  assert(plan.intervals[0].accel.x() == 1.0);
  assert(std::abs(plan.intervals[1].dt() - 0.025) < 1e-12);
  assert(plan.intervals[1].accel.x() == 2.0);
  assert(std::abs(plan.covered_duration_sec - 0.075) < 1e-12);
}

void test_sample_exactly_on_boundary_updates_only_the_next_interval()
{
  const std::vector<ll::TimestampedImuSample> samples{
    sample(1.0, 1.0), sample(1.1, 2.0), sample(1.2, 3.0)};

  const auto first = ll::planCausalImuIntervals(1.0, 1.1, samples, 0.11);
  assert(first.complete_coverage);
  assert(first.intervals.size() == 1);
  assert(first.intervals[0].accel.x() == 1.0);

  const auto second = ll::planCausalImuIntervals(1.1, 1.2, samples, 0.11);
  assert(second.complete_coverage);
  assert(second.intervals.size() == 1);
  assert(second.intervals[0].accel.x() == 2.0);
}

void test_missing_anchor_and_large_gap_fail_closed()
{
  const auto no_anchor = ll::planCausalImuIntervals(
    0.0, 0.1, {sample(0.05, 1.0), sample(0.1, 1.0)}, 0.06);
  assert(!no_anchor.complete_coverage);
  assert(std::abs(no_anchor.covered_duration_sec - 0.05) < 1e-12);

  const auto gap = ll::planCausalImuIntervals(
    0.0, 0.2, {sample(0.0, 1.0), sample(0.2, 2.0)}, 0.05);
  assert(!gap.complete_coverage);
  assert(gap.skipped_gap_count == 1);
  assert(gap.intervals.empty());
}

void test_non_monotonic_or_non_finite_input_is_rejected()
{
  const auto duplicate = ll::planCausalImuIntervals(
    0.0, 0.1, {sample(0.0, 1.0), sample(0.0, 2.0)}, 0.1);
  assert(!duplicate.input_order_valid);
  assert(!duplicate.complete_coverage);

  auto invalid = sample(0.1, 1.0);
  invalid.gyro.x() = std::numeric_limits<double>::quiet_NaN();
  const auto non_finite = ll::planCausalImuIntervals(
    0.0, 0.1, {sample(0.0, 1.0), invalid}, 0.1);
  assert(!non_finite.input_order_valid);
}

void test_dual_queue_keeps_prediction_tail_for_repropagation()
{
  ll::DualImuQueue queue;
  assert(queue.push(sample(0.0, 1.0)));
  assert(queue.push(sample(0.05, 2.0)));
  assert(queue.push(sample(0.10, 3.0)));
  assert(!queue.push(sample(0.10, 4.0)));

  const auto optimization = queue.consumeOptimizationThrough(0.06);
  assert(optimization.size() == 2);
  assert(queue.optimizationSize() == 1);
  assert(queue.predictionSize() == 3);
  assert(queue.optimizationSamplesThrough(0.06).size() == 0);

  const auto reprop = queue.preparePredictionRepropagation(0.06);
  assert(reprop.anchor_sample.has_value());
  assert(reprop.anchor_sample->stamp_sec == 0.05);
  assert(reprop.samples_after_correction.size() == 1);
  assert(reprop.samples_after_correction.front().stamp_sec == 0.10);
  assert(queue.predictionSize() == 1);
  assert(queue.predictionSamples().size() == 1);

  std::vector<ll::TimestampedImuSample> causal_samples{*reprop.anchor_sample};
  causal_samples.insert(
    causal_samples.end(), reprop.samples_after_correction.begin(),
    reprop.samples_after_correction.end());
  const auto plan = ll::planCausalImuIntervals(0.06, 0.10, causal_samples, 0.06);
  assert(plan.complete_coverage);
  assert(plan.intervals.size() == 1);
  assert(plan.intervals[0].accel.x() == 2.0);

  assert(queue.push(sample(0.15, 4.0)));
  assert(queue.push(sample(0.20, 5.0)));
  assert(queue.trimTo(1) == 2);
  assert(queue.optimizationSize() == 1);
  assert(queue.predictionSize() == 1);
}

int main()
{
  test_plan_stops_exactly_at_scan_time_without_future_sample();
  test_sample_exactly_on_boundary_updates_only_the_next_interval();
  test_missing_anchor_and_large_gap_fail_closed();
  test_non_monotonic_or_non_finite_input_is_rejected();
  test_dual_queue_keeps_prediction_tail_for_repropagation();
  return 0;
}
