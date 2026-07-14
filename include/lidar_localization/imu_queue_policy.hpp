#ifndef LIDAR_LOCALIZATION_IMU_QUEUE_POLICY_HPP_
#define LIDAR_LOCALIZATION_IMU_QUEUE_POLICY_HPP_

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <vector>

namespace lidar_localization
{

struct TimestampedImuSample
{
  double stamp_sec{0.0};
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
};

struct ImuIntegrationInterval
{
  double start_stamp_sec{0.0};
  double end_stamp_sec{0.0};
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};

  double dt() const {return end_stamp_sec - start_stamp_sec;}
};

struct ImuIntegrationPlan
{
  std::vector<ImuIntegrationInterval> intervals;
  bool input_order_valid{true};
  bool complete_coverage{false};
  std::size_t skipped_gap_count{0};
  double covered_duration_sec{0.0};
};

inline bool finiteImuSample(const TimestampedImuSample & sample)
{
  return std::isfinite(sample.stamp_sec) &&
         sample.gyro.allFinite() && sample.accel.allFinite();
}

/// Build causal, zero-order-held IMU intervals over [start, end]. A sample at
/// or before start is required as the held measurement. Samples after end are
/// deliberately ignored, so delayed scan callbacks cannot integrate the future.
inline ImuIntegrationPlan planCausalImuIntervals(
  double start_stamp_sec,
  double end_stamp_sec,
  const std::vector<TimestampedImuSample> & samples,
  double max_interval_sec)
{
  ImuIntegrationPlan plan;
  if (!std::isfinite(start_stamp_sec) || !std::isfinite(end_stamp_sec) ||
    end_stamp_sec < start_stamp_sec || !std::isfinite(max_interval_sec) ||
    max_interval_sec <= 0.0)
  {
    plan.input_order_valid = false;
    return plan;
  }

  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (!finiteImuSample(samples[i]) ||
      (i > 0 && samples[i].stamp_sec <= samples[i - 1].stamp_sec))
    {
      plan.input_order_valid = false;
      return plan;
    }
  }

  std::optional<TimestampedImuSample> active;
  for (const auto & sample : samples) {
    if (sample.stamp_sec <= start_stamp_sec) {
      active = sample;
    } else {
      break;
    }
  }

  double cursor = start_stamp_sec;
  bool coverage_broken = !active.has_value();
  for (const auto & sample : samples) {
    if (sample.stamp_sec <= start_stamp_sec) {
      continue;
    }
    if (sample.stamp_sec > end_stamp_sec) {
      break;
    }
    if (active.has_value()) {
      const double dt = sample.stamp_sec - cursor;
      if (dt > 0.0 && dt < max_interval_sec) {
        plan.intervals.push_back(
          {cursor, sample.stamp_sec, active->gyro, active->accel});
        plan.covered_duration_sec += dt;
      } else if (dt >= max_interval_sec) {
        ++plan.skipped_gap_count;
        coverage_broken = true;
      }
    } else {
      coverage_broken = true;
    }
    active = sample;
    cursor = sample.stamp_sec;
  }

  if (active.has_value() && cursor < end_stamp_sec) {
    const double dt = end_stamp_sec - cursor;
    if (dt < max_interval_sec) {
      plan.intervals.push_back({cursor, end_stamp_sec, active->gyro, active->accel});
      plan.covered_duration_sec += dt;
    } else {
      ++plan.skipped_gap_count;
      coverage_broken = true;
    }
  }

  const double requested_duration = end_stamp_sec - start_stamp_sec;
  plan.complete_coverage =
    plan.input_order_valid && !coverage_broken &&
    std::abs(plan.covered_duration_sec - requested_duration) <= 1e-9;
  return plan;
}

struct ImuRepropagationWindow
{
  std::optional<TimestampedImuSample> anchor_sample;
  std::vector<TimestampedImuSample> samples_after_correction;
};

/// Raw dual queues following the LIO-SAM separation: optimization consumes its
/// own queue through each LiDAR correction, while prediction retains the same
/// samples until that correction and can then repropagate the remaining tail.
class DualImuQueue
{
public:
  bool push(const TimestampedImuSample & sample)
  {
    if (!finiteImuSample(sample) ||
      (last_pushed_stamp_sec_.has_value() &&
      sample.stamp_sec <= *last_pushed_stamp_sec_))
    {
      return false;
    }
    optimization_queue_.push_back(sample);
    prediction_queue_.push_back(sample);
    last_pushed_stamp_sec_ = sample.stamp_sec;
    return true;
  }

  std::vector<TimestampedImuSample> consumeOptimizationThrough(double stamp_sec)
  {
    std::vector<TimestampedImuSample> consumed;
    while (!optimization_queue_.empty() &&
      optimization_queue_.front().stamp_sec <= stamp_sec)
    {
      consumed.push_back(optimization_queue_.front());
      optimization_queue_.pop_front();
    }
    return consumed;
  }

  std::vector<TimestampedImuSample> optimizationSamplesThrough(double stamp_sec) const
  {
    std::vector<TimestampedImuSample> samples;
    for (const auto & sample : optimization_queue_) {
      if (sample.stamp_sec > stamp_sec) break;
      samples.push_back(sample);
    }
    return samples;
  }

  ImuRepropagationWindow preparePredictionRepropagation(double correction_stamp_sec)
  {
    ImuRepropagationWindow window;
    while (!prediction_queue_.empty() &&
      prediction_queue_.front().stamp_sec <= correction_stamp_sec)
    {
      window.anchor_sample = prediction_queue_.front();
      prediction_queue_.pop_front();
    }
    window.samples_after_correction.assign(
      prediction_queue_.begin(), prediction_queue_.end());
    return window;
  }

  std::size_t optimizationSize() const {return optimization_queue_.size();}
  std::size_t predictionSize() const {return prediction_queue_.size();}

  std::vector<TimestampedImuSample> predictionSamples() const
  {
    return {prediction_queue_.begin(), prediction_queue_.end()};
  }

  std::size_t trimTo(std::size_t max_size)
  {
    std::size_t removed = 0;
    while (optimization_queue_.size() > max_size) {
      optimization_queue_.pop_front();
      ++removed;
    }
    while (prediction_queue_.size() > max_size) {
      prediction_queue_.pop_front();
    }
    return removed;
  }

  void clear()
  {
    optimization_queue_.clear();
    prediction_queue_.clear();
    last_pushed_stamp_sec_.reset();
  }

private:
  std::deque<TimestampedImuSample> optimization_queue_;
  std::deque<TimestampedImuSample> prediction_queue_;
  std::optional<double> last_pushed_stamp_sec_;
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_IMU_QUEUE_POLICY_HPP_
