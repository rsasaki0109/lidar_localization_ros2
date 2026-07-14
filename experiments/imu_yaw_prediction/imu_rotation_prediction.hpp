#ifndef LIDAR_LOCALIZATION_EXPERIMENT_IMU_ROTATION_PREDICTION_HPP_
#define LIDAR_LOCALIZATION_EXPERIMENT_IMU_ROTATION_PREDICTION_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iterator>
#include <vector>

namespace lidar_localization
{

enum class ImuRotationPredictionStatus
{
  kReady = 0,
  kDisabled,
  kInvalidInterval,
  kDurationTooLarge,
  kInsufficientCoverage,
  kSampleGapTooLarge,
  kNonFiniteSample,
};

inline const char * imuRotationPredictionStatusName(ImuRotationPredictionStatus status)
{
  switch (status) {
    case ImuRotationPredictionStatus::kReady: return "imu_rotation_prediction_ready";
    case ImuRotationPredictionStatus::kDisabled: return "imu_rotation_prediction_disabled";
    case ImuRotationPredictionStatus::kInvalidInterval:
      return "imu_rotation_prediction_invalid_interval";
    case ImuRotationPredictionStatus::kDurationTooLarge:
      return "imu_rotation_prediction_duration_too_large";
    case ImuRotationPredictionStatus::kInsufficientCoverage:
      return "imu_rotation_prediction_insufficient_coverage";
    case ImuRotationPredictionStatus::kSampleGapTooLarge:
      return "imu_rotation_prediction_sample_gap_too_large";
    case ImuRotationPredictionStatus::kNonFiniteSample:
      return "imu_rotation_prediction_non_finite_sample";
  }
  return "imu_rotation_prediction_unknown";
}

struct ImuRotationSample
{
  double stamp_sec{0.0};
  Eigen::Vector3d angular_velocity_base_rad_s{Eigen::Vector3d::Zero()};
};

struct ImuRotationPredictionResult
{
  ImuRotationPredictionStatus status{ImuRotationPredictionStatus::kInsufficientCoverage};
  Eigen::Matrix3f relative_rotation{Eigen::Matrix3f::Identity()};
  double duration_sec{0.0};
  double max_sample_gap_sec{0.0};
  std::size_t integrated_sample_count{0};

  bool ready() const {return status == ImuRotationPredictionStatus::kReady;}
};

class ImuRotationPredictionBuffer
{
public:
  explicit ImuRotationPredictionBuffer(double history_duration_sec = 2.0)
  : history_duration_sec_(history_duration_sec) {}

  bool addSample(double stamp_sec, const Eigen::Vector3d & angular_velocity_base_rad_s)
  {
    if (!std::isfinite(stamp_sec) || !angular_velocity_base_rad_s.allFinite()) {
      return false;
    }
    if (!samples_.empty() && stamp_sec <= samples_.back().stamp_sec) {
      return false;
    }
    samples_.push_back({stamp_sec, angular_velocity_base_rad_s});
    const double oldest_stamp = stamp_sec - history_duration_sec_;
    while (samples_.size() > 2 && samples_[1].stamp_sec < oldest_stamp) {
      samples_.pop_front();
    }
    return true;
  }

  void clear() {samples_.clear();}
  std::size_t size() const {return samples_.size();}

  ImuRotationPredictionResult predict(
    double start_sec, double end_sec, double max_duration_sec,
    double max_sample_gap_sec) const
  {
    ImuRotationPredictionResult result;
    result.duration_sec = end_sec - start_sec;
    if (!std::isfinite(start_sec) || !std::isfinite(end_sec) || end_sec <= start_sec) {
      result.status = ImuRotationPredictionStatus::kInvalidInterval;
      return result;
    }
    if (max_duration_sec <= 0.0 || result.duration_sec > max_duration_sec) {
      result.status = ImuRotationPredictionStatus::kDurationTooLarge;
      return result;
    }
    if (
      samples_.size() < 2 || samples_.front().stamp_sec > start_sec ||
      samples_.back().stamp_sec < end_sec)
    {
      result.status = ImuRotationPredictionStatus::kInsufficientCoverage;
      return result;
    }

    std::vector<ImuRotationSample> window;
    window.reserve(samples_.size() + 2);
    window.push_back({start_sec, interpolate(start_sec)});
    for (const auto & sample : samples_) {
      if (sample.stamp_sec > start_sec && sample.stamp_sec < end_sec) {
        window.push_back(sample);
      }
    }
    window.push_back({end_sec, interpolate(end_sec)});

    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    for (std::size_t index = 1; index < window.size(); ++index) {
      const double dt = window[index].stamp_sec - window[index - 1].stamp_sec;
      result.max_sample_gap_sec = std::max(result.max_sample_gap_sec, dt);
      if (!std::isfinite(dt) || dt <= 0.0 ||
        max_sample_gap_sec <= 0.0 || dt > max_sample_gap_sec)
      {
        result.status = ImuRotationPredictionStatus::kSampleGapTooLarge;
        return result;
      }
      const Eigen::Vector3d mean_rate = 0.5 * (
        window[index - 1].angular_velocity_base_rad_s +
        window[index].angular_velocity_base_rad_s);
      if (!mean_rate.allFinite()) {
        result.status = ImuRotationPredictionStatus::kNonFiniteSample;
        return result;
      }
      const Eigen::Vector3d rotation_vector = mean_rate * dt;
      const double angle = rotation_vector.norm();
      if (angle > 1e-12) {
        rotation = (rotation * Eigen::Quaterniond(
          Eigen::AngleAxisd(angle, rotation_vector / angle))).normalized();
      }
      ++result.integrated_sample_count;
    }
    result.relative_rotation = rotation.toRotationMatrix().cast<float>();
    if (!result.relative_rotation.allFinite()) {
      result.status = ImuRotationPredictionStatus::kNonFiniteSample;
      result.relative_rotation = Eigen::Matrix3f::Identity();
      return result;
    }
    result.status = ImuRotationPredictionStatus::kReady;
    return result;
  }

private:
  Eigen::Vector3d interpolate(double stamp_sec) const
  {
    auto right = std::lower_bound(
      samples_.begin(), samples_.end(), stamp_sec,
      [](const ImuRotationSample & sample, double stamp) {
        return sample.stamp_sec < stamp;
      });
    if (right == samples_.begin()) {
      return right->angular_velocity_base_rad_s;
    }
    if (right == samples_.end()) {
      return samples_.back().angular_velocity_base_rad_s;
    }
    if (right->stamp_sec == stamp_sec) {
      return right->angular_velocity_base_rad_s;
    }
    const auto left = std::prev(right);
    const double fraction =
      (stamp_sec - left->stamp_sec) / (right->stamp_sec - left->stamp_sec);
    return left->angular_velocity_base_rad_s + fraction * (
      right->angular_velocity_base_rad_s - left->angular_velocity_base_rad_s);
  }

  double history_duration_sec_{2.0};
  std::deque<ImuRotationSample> samples_;
};

inline Eigen::Matrix4f applyImuRotationPrediction(
  const Eigen::Matrix4f & accepted_pose,
  const Eigen::Matrix4f & translation_seed,
  const Eigen::Matrix3f & relative_rotation)
{
  Eigen::Matrix4f result = translation_seed;
  result.block<3, 3>(0, 0) =
    accepted_pose.block<3, 3>(0, 0) * relative_rotation;
  return result;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_EXPERIMENT_IMU_ROTATION_PREDICTION_HPP_
