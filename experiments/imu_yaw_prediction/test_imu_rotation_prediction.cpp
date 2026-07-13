#include "imu_rotation_prediction.hpp"

#include <Eigen/Core>

#include <cassert>
#include <cmath>

int main()
{
  using lidar_localization::ImuRotationPredictionBuffer;
  using lidar_localization::ImuRotationPredictionStatus;

  ImuRotationPredictionBuffer buffer;
  for (int index = 0; index <= 100; ++index) {
    assert(buffer.addSample(
      0.01 * index, Eigen::Vector3d(0.0, 0.0, 1.0)));
  }
  const auto prediction = buffer.predict(0.2, 0.8, 1.0, 0.02);
  assert(prediction.ready());
  const Eigen::Matrix3f expected = Eigen::AngleAxisf(
    0.6f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  assert(prediction.relative_rotation.isApprox(expected, 1e-5f));
  assert(prediction.integrated_sample_count >= 60);

  const auto too_long = buffer.predict(0.1, 0.9, 0.5, 0.02);
  assert(too_long.status == ImuRotationPredictionStatus::kDurationTooLarge);
  const auto uncovered = buffer.predict(-0.1, 0.2, 1.0, 0.02);
  assert(uncovered.status == ImuRotationPredictionStatus::kInsufficientCoverage);

  ImuRotationPredictionBuffer gap_buffer;
  assert(gap_buffer.addSample(0.0, Eigen::Vector3d::Zero()));
  assert(gap_buffer.addSample(0.2, Eigen::Vector3d::Zero()));
  const auto gap = gap_buffer.predict(0.0, 0.2, 1.0, 0.05);
  assert(gap.status == ImuRotationPredictionStatus::kSampleGapTooLarge);

  Eigen::Matrix4f accepted = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f translation_seed = Eigen::Matrix4f::Identity();
  translation_seed(0, 3) = 2.5f;
  const Eigen::Matrix4f combined = lidar_localization::applyImuRotationPrediction(
    accepted, translation_seed, expected);
  assert(std::abs(combined(0, 3) - 2.5f) < 1e-6f);
  assert((combined.block<3, 3>(0, 0).isApprox(expected, 1e-5f)));
  return 0;
}
