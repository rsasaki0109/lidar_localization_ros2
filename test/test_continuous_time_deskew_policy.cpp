#include "lidar_localization/continuous_time_deskew_policy.hpp"

#include <Eigen/Geometry>

#include <cassert>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace ll = lidar_localization;

constexpr float kHalfPi = 1.5707963267948966f;

bool close(float a, float b, float tolerance = 1.0e-5f)
{
  return std::abs(a - b) < tolerance;
}

pcl::PointXYZI point(float x, float y, float z, float intensity = 1.0f)
{
  pcl::PointXYZI p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = intensity;
  return p;
}

pcl::PointCloud<pcl::PointXYZI> make_cloud(
  const pcl::PointXYZI & first,
  const pcl::PointXYZI & second)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.width = 2;
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.points = {first, second};
  return cloud;
}

Eigen::Matrix4f translation(float x, float y, float z)
{
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix(0, 3) = x;
  matrix(1, 3) = y;
  matrix(2, 3) = z;
  return matrix;
}

Eigen::Matrix4f yaw_rotation(float yaw_rad)
{
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  return matrix;
}

void test_normalized_time_and_interpolation()
{
  assert(std::abs(ll::normalizedScanTime(0.05, 0.1) - 0.5) < 1.0e-12);
  assert(std::isnan(ll::normalizedScanTime(0.05, 0.0)));

  bool clamped = false;
  assert(ll::clampUnitInterval(1.5, &clamped) == 1.0);
  assert(clamped);
  assert(ll::clampUnitInterval(-0.5, &clamped) == 0.0);
  assert(clamped);
  assert(ll::clampUnitInterval(0.5, &clamped) == 0.5);
  assert(!clamped);

  const Eigen::Matrix4f half = ll::interpolateRelativeMotion(translation(2.0f, 0.0f, 0.0f), 0.5);
  assert(close(half(0, 3), 1.0f));
}

ll::ContinuousTimeDeskewDecisionInput ready_decision_input()
{
  ll::ContinuousTimeDeskewDecisionInput input;
  input.enabled = true;
  input.cloud_empty = false;
  input.scan_time_status = ll::ScanTimeRangeStatus::kReady;
  input.relative_times_aligned_with_cloud = true;
  input.relative_time_count = 10;
  input.cloud_point_count = 10;
  input.use_imu_preintegration = true;
  input.imu_preintegration_fallback_mode = false;
  input.smoother_initialized = true;
  input.has_new_imu_samples = true;
  input.prediction_finite = true;
  return input;
}

void test_continuous_time_deskew_decision_ready()
{
  const auto decision = ll::decideContinuousTimeDeskew(ready_decision_input());

  assert(decision.should_apply);
  assert(decision.status == ll::ContinuousTimeDeskewStatus::kReady);
  assert(std::string(ll::continuousTimeDeskewStatusMessage(decision.status)) ==
         "continuous_time_deskew_ready");
}

void test_continuous_time_deskew_decision_blocks_in_order()
{
  auto disabled = ready_decision_input();
  disabled.enabled = false;
  auto decision = ll::decideContinuousTimeDeskew(disabled);
  assert(!decision.should_apply);
  assert(decision.status == ll::ContinuousTimeDeskewStatus::kDisabled);

  auto no_time = ready_decision_input();
  no_time.scan_time_status = ll::ScanTimeRangeStatus::kNoTimeField;
  decision = ll::decideContinuousTimeDeskew(no_time);
  assert(!decision.should_apply);
  assert(decision.status == ll::ContinuousTimeDeskewStatus::kScanTimeNotReady);

  auto not_aligned = ready_decision_input();
  not_aligned.relative_time_count = 9;
  decision = ll::decideContinuousTimeDeskew(not_aligned);
  assert(!decision.should_apply);
  assert(decision.status == ll::ContinuousTimeDeskewStatus::kTimesNotAligned);

  auto imu_off = ready_decision_input();
  imu_off.use_imu_preintegration = false;
  decision = ll::decideContinuousTimeDeskew(imu_off);
  assert(!decision.should_apply);
  assert(decision.status == ll::ContinuousTimeDeskewStatus::kImuPreintegrationDisabled);

  auto waiting_imu = ready_decision_input();
  waiting_imu.has_new_imu_samples = false;
  decision = ll::decideContinuousTimeDeskew(waiting_imu);
  assert(!decision.should_apply);
  assert(decision.status == ll::ContinuousTimeDeskewStatus::kWaitingForNewImu);

  auto non_finite = ready_decision_input();
  non_finite.prediction_finite = false;
  decision = ll::decideContinuousTimeDeskew(non_finite);
  assert(!decision.should_apply);
  assert(decision.status == ll::ContinuousTimeDeskewStatus::kPredictionNonFinite);
}

void test_translation_deskews_points_to_scan_start()
{
  const auto cloud = make_cloud(
    point(10.0f, 0.0f, 0.0f, 3.0f),
    point(9.0f, 0.0f, 0.0f, 4.0f));
  const std::vector<double> times{0.0, 1.0};

  const auto result = ll::deskewPointCloudWithRelativeMotion(
    cloud, times, 1.0, translation(1.0f, 0.0f, 0.0f));

  assert(result.applied);
  assert(result.deskewed_point_count == 2);
  assert(result.skipped_invalid_time_count == 0);
  assert(close(result.cloud[0].x, 10.0f));
  assert(close(result.cloud[1].x, 10.0f));
  assert(close(result.cloud[1].intensity, 4.0f));
}

void test_translation_can_deskew_to_scan_end_reference()
{
  const auto cloud = make_cloud(
    point(10.0f, 0.0f, 0.0f),
    point(9.0f, 0.0f, 0.0f));
  const std::vector<double> times{0.0, 1.0};

  const auto result = ll::deskewPointCloudWithRelativeMotion(
    cloud, times, 1.0, translation(1.0f, 0.0f, 0.0f), 1.0);

  assert(result.applied);
  assert(close(result.cloud[0].x, 9.0f));
  assert(close(result.cloud[1].x, 9.0f));
}

void test_rotation_deskews_points_to_scan_start()
{
  const auto cloud = make_cloud(
    point(1.0f, 0.0f, 0.0f),
    point(0.0f, -1.0f, 0.0f));
  const std::vector<double> times{0.0, 1.0};

  const auto result = ll::deskewPointCloudWithRelativeMotion(
    cloud, times, 1.0, yaw_rotation(kHalfPi));

  assert(result.applied);
  assert(close(result.cloud[0].x, 1.0f));
  assert(close(result.cloud[0].y, 0.0f));
  assert(close(result.cloud[1].x, 1.0f));
  assert(close(result.cloud[1].y, 0.0f));
}

void test_invalid_or_out_of_range_times_are_handled()
{
  const auto cloud = make_cloud(
    point(10.0f, 0.0f, 0.0f),
    point(9.0f, 0.0f, 0.0f));
  const std::vector<double> times{
    std::numeric_limits<double>::quiet_NaN(),
    2.0};

  const auto result = ll::deskewPointCloudWithRelativeMotion(
    cloud, times, 1.0, translation(1.0f, 0.0f, 0.0f));

  assert(result.applied);
  assert(result.deskewed_point_count == 1);
  assert(result.skipped_invalid_time_count == 1);
  assert(result.clamped_time_count == 1);
  assert(close(result.cloud[0].x, 10.0f));
  assert(close(result.cloud[1].x, 10.0f));
}

void test_invalid_inputs_return_unchanged_cloud()
{
  const auto cloud = make_cloud(
    point(1.0f, 0.0f, 0.0f),
    point(2.0f, 0.0f, 0.0f));
  const std::vector<double> too_few_times{0.0};

  const auto result = ll::deskewPointCloudWithRelativeMotion(
    cloud, too_few_times, 1.0, translation(1.0f, 0.0f, 0.0f));

  assert(!result.applied);
  assert(result.cloud.size() == cloud.size());
  assert(close(result.cloud[0].x, 1.0f));
  assert(close(result.cloud[1].x, 2.0f));
}

int main()
{
  test_normalized_time_and_interpolation();
  test_continuous_time_deskew_decision_ready();
  test_continuous_time_deskew_decision_blocks_in_order();
  test_translation_deskews_points_to_scan_start();
  test_translation_can_deskew_to_scan_end_reference();
  test_rotation_deskews_points_to_scan_start();
  test_invalid_or_out_of_range_times_are_handled();
  test_invalid_inputs_return_unchanged_cloud();
  return 0;
}
