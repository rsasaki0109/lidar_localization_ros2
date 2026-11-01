#include "lidar_localization/imu_pose_history_deskew.hpp"

#include <cassert>
#include <cmath>
#include <vector>

namespace ll = lidar_localization;

ll::TimestampedPose pose(double stamp, float x, float yaw)
{
  ll::TimestampedPose sample;
  sample.stamp_sec = stamp;
  sample.pose(0, 3) = x;
  sample.pose.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  return sample;
}

void test_coverage_requires_both_boundaries()
{
  const std::vector<ll::TimestampedPose> history{pose(10.0, 0.0f, 0.0f), pose(10.2, 0.0f, 0.0f)};
  auto coverage = ll::evaluatePoseHistoryCoverage(history, 10.0, 10.2);
  assert(coverage.full_coverage);
  assert(std::abs(coverage.ratio - 1.0) < 1e-12);

  coverage = ll::evaluatePoseHistoryCoverage(history, 9.9, 10.2);
  assert(!coverage.full_coverage);
  assert(coverage.status == ll::PoseHistoryDeskewStatus::kHistoryStartsTooLate);
  assert(coverage.ratio > 0.66 && coverage.ratio < 0.67);

  coverage = ll::evaluatePoseHistoryCoverage(history, 10.0, 10.3);
  assert(!coverage.full_coverage);
  assert(coverage.status == ll::PoseHistoryDeskewStatus::kHistoryEndsTooEarly);
}

void test_piecewise_pose_interpolation()
{
  const std::vector<ll::TimestampedPose> history{
    pose(10.0, 0.0f, 0.0f), pose(10.1, 1.0f, 0.0f), pose(10.2, 1.0f, static_cast<float>(M_PI))};
  Eigen::Matrix4f interpolated;
  assert(ll::interpolatePoseHistory(history, 10.05, &interpolated));
  assert(std::abs(interpolated(0, 3) - 0.5f) < 1e-5f);
  assert(ll::interpolatePoseHistory(history, 10.15, &interpolated));
  assert(std::abs(interpolated(0, 3) - 1.0f) < 1e-5f);
  const Eigen::Vector3f rotated = interpolated.block<3, 3>(0, 0) * Eigen::Vector3f::UnitX();
  assert(std::abs(rotated.x()) < 1e-4f);
  assert(std::abs(std::abs(rotated.y()) - 1.0f) < 1e-4f);
}

void test_deskew_maps_each_point_to_reference_pose()
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.resize(3);
  for (auto & point : cloud) {
    point.x = 1.0f;
    point.y = 0.0f;
    point.z = 0.0f;
  }
  const std::vector<double> times{0.0, 0.1, 0.2};
  const std::vector<ll::TimestampedPose> history{
    pose(10.0, 0.0f, 0.0f), pose(10.1, 1.0f, 0.0f), pose(10.2, 2.0f, 0.0f)};
  const auto result = ll::deskewPointCloudWithPoseHistory(
    cloud, times, 10.0, 0.2, history, 0.0);
  assert(result.applied);
  assert(result.status == ll::PoseHistoryDeskewStatus::kApplied);
  assert(result.deskewed_point_count == 3);
  assert(std::abs(result.cloud[0].x - 1.0f) < 1e-5f);
  assert(std::abs(result.cloud[1].x - 2.0f) < 1e-5f);
  assert(std::abs(result.cloud[2].x - 3.0f) < 1e-5f);
}

void test_deskew_rejects_partial_coverage_without_mutating_cloud()
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.resize(1);
  cloud[0].x = 4.0f;
  const auto result = ll::deskewPointCloudWithPoseHistory(
    cloud, {0.0}, 10.0, 0.2,
    {pose(10.0, 0.0f, 0.0f), pose(10.1, 1.0f, 0.0f)}, 0.0);
  assert(!result.applied);
  assert(result.status == ll::PoseHistoryDeskewStatus::kHistoryEndsTooEarly);
  assert(std::abs(result.coverage_ratio - 0.5) < 1e-12);
  assert(result.cloud[0].x == 4.0f);
}

int main()
{
  test_coverage_requires_both_boundaries();
  test_piecewise_pose_interpolation();
  test_deskew_maps_each_point_to_reference_pose();
  test_deskew_rejects_partial_coverage_without_mutating_cloud();
  return 0;
}
