#include <lidar_localization/lidar_localization_component.hpp>
#include <cassert>
#include <cmath>

Eigen::Matrix4f pose(float x)
{
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  result(0, 3) = x;
  return result;
}

void check_deskew(PCLLocalization & node)
{
  PCLLocalization::PreparedScanCloud scan;
  scan.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointXYZI point;
  point.x = 1; point.y = 0; point.z = 0; point.intensity = 7;
  scan.cloud->push_back(point);
  scan.relative_times_sec = {0.1};
  scan.relative_times_aligned_with_cloud = true;
  node.latest_scan_time_status_ = lidar_localization::ScanTimeRangeStatus::kReady;
  node.latest_scan_time_duration_sec_ = 0.1;
  assert(node.applyContinuousTimeDeskewIfEnabled(scan, 20.0));
  assert(std::abs(scan.cloud->front().x - 1.1f) < 1e-5f);
  assert(scan.cloud->front().intensity == 7);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  PCLLocalization node{rclcpp::NodeOptions()};
  node.scan_period_ = 0.1;
  node.use_continuous_time_deskew_ = true;
  node.continuous_time_deskew_mode_ = "lidar_constant_velocity";
  node.continuous_time_deskew_reference_time_sec_ = 0.0;
  for (int rejects : {0, 1, 10}) {
    node.resetPredictionState(pose(0), 10.0);
    assert(node.last_relative_motion_duration_sec_ == 0.0);
    node.updatePredictionState(pose(.1f), 10.1);
    check_deskew(node);
    for (int k = 0; k < rejects; ++k) {
      node.updatePredictionFromRejectedMeasurement(pose(.2f + .1f * k), 10.2 + .1 * k);
    }
    node.updatePredictionState(pose(.2f + .1f * rejects), 10.2 + .1 * rejects);
    check_deskew(node);
    assert(std::abs(node.last_relative_motion_duration_sec_ - .1) < 1e-9);
    node.updatePredictionState(pose(.3f + .1f * rejects), 10.3 + .1 * rejects);
    check_deskew(node);
  }
  node.resetPredictionState(pose(5), 30.0);
  assert(node.last_relative_motion_duration_sec_ == 0.0);
  assert(node.last_relative_motion_matrix_.isIdentity());
  rclcpp::shutdown();
}
