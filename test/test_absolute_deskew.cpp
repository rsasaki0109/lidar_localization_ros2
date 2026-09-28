#include <lidar_localization/lidar_localization_component.hpp>
#include <cassert>
#include <cmath>
#include <limits>

void check(PCLLocalization & node, const char * reference, double first,
  double stamp, double offset, float expected, bool apply = true)
{
  node.continuous_time_cloud_stamp_reference_ = reference;
  node.continuous_time_deskew_reference_time_sec_ = offset;
  PCLLocalization::PreparedScanCloud scan;
  scan.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  scan.point_time_reference_sec = first;
  scan.relative_times_sec = {0.0, 0.05, 0.1};
  scan.relative_times_aligned_with_cloud = true;
  for (double t : scan.relative_times_sec) {
    pcl::PointXYZI point;
    point.x = static_cast<float>(5.0 - t);
    point.y = 2; point.z = 1; point.intensity = 7;
    scan.cloud->push_back(point);
  }
  node.latest_scan_time_status_ = lidar_localization::ScanTimeRangeStatus::kReady;
  node.latest_scan_time_duration_sec_ = 0.1;
  assert(node.applyContinuousTimeDeskewIfEnabled(scan, stamp) == apply);
  for (std::size_t i = 0; i < scan.cloud->size(); ++i) {
    const auto & point = (*scan.cloud)[i];
    const float want = apply ? expected : static_cast<float>(5.0 - scan.relative_times_sec[i]);
    assert(std::abs(point.x - want) < 1e-5f);
    assert(point.y == 2 && point.z == 1 && point.intensity == 7);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  PCLLocalization node{rclcpp::NodeOptions()};
  node.scan_period_ = 0.1;
  node.use_continuous_time_deskew_ = true;
  node.continuous_time_deskew_mode_ = "lidar_constant_velocity";
  Eigen::Matrix4f motion = Eigen::Matrix4f::Identity();
  motion(0, 3) = .1f;
  node.resetPredictionState(Eigen::Matrix4f::Identity(), 10.0);
  node.updatePredictionState(motion, 10.1);
  check(node, "start", 0, 100, 0, 5);
  check(node, "end", 0, 100.1, 0, 5);
  check(node, "start", 0, 100, .02, 4.98f);
  check(node, "absolute", 100, 100.05, 0, 4.95f);
  check(node, "absolute", 100, 100, 0, 5);
  check(node, "absolute", 100, 100.1, 0, 4.9f);
  check(node, "absolute", 100, 100.05, .01, 4.94f);
  check(node, "absolute", 0, 100.05, 0, 0, false);
  check(node, "absolute", 100, 99.9, 0, 0, false);
  check(node, "absolute", 100, 100.05, .1, 0, false);
  check(node, "absolute", std::numeric_limits<double>::quiet_NaN(), 100, 0, 0, false);
  node.use_continuous_time_deskew_ = false;
  check(node, "absolute", 100, 100.05, 0, 0, false);
  node.use_continuous_time_deskew_ = true;
  node.use_imu_preintegration_ = true;
  node.continuous_time_deskew_mode_ = "imu_pose_history";
  for (int i : {0, 1}) {
    lidar_localization::TimestampedPose sample;
    sample.stamp_sec = 100 + .1 * i;
    sample.pose = Eigen::Matrix4f::Identity();
    sample.pose(0, 3) = .1f * i;
    node.continuous_time_imu_pose_history_.push_back(sample);
  }
  check(node, "absolute", 100, 100.05, 0, 4.95f);
  check(node, "start", 0, 100, 0, 5);
  check(node, "end", 0, 100.1, 0, 5);
  rclcpp::shutdown();
}
