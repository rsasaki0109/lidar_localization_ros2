// imu_pose_history deskew without IMU preintegration: the IMU callback keeps the
// gyro rotation history on its own, the scan is rotated with it, and the
// preintegration state stays untouched.
#include <lidar_localization/lidar_localization_component.hpp>

#include <cassert>
#include <cmath>
#include <memory>
#include <string>

namespace
{

void feed_gyro(PCLLocalization & node, double from_sec, double to_sec, double yaw_rate)
{
  for (double t = from_sec; t <= to_sec + 1e-9; t += 0.005) {
    auto msg = std::make_shared<sensor_msgs::msg::Imu>();
    msg->header.frame_id = node.base_frame_id_;
    msg->header.stamp.sec = static_cast<int32_t>(std::floor(t));
    msg->header.stamp.nanosec = static_cast<uint32_t>(std::lround((t - std::floor(t)) * 1e9));
    msg->angular_velocity.z = yaw_rate;
    msg->linear_acceleration.z = 9.81;
    node.imuReceived(msg);
  }
}

PCLLocalization::PreparedScanCloud scan_with_point_at(double relative_time_sec)
{
  PCLLocalization::PreparedScanCloud scan;
  scan.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointXYZI point;
  point.x = 10.0f;
  point.y = 0.0f;
  point.z = 0.0f;
  point.intensity = 3.0f;
  scan.cloud->push_back(point);
  scan.relative_times_sec = {relative_time_sec};
  scan.relative_times_aligned_with_cloud = true;
  return scan;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  PCLLocalization node{rclcpp::NodeOptions()};
  node.scan_period_ = 0.1;
  node.use_imu_preintegration_ = false;
  node.use_continuous_time_deskew_ = true;
  node.continuous_time_deskew_mode_ = "imu_pose_history";
  node.continuous_time_cloud_stamp_reference_ = "start";
  node.continuous_time_deskew_reference_time_sec_ = 0.0;
  assert(node.usesGyroPoseHistoryWithoutPreintegration());

  // 90 deg/s about z for 0.3 s at 200 Hz.
  const double yaw_rate = M_PI / 2.0;
  feed_gyro(node, 10.0, 10.3, yaw_rate);
  assert(node.continuous_time_imu_pose_history_.size() > 2);
  // The preintegration state is left alone.
  assert(node.last_imu_stamp_ == 0.0);
  assert(node.imu_dual_queue_.predictionSamples().empty());
  assert(node.imu_preintegration_received_sample_count_since_scan_ == 0);

  // A point taken 0.1 s after the sweep start at 10.1 s: the sensor has turned 9 deg
  // since, so in the start frame the point lies 9 deg further round.
  auto scan = scan_with_point_at(0.1);
  node.latest_scan_time_status_ = lidar_localization::ScanTimeRangeStatus::kReady;
  node.latest_scan_time_duration_sec_ = 0.1;
  assert(node.applyContinuousTimeDeskewIfEnabled(scan, 10.1));
  assert(
    node.latest_continuous_time_deskew_status_ ==
    std::string("continuous_time_deskew_pose_history_applied"));
  const double angle = yaw_rate * 0.1;
  assert(std::abs(scan.cloud->front().x - 10.0 * std::cos(angle)) < 1e-3);
  assert(std::abs(scan.cloud->front().y - 10.0 * std::sin(angle)) < 1e-3);
  assert(scan.cloud->front().intensity == 3.0f);

  // A point at the sweep start does not move.
  auto at_start = scan_with_point_at(0.0);
  assert(node.applyContinuousTimeDeskewIfEnabled(at_start, 10.1));
  assert(std::abs(at_start.cloud->front().x - 10.0f) < 1e-4f);
  assert(std::abs(at_start.cloud->front().y) < 1e-4f);

  // A sweep ending 50 ms after the last IMU sample (10.3 s): rejected by default,
  // deskewed with a 60 ms end gap, the late point holding the last rotation.
  auto late = scan_with_point_at(0.1);
  node.latest_scan_time_duration_sec_ = 0.1;
  assert(!node.applyContinuousTimeDeskewIfEnabled(late, 10.25));
  assert(
    node.latest_continuous_time_deskew_status_ ==
    std::string("continuous_time_deskew_pose_history_ends_too_early"));
  node.continuous_time_pose_history_max_end_gap_sec_ = 0.06;
  assert(node.applyContinuousTimeDeskewIfEnabled(late, 10.25));
  assert(node.latest_continuous_time_deskew_clamped_time_count_ == 1);
  const double held = yaw_rate * 0.05;  // 10.25 s to the last sample at 10.3 s
  assert(std::abs(late.cloud->front().x - 10.0 * std::cos(held)) < 1e-3);
  assert(std::abs(late.cloud->front().y - 10.0 * std::sin(held)) < 1e-3);
  node.continuous_time_pose_history_max_end_gap_sec_ = 0.0;

  // relative_motion still waits for IMU preintegration.
  node.continuous_time_deskew_mode_ = "relative_motion";
  assert(!node.usesGyroPoseHistoryWithoutPreintegration());
  auto relative = scan_with_point_at(0.1);
  assert(!node.applyContinuousTimeDeskewIfEnabled(relative, 10.1));
  assert(
    node.latest_continuous_time_deskew_status_ ==
    std::string("continuous_time_deskew_imu_preintegration_disabled"));

  rclcpp::shutdown();
  return 0;
}
