// The IMU subscription gets its own callback group whenever the preintegration needs
// its samples before a scan is prepared, with or without the use_imu deskew. With
// use_imu it used to share the cloud callback's group: the samples then waited behind
// each registration, arrived in bursts after the scan had used a stale window, and on
// Boreas the pose left the road within 10 s.
#include <lidar_localization/lidar_localization_component.hpp>

#include <cassert>
#include <memory>

namespace
{

bool hasOwnImuGroup(bool use_imu, bool use_imu_preintegration)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("use_imu", use_imu),
    rclcpp::Parameter("use_imu_preintegration", use_imu_preintegration)});
  auto node = std::make_shared<PCLLocalization>(options);
  node->configure();
  assert(node->imu_sub_);
  const bool own_group = static_cast<bool>(node->imu_callback_group_);
  node->cleanup();
  return own_group;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  assert(hasOwnImuGroup(false, true));
  assert(hasOwnImuGroup(true, true));
  // Without the preintegration (or the gyro history) nothing needs the samples early.
  assert(!hasOwnImuGroup(false, false));
  assert(!hasOwnImuGroup(true, false));
  rclcpp::shutdown();
  return 0;
}
