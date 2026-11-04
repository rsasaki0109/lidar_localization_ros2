// The IMU subscription gets its own callback group whenever the preintegration needs
// its samples before a scan is prepared. With the legacy use_imu deskew (removed in
// 1.8.0) it used to share the cloud callback's group: the samples then waited behind
// each registration, arrived in bursts after the scan had used a stale window, and on
// Boreas the pose left the road within 10 s.
//
// use_imu itself stays declared so that configuring with use_imu: true fails loudly
// instead of silently running without the deskew it asked for.
#include <lidar_localization/lidar_localization_component.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <cassert>
#include <memory>

namespace
{

std::shared_ptr<PCLLocalization> makeNode(bool use_imu, bool use_imu_preintegration)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("use_imu", use_imu),
    rclcpp::Parameter("use_imu_preintegration", use_imu_preintegration)});
  return std::make_shared<PCLLocalization>(options);
}

bool hasOwnImuGroup(bool use_imu_preintegration)
{
  auto node = makeNode(false, use_imu_preintegration);
  node->configure();
  assert(node->imu_sub_);
  const bool own_group = static_cast<bool>(node->imu_callback_group_);
  node->cleanup();
  return own_group;
}

bool configureSucceeds(bool use_imu)
{
  auto node = makeNode(use_imu, false);
  const auto state = node->configure();
  return state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  assert(hasOwnImuGroup(true));
  // Without the preintegration (or the gyro history) nothing needs the samples early.
  assert(!hasOwnImuGroup(false));
  assert(configureSucceeds(false));
  assert(!configureSucceeds(true));
  rclcpp::shutdown();
  return 0;
}
