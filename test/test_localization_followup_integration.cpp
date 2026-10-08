#include <lidar_localization/lidar_localization_component.hpp>
#include <cassert>
#include <cmath>

namespace ll = lidar_localization;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("path_max_poses", 2),
    rclcpp::Parameter("point_timestamp_unit", "nanoseconds")});
  {
    PCLLocalization node(options);
    node.initializeParameters();
    assert(node.path_max_poses_ == 2);
    assert(node.point_timestamp_unit_ == ll::PointTimestampUnit::kNanoseconds);
    node.path_ptr_ = std::make_shared<nav_msgs::msg::Path>();
    builtin_interfaces::msg::Time stamp;
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1;
    for (int i = 0; i < 5; ++i) {
      stamp.sec = i;
      node.appendCurrentPoseToPath(stamp, pose);
    }
    assert(node.path_ptr_->poses.size() == 2);
    assert(node.path_ptr_->poses.front().header.stamp.sec == 3);
    assert(node.path_ptr_->poses.back().header.stamp.sec == 4);

  }
  rclcpp::shutdown();
}
