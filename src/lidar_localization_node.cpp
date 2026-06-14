#include <lidar_localization/lidar_localization_component.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  {
    // SingleThreadedExecutor is intentional: every subscription, timer and
    // service callback runs in the default mutually-exclusive callback group
    // on one thread, so the node's shared mutable state (e.g.
    // corrent_pose_with_cov_stamped_ptr_) is serialized without locks. Switching
    // to a MultiThreadedExecutor or adding a Reentrant callback group would
    // introduce data races and requires guarding that state first.
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::NodeOptions options;
    std::shared_ptr<PCLLocalization> pcl_l = std::make_shared<PCLLocalization>(options);

    executor.add_node(pcl_l->get_node_base_interface());
    executor.spin();
    executor.remove_node(pcl_l->get_node_base_interface());
    pcl_l.reset();
  }
  rclcpp::shutdown();

  return 0;
}
