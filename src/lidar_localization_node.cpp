#include <lidar_localization/lidar_localization_component.hpp>

#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  {
    // The default callback group remains mutually-exclusive. IMU preintegration
    // and opt-in timer publication use dedicated guarded groups so neither is
    // starved by long cloud registration callbacks during bag replay.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
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
