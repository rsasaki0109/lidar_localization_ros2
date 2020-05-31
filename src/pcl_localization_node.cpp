#include <pcl_localization/pcl_localization_component.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::NodeOptions options;
  std::shared_ptr<PCLLocalization> pcl_l = std::make_shared<PCLLocalization>(options);

  executor.add_node(pcl_l->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
