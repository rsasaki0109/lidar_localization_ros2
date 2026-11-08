// A removed feature's switch stays declared: configuring with it on fails instead of
// running without the feature it asked for.
#include <lidar_localization/lidar_localization_component.hpp>
#include <lidar_localization/removed_parameters.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <cassert>
#include <memory>
#include <string>
#include <string_view>

namespace
{

bool configureSucceeds(std::string_view enabled_switch)
{
  rclcpp::NodeOptions options;
  if (!enabled_switch.empty()) {
    options.parameter_overrides({rclcpp::Parameter(std::string(enabled_switch), true)});
  }
  auto node = std::make_shared<PCLLocalization>(options);
  const auto state = node->configure();
  return state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  assert(configureSucceeds(""));
  for (const auto & removed : lidar_localization::kRemovedSwitches) {
    assert(!configureSucceeds(removed.name));
  }
  rclcpp::shutdown();
  return 0;
}
