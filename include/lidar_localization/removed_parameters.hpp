#ifndef LIDAR_LOCALIZATION_REMOVED_PARAMETERS_HPP_
#define LIDAR_LOCALIZATION_REMOVED_PARAMETERS_HPP_

#include <array>
#include <string_view>

namespace lidar_localization
{

// Switches of features that were removed. Each stays declared, off, so that
// on_configure() refuses a configuration that still turns it on instead of
// ignoring it. Their other parameters are no longer declared, and ROS ignores them.
struct RemovedSwitch
{
  std::string_view name;
  std::string_view removed_in;
  std::string_view what_it_did;
};

inline constexpr std::array<RemovedSwitch, 3> kRemovedSwitches{{
  {"constrain_odom_tf_prediction_to_planar", "1.9.0",
    "kept the odometry prediction's x, y and yaw and took z, roll and pitch from the "
    "last accepted pose"},
  {"constrain_odom_tf_prediction_height_only", "1.9.0",
    "took the odometry prediction's height from the last accepted pose"},
  {"enable_map_odom_anchor_fitness_gate", "1.9.0",
    "kept map -> odom unchanged after an accepted scan with a fitness over "
    "map_odom_anchor_max_fitness"},
}};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_REMOVED_PARAMETERS_HPP_
