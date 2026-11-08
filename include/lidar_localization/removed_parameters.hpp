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

inline constexpr std::array<RemovedSwitch, 10> kRemovedSwitches{{
  {"constrain_odom_tf_prediction_to_planar", "1.9.0",
    "kept the odometry prediction's x, y and yaw and took z, roll and pitch from the "
    "last accepted pose"},
  {"constrain_odom_tf_prediction_height_only", "1.9.0",
    "took the odometry prediction's height from the last accepted pose"},
  {"enable_map_odom_anchor_fitness_gate", "1.9.0",
    "kept map -> odom unchanged after an accepted scan with a fitness over "
    "map_odom_anchor_max_fitness"},
  {"enable_odom_tf_prediction_correction_guard", "1.9.0",
    "rejected a scan whose correction from the odometry seed exceeded "
    "odom_tf_prediction_correction_guard_translation_m or _yaw_deg"},
  {"enable_odom_tf_prediction_recovery_correction_guard", "1.9.0",
    "widened that guard after a rejection streak for a strong match"},
  {"enable_soft_odom_correction_gate", "1.9.0",
    "blended a scan that guard rejected with the odometry prediction"},
  {"enable_consistency_recovery_gate", "1.9.0",
    "accepted a scan over the score threshold after a rejection streak when its "
    "correction was small"},
  {"use_twist_ekf", "1.9.0",
    "ran a constant-velocity EKF on the twist; on the Istanbul windows it lost track in "
    "three of six runs where use_gtsam_smoother kept all of them, so use that"},
  {"enable_localizability_guard", "1.9.0",
    "dropped the previous-delta seed when the scan's XY spread was narrow"},
  {"enable_open_loop_strict_score_threshold", "1.9.0",
    "lowered the score threshold once no scan had been accepted for a while and the seed "
    "had moved far from the last accepted pose"},
}};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_REMOVED_PARAMETERS_HPP_
