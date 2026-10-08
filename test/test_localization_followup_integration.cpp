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
    rclcpp::Parameter("point_timestamp_unit", "nanoseconds"),
    rclcpp::Parameter("enable_soft_odom_correction_gate", true),
    rclcpp::Parameter("enable_odom_tf_prediction_correction_guard", true),
    rclcpp::Parameter("odom_tf_prediction_correction_guard_translation_m", .3),
    rclcpp::Parameter("odom_tf_prediction_correction_guard_yaw_deg", 5.0)});
  {
    PCLLocalization node(options);
    node.initializeParameters();
    assert(node.path_max_poses_ == 2);
    assert(node.point_timestamp_unit_ == ll::PointTimestampUnit::kNanoseconds);
    assert(node.enable_soft_odom_correction_gate_);
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
    node.use_odom_tf_prediction_ = true;
    node.has_last_good_map_to_odom_ = true;
    for (double fitness : {.1, 100.0}) {
      ll::AlignmentPipelineResult r;
      r.selected_attempt.fitness_score = fitness;
      r.selected_attempt.accepted_gap_sec = 0;
      r.selected_attempt.seed_translation_since_accept_m = 0;
      r.selected_attempt.correction_translation_m = .6;
      r.selected_attempt.correction_yaw_deg = 0;
      r.selected_attempt.final_transformation(0, 3) = .6f;
      r.gate_result = node.evaluateMeasurementGateForAttempt(
        r.selected_attempt, ll::RegistrationSeedSource::kPreviousDelta);
      assert(r.gate_result.reject_measurement);
      node.applySoftOdomCorrectionGate(r);
      assert(r.gate_result.reject_measurement == (fitness == 100));
      if (fitness == .1) {
        assert(std::abs(r.selected_attempt.final_transformation(0, 3) - .3) < 1e-6);
        assert(std::abs(node.last_gated_correction_translation_m_ - .3) < 1e-6);
      }
    }
  }
  rclcpp::shutdown();
}
