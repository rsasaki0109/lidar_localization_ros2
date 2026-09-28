#include <lidar_localization/lidar_localization_component.hpp>
#include <cassert>

namespace ll = lidar_localization;

void exercise(int mode)
{
  PCLLocalization node{rclcpp::NodeOptions{}};
  node.configure();
  node.enable_map_odom_tf_ = true;
  node.use_odom_tf_prediction_ = mode != 4;
  node.tfbuffer_.setUsingDedicatedThread(true);
  node.constrain_odom_tf_prediction_to_planar_ = false;
  node.constrain_odom_tf_prediction_height_only_ = false;
  node.measurement_gate_config_.enable_seed_correction_guard = mode != 6;
  node.measurement_gate_config_.seed_correction_guard_translation_m = 0.3;
  node.measurement_gate_config_.seed_correction_guard_yaw_deg = 15.0;
  node.measurement_gate_config_.seed_correction_guard_warmup_accepts = 5;
  node.measurement_gate_config_.seed_correction_guard_release_rejections = 30;
  node.accepted_updates_since_reset_ = mode == 7 ? 4 : 5;
  node.consecutive_rejected_updates_ = mode == 8 ? 30 : 14;
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = node.odom_frame_id_;
  tf.child_frame_id = node.base_frame_id_;
  tf.header.stamp.sec = 1;
  tf.transform.rotation.w = 1;
  assert(node.tfbuffer_.setTransform(tf, "fixture", false));
  auto pose = tf;
  pose.header.frame_id = node.global_frame_id_;
  assert(node.publishMapToOdomTransform(pose.header.stamp, pose, true));
  tf.header.stamp.sec = 2;
  tf.transform.translation.x = 1;
  if (mode != 2) {assert(node.tfbuffer_.setTransform(tf, "fixture", false));}
  if (mode == 3) {node.has_last_good_map_to_odom_ = false;}

  ll::AlignmentPipelineResult result;
  result.selected_attempt.target_ready = true;
  result.selected_attempt.has_converged = true;
  result.selected_attempt.fitness_score = 0.04;
  result.selected_attempt.correction_translation_m = 0.1;
  result.selected_attempt.correction_yaw_deg = 0.0;
  result.selected_attempt.final_transformation(0, 3) = mode == 1 ? 1.1f : 3.0f;
  if (mode == 9) {
    result.gate_result.reject_measurement = true;
    result.gate_result.status_message = "original_rejection";
  }
  if (mode == 10) {result.should_continue = false;}
  if (mode == 11) {result.recovered_by_retry_from_last_pose = true;}
  node.recheckAlignmentWithReturnedOdomTf(
    tf.header.stamp, mode == 5 ? ll::RegistrationSeedSource::kOdomTfPrediction :
    ll::RegistrationSeedSource::kTwistPrediction, result);
  const bool should_reject = mode == 0 || mode == 11;
  assert(result.gate_result.reject_measurement == (should_reject || mode == 9));
  if (should_reject) {
    assert(result.status_message == "returned_odom_tf_seed_correction_guard_rejected");
    assert(!result.gate_result.rejected_seed_update_applied);
    assert(!result.recovered_by_retry_from_last_pose);
  }
  if (mode == 9) {assert(result.gate_result.status_message == "original_rejection");}
  assert(node.last_good_map_to_odom_.header.stamp.sec == 1);
  assert(result.selected_attempt.correction_translation_m == 0.1);
  node.cleanup();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  for (int mode = 0; mode < 12; ++mode) {exercise(mode);}
  rclcpp::shutdown();
}
