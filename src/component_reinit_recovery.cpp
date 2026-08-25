#include "component_internal.hpp"
PCLLocalization::ReinitializationRequestDecision
PCLLocalization::applyReinitializationRequestLatch(
  const builtin_interfaces::msg::Time & stamp,
  const ReinitializationRequestDecision & decision,
  bool clear_sample)
{
  const auto latch_result = lidar_localization::applyReinitializationRequestLatch(
    lidar_localization::ReinitializationRequestLatchInput{
      enable_reinitialization_request_latch_,
      lidar_localization::ReinitializationRequestLatchState{
        reinitialization_request_latched_,
        reinitialization_request_latch_reason_,
        reinitialization_request_latch_score_,
        reinitialization_request_latch_stamp_sec_,
        reinitialization_request_latch_consecutive_ok_samples_},
      decision,
      stamp_to_sec(stamp),
      clear_sample,
      reinitialization_request_clear_ok_samples_});

  reinitialization_request_latched_ = latch_result.state.latched;
  reinitialization_request_latch_reason_ = latch_result.state.reason;
  reinitialization_request_latch_score_ = latch_result.state.score;
  reinitialization_request_latch_stamp_sec_ = latch_result.state.stamp_sec;
  reinitialization_request_latch_consecutive_ok_samples_ =
    latch_result.state.consecutive_clear_samples;
  return latch_result.decision;
}

void PCLLocalization::updateRecoverySupervisorState(
  const builtin_interfaces::msg::Time & stamp,
  RecoverySupervisorState next_state,
  const std::string & action)
{
  const double stamp_sec = stamp_to_sec(stamp);
  const auto update = lidar_localization::updateRecoverySupervisorRuntimeState(
    lidar_localization::RecoverySupervisorStateUpdateInput{
      lidar_localization::RecoverySupervisorRuntimeState{
        recovery_supervisor_state_,
        recovery_supervisor_action_,
        recovery_supervisor_state_entered_stamp_sec_,
        recovery_supervisor_transition_count_},
      next_state,
      action,
      stamp_sec});

  if (update.transitioned) {
    RCLCPP_INFO(
      get_logger(),
      "Recovery supervisor state transition: %s -> %s (%s)",
      lidar_localization::recoverySupervisorStateName(update.previous_state),
      lidar_localization::recoverySupervisorStateName(update.state.state),
      action.c_str());
  }

  recovery_supervisor_state_ = update.state.state;
  recovery_supervisor_action_ = update.state.action;
  recovery_supervisor_state_entered_stamp_sec_ = update.state.entered_stamp_sec;
  recovery_supervisor_transition_count_ = update.state.transition_count;
}

void PCLLocalization::publishReinitializationRequest(
  const builtin_interfaces::msg::Time & stamp,
  const ReinitializationRequestDecision & decision)
{
  (void)stamp;
  const bool publisher_ready =
    reinitialization_request_pub_ && reinitialization_request_pub_->is_activated();
  const auto output = lidar_localization::prepareReinitializationRequestOutput(
    lidar_localization::ReinitializationRequestOutputInput{
      enable_reinitialization_request_output_,
      publisher_ready,
      decision});

  reinitialization_requested_ = output.state.requested;
  reinitialization_request_reason_ = output.state.reason;
  reinitialization_request_score_ = output.state.score;

  if (!output.should_publish) {
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = output.message_value;
  reinitialization_request_pub_->publish(msg);
}

lidar_localization::MeasurementGateParams PCLLocalization::measurementGateParams() const
{
  return lidar_localization::makeMeasurementGateParams(measurement_gate_config_);
}

lidar_localization::ReinitializationTriggerParams
PCLLocalization::reinitializationTriggerParams() const
{
  return reinitialization_trigger_config_;
}

lidar_localization::RecoveryRetryFromLastPoseParams
PCLLocalization::recoveryRetryFromLastPoseParams() const
{
  return recovery_retry_from_last_pose_config_;
}

