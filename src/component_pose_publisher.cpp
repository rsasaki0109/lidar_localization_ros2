#include "component_internal.hpp"
void PCLLocalization::setCurrentPoseFromMatrix(
  const Eigen::Matrix4f & pose_matrix,
  const builtin_interfaces::msg::Time & stamp)
{
  corrent_pose_with_cov_stamped_ptr_->header.stamp = stamp;
  corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
  corrent_pose_with_cov_stamped_ptr_->pose.pose =
    lidar_localization::poseFromMatrix(pose_matrix);
}

void PCLLocalization::publishCurrentPose(
  const builtin_interfaces::msg::Time & stamp,
  double fitness_score,
  double correction_rotation_deg)
{
  if (
    shutting_down_.load(std::memory_order_acquire) ||
    !pose_pub_ || !corrent_pose_with_cov_stamped_ptr_)
  {
    return;
  }
  const bool freeze_as_last_good = lidar_localization::shouldFreezeMapToOdomAnchor(
    enable_map_odom_anchor_fitness_gate_, map_odom_anchor_max_fitness_, fitness_score,
    map_odom_anchor_max_correction_rotation_deg_, correction_rotation_deg);
  if (
    !freeze_as_last_good && publish_bridge_pose_when_lost_ &&
    has_last_good_map_to_odom_)
  {
    // This accepted scan is useful to the estimator but not trustworthy
    // enough to move the public odometry anchor. Do not inject its transient
    // pose into /pcl_pose or TF; emit the confidence-bounded bridge at the
    // same scan stamp instead.
    publishBridgePoseAsRejectedOutput(stamp);
    geometry_msgs::msg::TransformStamped frozen = last_good_map_to_odom_;
    frozen.header.stamp = stamp;
    broadcaster_.sendTransform(frozen);
    return;
  }
  publishPoseMessage(*corrent_pose_with_cov_stamped_ptr_);
  if (!publishPoseTransform(
      stamp, corrent_pose_with_cov_stamped_ptr_->pose.pose, freeze_as_last_good))
  {
    return;
  }

  appendCurrentPoseToPath(stamp, corrent_pose_with_cov_stamped_ptr_->pose.pose);
  publishPathMessage();
}

void PCLLocalization::publishPoseMessage(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  pose_pub_->publish(pose);
}

void PCLLocalization::appendCurrentPoseToPath(
  const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::Pose & pose)
{
  lidar_localization::appendPoseToPath(
    *path_ptr_,
    lidar_localization::makePoseStamped(stamp, global_frame_id_, pose),
    path_max_poses_);
}

void PCLLocalization::publishPathMessage()
{
  path_pub_->publish(*path_ptr_);
}

bool PCLLocalization::publishPoseTransform(
  const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::Pose & pose,
  bool freeze_as_last_good)
{
  const geometry_msgs::msg::TransformStamped map_to_base_link_stamped =
    lidar_localization::makeMapToBaseTransform(
      stamp,
      global_frame_id_,
      base_frame_id_,
      pose);
  if (!enable_map_odom_tf_) {
    broadcaster_.sendTransform(map_to_base_link_stamped);
    return true;
  }

  return publishMapToOdomTransform(stamp, map_to_base_link_stamped, freeze_as_last_good);
}

bool PCLLocalization::publishMapToOdomTransform(
  const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::TransformStamped & map_to_base_link_stamped,
  bool freeze_as_last_good)
{
  geometry_msgs::msg::TransformStamped odom_to_base_link_msg;
  try {
    odom_to_base_link_msg = tfbuffer_.lookupTransform(
      odom_frame_id_, base_frame_id_, stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not get transform %s to %s: %s",
      base_frame_id_.c_str(), odom_frame_id_.c_str(), ex.what());
    return false;
  }
  const geometry_msgs::msg::TransformStamped map_to_odom =
    lidar_localization::composeMapToOdomTransform(
      stamp,
      global_frame_id_,
      odom_frame_id_,
      map_to_base_link_stamped,
      odom_to_base_link_msg);
  broadcaster_.sendTransform(map_to_odom);
  // Freeze this map -> odom offset so republishFrozenMapToOdomTransform can keep
  // it alive (re-stamped) through a dropout. Only called from accepted-match /
  // accepted-reset call sites (see publishPoseTransform's default and its one
  // explicit false caller, timerPublishPose) -- never from a rejected match.
  if (freeze_as_last_good) {
    last_good_map_to_odom_ = map_to_odom;
    has_last_good_map_to_odom_ = true;
    geometry_msgs::msg::Pose anchor_pose;
    anchor_pose.position.x = map_to_base_link_stamped.transform.translation.x;
    anchor_pose.position.y = map_to_base_link_stamped.transform.translation.y;
    anchor_pose.position.z = map_to_base_link_stamped.transform.translation.z;
    anchor_pose.orientation = map_to_base_link_stamped.transform.rotation;
    Eigen::Affine3d anchor_affine;
    tf2::fromMsg(anchor_pose, anchor_affine);
    odom_tf_constraint_anchor_pose_matrix_ = anchor_affine.matrix().cast<float>();
    has_odom_tf_constraint_anchor_pose_ = true;
  }
  return true;
}

void PCLLocalization::republishFrozenMapToOdomTransform(
  const builtin_interfaces::msg::Time & stamp)
{
  // AMCL-style bridge: while enable_map_odom_tf_ is on, keep the last ACCEPTED
  // map -> odom offset alive with a fresh stamp even when the current scan was
  // rejected/skipped, so map -> base_link stays resolvable via TF composition
  // with the external front end's continuously-live odom -> base_link for the
  // whole dropout (see docs on the GLIM front-end odom-bridge architecture).
  if (!enable_map_odom_tf_ || !has_last_good_map_to_odom_) {
    return;
  }
  geometry_msgs::msg::TransformStamped frozen = last_good_map_to_odom_;
  frozen.header.stamp = stamp;
  broadcaster_.sendTransform(frozen);
  publishOdomBridgePose(stamp, frozen);
}

void PCLLocalization::publishOdomBridgePose(
  const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::TransformStamped & frozen_map_to_odom)
{
  // Compose map -> base_link(now) = map -> odom(last accepted, frozen) x
  // odom -> base_link(now) and publish it for G3's odom-bridge candidate
  // source. This is the same composition publishMapToOdomTransform performs
  // on an accept, just with the frozen offset and the current live odom edge.
  //
  // Published on a plain topic (not read back off /tf) because a *third*
  // process's TF listener was found not to reliably receive this node's /tf
  // in testing (default volatile durability -- a late/separate-process
  // subscriber can miss every sample), while the KeepLast(1)/transient_local
  // QoS used here and by pcl_pose does reach a late subscriber reliably.
  if (!odom_bridge_pose_pub_ || !odom_bridge_pose_pub_->is_activated()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "odom bridge: publisher missing or not activated; skipping");
    return;
  }
  geometry_msgs::msg::TransformStamped odom_to_base_link_msg;
  try {
    odom_to_base_link_msg = tfbuffer_.lookupTransform(
      odom_frame_id_, base_frame_id_, stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & ex) {
    // No live odom right now (external front end down/lagging): the odom
    // bridge is simply unavailable this tick, exactly like any other TF-chain
    // failure -- do not publish a stale/guessed pose.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "odom bridge: %s -> %s lookup failed at publish time: %s",
      odom_frame_id_.c_str(), base_frame_id_.c_str(), ex.what());
    return;
  }
  tf2::Transform map_to_odom_tf;
  tf2::Transform odom_to_base_tf;
  tf2::fromMsg(frozen_map_to_odom.transform, map_to_odom_tf);
  tf2::fromMsg(odom_to_base_link_msg.transform, odom_to_base_tf);
  const geometry_msgs::msg::Transform composed_transform =
    tf2::toMsg(map_to_odom_tf * odom_to_base_tf);

  geometry_msgs::msg::Pose composed_pose;
  composed_pose.position.x = composed_transform.translation.x;
  composed_pose.position.y = composed_transform.translation.y;
  composed_pose.position.z = composed_transform.translation.z;
  composed_pose.orientation = composed_transform.rotation;
  Eigen::Affine3d composed_affine;
  tf2::fromMsg(composed_pose, composed_affine);
  Eigen::Matrix4f bridge_matrix = composed_affine.matrix().cast<float>();
  if (
    constrain_odom_tf_prediction_height_only_ &&
    has_odom_tf_constraint_anchor_pose_)
  {
    bridge_matrix = lidar_localization::constrainOdomPredictionHeightOnly(
      bridge_matrix, odom_tf_constraint_anchor_pose_matrix_);
    composed_pose = lidar_localization::poseFromMatrix(bridge_matrix);
  } else if (
    constrain_odom_tf_prediction_to_planar_ &&
    has_odom_tf_constraint_anchor_pose_)
  {
    bridge_matrix = lidar_localization::constrainOdomPredictionToPlanarMotion(
      bridge_matrix, odom_tf_constraint_anchor_pose_matrix_);
    composed_pose = lidar_localization::poseFromMatrix(bridge_matrix);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = global_frame_id_;
  msg.pose.pose = composed_pose;
  odom_bridge_pose_pub_->publish(msg);
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "odom bridge: published odom_bridge_pose (%.2f, %.2f, z=%.2f)",
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
}

bool PCLLocalization::lookupOdomBridgePoseMatrix(
  const builtin_interfaces::msg::Time & stamp,
  Eigen::Matrix4f & out_pose_matrix,
  bool use_latest_odom_transform,
  builtin_interfaces::msg::Time * resolved_stamp)
{
  // Same composition as publishOdomBridgePose (frozen map -> odom x live
  // odom -> base_link(stamp)), but returning an Eigen matrix for use as an
  // NDT registration seed (see selectRegistrationSeed / use_odom_tf_prediction_).
  if (!enable_map_odom_tf_ || !has_last_good_map_to_odom_) {
    return false;
  }
  geometry_msgs::msg::TransformStamped odom_to_base_link_msg;
  try {
    if (use_latest_odom_transform) {
      odom_to_base_link_msg = tfbuffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, tf2::TimePointZero);
    } else {
      odom_to_base_link_msg = tfbuffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, stamp, rclcpp::Duration::from_seconds(0.1));
    }
  } catch (tf2::TransformException &) {
    return false;
  }
  if (use_latest_odom_transform) {
    const rclcpp::Time node_stamp(stamp);
    const bool source_advanced =
      odom_bridge_transform_history_.empty() ||
      rclcpp::Time(odom_to_base_link_msg.header.stamp) >
      rclcpp::Time(odom_bridge_transform_history_.back().header.stamp);
    if (source_advanced) {
      odom_bridge_transform_history_.push_back(odom_to_base_link_msg);
      while (
        odom_bridge_transform_history_.size() > 2 &&
        (rclcpp::Time(odom_bridge_transform_history_.back().header.stamp) -
        rclcpp::Time(odom_bridge_transform_history_.front().header.stamp)).seconds() > 1.0)
      {
        odom_bridge_transform_history_.pop_front();
      }
      last_odom_bridge_source_advance_node_stamp_ = stamp;
      has_last_odom_bridge_source_advance_node_stamp_ = true;
    } else if (has_last_odom_bridge_source_advance_node_stamp_) {
      const double paused_sec =
        (node_stamp - rclcpp::Time(last_odom_bridge_source_advance_node_stamp_)).seconds();
      if (paused_sec < 0.0) {
        last_odom_bridge_source_advance_node_stamp_ = stamp;
      } else if (paused_sec > 0.5 && odom_bridge_transform_history_.size() >= 2) {
        const auto extrapolated =
          lidar_localization::extrapolateTransformConstantBodyMotion(
          odom_bridge_transform_history_.front(),
          odom_bridge_transform_history_.back(), stamp, 2.0);
        if (extrapolated.has_value()) {
          odom_to_base_link_msg = extrapolated.value();
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "odom bridge: upstream TF paused for %.3f sec; publishing bounded constant-motion extrapolation",
            paused_sec);
        }
      }
    }
  }
  if (resolved_stamp != nullptr) {
    *resolved_stamp = odom_to_base_link_msg.header.stamp;
  }
  tf2::Transform map_to_odom_tf;
  tf2::Transform odom_to_base_tf;
  tf2::fromMsg(last_good_map_to_odom_.transform, map_to_odom_tf);
  tf2::fromMsg(odom_to_base_link_msg.transform, odom_to_base_tf);
  const geometry_msgs::msg::Transform composed =
    tf2::toMsg(map_to_odom_tf * odom_to_base_tf);

  geometry_msgs::msg::Pose pose;
  pose.position.x = composed.translation.x;
  pose.position.y = composed.translation.y;
  pose.position.z = composed.translation.z;
  pose.orientation = composed.rotation;
  Eigen::Affine3d affine;
  tf2::fromMsg(pose, affine);
  out_pose_matrix = affine.matrix().cast<float>();
  if (
    constrain_odom_tf_prediction_height_only_ &&
    has_odom_tf_constraint_anchor_pose_)
  {
    out_pose_matrix = lidar_localization::constrainOdomPredictionHeightOnly(
      out_pose_matrix, odom_tf_constraint_anchor_pose_matrix_);
  } else if (
    constrain_odom_tf_prediction_to_planar_ &&
    has_odom_tf_constraint_anchor_pose_)
  {
    out_pose_matrix = lidar_localization::constrainOdomPredictionToPlanarMotion(
      out_pose_matrix, odom_tf_constraint_anchor_pose_matrix_);
  }
  return true;
}

void PCLLocalization::publishBridgePoseAsRejectedOutput(
  const builtin_interfaces::msg::Time & stamp)
{
  // Opt-in output continuity for a rejected/lost scan: /pcl_pose otherwise
  // stays silent (only an accepted match calls publishCurrentPose), so the
  // benchmark's coverage/RMSE would see a gap for the whole dropout even
  // though the odom bridge has a perfectly good estimate. This intentionally
  // never touches corrent_pose_with_cov_stamped_ptr_, predicted_pose_matrix_,
  // or any other internal belief -- publishPoseMessage is a bare topic
  // publish, so the next scan's seed/prediction is unaffected either way.
  if (!publish_bridge_pose_when_lost_) {
    return;
  }
  Eigen::Matrix4f bridge_pose_matrix;
  if (!lookupOdomBridgePoseMatrix(stamp, bridge_pose_matrix)) {
    return;
  }
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = global_frame_id_;
  msg.pose.pose = lidar_localization::poseFromMatrix(bridge_pose_matrix);
  // Deliberately wider than an accepted fix's covariance (large fitness_score
  // input to the same error-floor model fillPoseCovariance uses): this is an
  // odometry-only prediction, not a scan-matched confirmation.
  const auto covariance = lidar_localization::makeErrorFloorPoseCovariance(
    error_floor_covariance_params_, error_floor_covariance_params_.xy_max_std_m * 10.0);
  msg.pose.covariance = covariance;
  publishPoseMessage(msg);
}

void PCLLocalization::fillPoseCovariance(double fitness_score)
{
  if (use_twist_ekf_ && twist_ekf_.isInitialized()) {
    corrent_pose_with_cov_stamped_ptr_->pose.covariance =
      use_error_floor_covariance_ ?
      lidar_localization::makeEkfErrorFloorPoseCovariance(
      twist_ekf_.covariance(), error_floor_covariance_params_, fitness_score) :
      lidar_localization::makeEkfPoseCovariance(twist_ekf_.covariance(), fitness_score);
    return;
  }

  corrent_pose_with_cov_stamped_ptr_->pose.covariance =
    use_error_floor_covariance_ ?
    lidar_localization::makeErrorFloorPoseCovariance(
    error_floor_covariance_params_, fitness_score) :
    lidar_localization::makeFitnessPoseCovariance(fitness_score);
}

void PCLLocalization::timerPublishPose()
{
  auto state_lock = callback_state_coordinator_.lockState();
  if (
    shutting_down_.load(std::memory_order_acquire) ||
    !pose_pub_ || !path_pub_ || !path_ptr_)
  {
    return;
  }
  if (!corrent_pose_with_cov_stamped_ptr_) {return;}
  builtin_interfaces::msg::Time publish_stamp = now();
  geometry_msgs::msg::PoseWithCovarianceStamped pose_copy =
    lidar_localization::stampPoseWithCovariance(
    *corrent_pose_with_cov_stamped_ptr_, publish_stamp);

  // A scan-driven bridge alone cannot guarantee an output-rate contract when
  // the upstream registered cloud pauses.  In the opt-in bridge mode, compose
  // the frozen accepted map->odom anchor with GLIM's latest odom->base pose on
  // every timer tick.  The latest lookup avoids needless future-extrapolation
  // failures when /clock is a few milliseconds ahead of the newest odom TF.
  bool published_odom_bridge_pose = false;
  if (publish_bridge_pose_when_lost_) {
    Eigen::Matrix4f bridge_pose_matrix;
    if (!lookupOdomBridgePoseMatrix(
        publish_stamp, bridge_pose_matrix, true))
    {
      // Before the first accepted map->odom anchor (or while GLIM has no live
      // odom TF), publishing the stale configured initial pose would fabricate
      // continuity.  It would also make the fallback exact-time lookup block
      // this mutually-exclusive callback group on every timer tick.
      return;
    }
    // Keep the timer's current ROS stamp even when the newest odom transform
    // has not advanced yet. Reusing the source TF stamp makes consecutive
    // timer outputs duplicates; consumers that de-duplicate by stamp then see
    // an artificial output gap during a short upstream pause. The pose itself
    // is still based on the latest transform (and becomes bounded
    // constant-motion extrapolation after the pause threshold above).
    pose_copy.pose.pose = lidar_localization::poseFromMatrix(bridge_pose_matrix);
    pose_copy.pose.covariance = lidar_localization::makeErrorFloorPoseCovariance(
      error_floor_covariance_params_,
      error_floor_covariance_params_.xy_max_std_m * 10.0);
    published_odom_bridge_pose = true;
  }

  appendCurrentPoseToPath(pose_copy.header.stamp, pose_copy.pose.pose);

  publishPoseMessage(pose_copy);
  if (published_odom_bridge_pose && odom_bridge_pose_pub_) {
    // Keep the supervisor candidate as fresh as the public bridge output.
    // Reusing pose_copy avoids a second exact-time TF lookup and preserves
    // the bounded extrapolation (plus its deliberately wide covariance)
    // through upstream TF pauses.
    odom_bridge_pose_pub_->publish(pose_copy);
  }
  publishPathMessage();

  // The timer republishes whatever pose was last set, which may already be
  // stale (held by prediction, not a fresh accept); do not re-freeze it as the
  // bridge's "last good" reference here -- that only happens at the accepted-
  // match / accepted-reset call sites (publishCurrentPose, initialPoseReceived).
  if (published_odom_bridge_pose) {
    // The pose was derived from this exact frozen edge. Re-stamp only that
    // edge here; republishFrozenMapToOdomTransform also performs an exact-time
    // odom lookup for the supervisor topic, which can consume the whole 10 Hz
    // callback budget while /clock is slightly ahead of GLIM's newest TF.
    geometry_msgs::msg::TransformStamped frozen = last_good_map_to_odom_;
    frozen.header.stamp = pose_copy.header.stamp;
    broadcaster_.sendTransform(frozen);
  } else {
    publishPoseTransform(pose_copy.header.stamp, pose_copy.pose.pose, false);
  }
}
