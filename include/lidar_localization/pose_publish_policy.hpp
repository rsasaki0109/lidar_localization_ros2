#ifndef LIDAR_LOCALIZATION_POSE_PUBLISH_POLICY_HPP_
#define LIDAR_LOCALIZATION_POSE_PUBLISH_POLICY_HPP_

#include <cmath>
#include <optional>
#include <string>

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lidar_localization
{

inline bool shouldPublishAcceptedPoseImmediately(
  bool enable_timer_publishing,
  bool publish_bridge_pose_when_lost)
{
  // A bridge timer supplements scan-stamped accepted outputs.  Replacing
  // them would leave startup uncovered until the first timer tick and would
  // expose gaps whenever the external odom TF pauses briefly.
  return !enable_timer_publishing || publish_bridge_pose_when_lost;
}

inline bool shouldFreezeMapToOdomAnchor(
  bool enable_fitness_gate,
  double max_fitness,
  double fitness_score,
  double max_correction_rotation_deg,
  double correction_rotation_deg)
{
  // With an external odometry bridge, repeatedly freezing individually small
  // but low-confidence registration corrections can make map -> odom drift
  // cumulatively.  Disabled preserves the legacy behaviour.  When enabled,
  // only a finite, sufficiently good scan match may replace the bridge anchor.
  return !enable_fitness_gate ||
         (std::isfinite(max_fitness) && max_fitness >= 0.0 &&
         std::isfinite(fitness_score) && fitness_score <= max_fitness &&
         std::isfinite(max_correction_rotation_deg) &&
         max_correction_rotation_deg >= 0.0 &&
         std::isfinite(correction_rotation_deg) &&
         correction_rotation_deg <= max_correction_rotation_deg);
}

inline double builtinTimeSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}

inline std::optional<geometry_msgs::msg::TransformStamped>
extrapolateTransformConstantBodyMotion(
  const geometry_msgs::msg::TransformStamped & previous,
  const geometry_msgs::msg::TransformStamped & latest,
  const builtin_interfaces::msg::Time & target_stamp,
  double max_extrapolation_sec)
{
  const double previous_sec = builtinTimeSeconds(previous.header.stamp);
  const double latest_sec = builtinTimeSeconds(latest.header.stamp);
  const double target_sec = builtinTimeSeconds(target_stamp);
  const double history_sec = latest_sec - previous_sec;
  const double future_sec = target_sec - latest_sec;
  if (
    !std::isfinite(history_sec) || !std::isfinite(future_sec) ||
    !std::isfinite(max_extrapolation_sec) || history_sec <= 1.0e-3 ||
    future_sec <= 0.0 || future_sec > max_extrapolation_sec)
  {
    return std::nullopt;
  }

  tf2::Transform previous_tf;
  tf2::Transform latest_tf;
  tf2::fromMsg(previous.transform, previous_tf);
  tf2::fromMsg(latest.transform, latest_tf);
  const tf2::Transform delta = previous_tf.inverse() * latest_tf;
  const double scale = future_sec / history_sec;

  tf2::Quaternion scaled_rotation;
  const tf2::Quaternion delta_rotation = delta.getRotation();
  const double angle = delta_rotation.getAngleShortestPath();
  if (std::abs(angle) <= 1.0e-9) {
    scaled_rotation.setValue(0.0, 0.0, 0.0, 1.0);
  } else {
    scaled_rotation.setRotation(delta_rotation.getAxis(), angle * scale);
  }
  const tf2::Transform scaled_delta(
    scaled_rotation, delta.getOrigin() * scale);
  const tf2::Transform predicted_tf = latest_tf * scaled_delta;

  geometry_msgs::msg::TransformStamped predicted = latest;
  predicted.header.stamp = target_stamp;
  predicted.transform = tf2::toMsg(predicted_tf);
  return predicted;
}

inline geometry_msgs::msg::Pose poseFromMatrix(const Eigen::Matrix4f & pose_matrix)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Matrix3d rot_mat = pose_matrix.block<3, 3>(0, 0).cast<double>();
  const Eigen::Quaterniond quat_eig(rot_mat);
  pose.position.x = static_cast<double>(pose_matrix(0, 3));
  pose.position.y = static_cast<double>(pose_matrix(1, 3));
  pose.position.z = static_cast<double>(pose_matrix(2, 3));
  pose.orientation.x = quat_eig.x();
  pose.orientation.y = quat_eig.y();
  pose.orientation.z = quat_eig.z();
  pose.orientation.w = quat_eig.w();
  return pose;
}

inline Eigen::Matrix4f constrainOdomPredictionToPlanarMotion(
  const Eigen::Matrix4f & odom_prediction,
  const Eigen::Matrix4f & last_accepted_pose)
{
  // Outdoor LIO retains useful planar motion through long geometrically
  // degenerate stretches, while its unobservable height and tilt can drift by
  // many metres/degrees. Keep the prediction's x/y/yaw, but anchor z/roll/pitch
  // to the last scan-matched pose. This is deliberately an opt-in policy at the
  // component level; unconstrained 6-DoF users keep the legacy composition.
  // Extract the canonical ground-vehicle ZYX angles directly. Eigen's generic
  // eulerAngles() is allowed to return the equivalent yaw+pi / inverted-tilt
  // branch; mixing yaw from one pose with tilt from another would then create
  // a real 180-degree jump even though each original rotation was continuous.
  const float yaw_rad = std::atan2(odom_prediction(1, 0), odom_prediction(0, 0));
  const float pitch_rad = std::asin(
    std::clamp(-last_accepted_pose(2, 0), -1.0f, 1.0f));
  const float roll_rad = std::atan2(
    last_accepted_pose(2, 1), last_accepted_pose(2, 2));
  const Eigen::AngleAxisf roll(roll_rad, Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf pitch(pitch_rad, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf yaw(yaw_rad, Eigen::Vector3f::UnitZ());

  Eigen::Matrix4f constrained = odom_prediction;
  constrained.block<3, 3>(0, 0) = (yaw * pitch * roll).toRotationMatrix();
  constrained(2, 3) = last_accepted_pose(2, 3);
  return constrained;
}

inline Eigen::Matrix4f constrainOdomPredictionHeightOnly(
  const Eigen::Matrix4f & odom_prediction,
  const Eigen::Matrix4f & anchor_pose)
{
  // Some sensor frames do not use the conventional ground-vehicle axes, so
  // fixing ZYX roll/pitch in the map frame is not meaningful. Preserve the
  // external odometry's full relative rotation and horizontal motion while
  // removing only its unobservable height drift.
  Eigen::Matrix4f constrained = odom_prediction;
  constrained(2, 3) = anchor_pose(2, 3);
  return constrained;
}

inline geometry_msgs::msg::PoseStamped makePoseStamped(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id,
  const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = stamp;
  pose_stamped.header.frame_id = frame_id;
  pose_stamped.pose = pose;
  return pose_stamped;
}

inline geometry_msgs::msg::PoseWithCovarianceStamped stampPoseWithCovariance(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
  const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::PoseWithCovarianceStamped stamped_pose = pose;
  stamped_pose.header.stamp = stamp;
  return stamped_pose;
}

inline void appendPoseToPath(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose_stamped)
{
  path.poses.push_back(pose_stamped);
}

inline geometry_msgs::msg::TransformStamped makeMapToBaseTransform(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & global_frame_id,
  const std::string & base_frame_id,
  const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = global_frame_id;
  transform.child_frame_id = base_frame_id;
  transform.transform.translation.x = pose.position.x;
  transform.transform.translation.y = pose.position.y;
  transform.transform.translation.z = pose.position.z;
  transform.transform.rotation = pose.orientation;
  return transform;
}

inline geometry_msgs::msg::TransformStamped composeMapToOdomTransform(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & global_frame_id,
  const std::string & odom_frame_id,
  const geometry_msgs::msg::TransformStamped & map_to_base_link,
  const geometry_msgs::msg::TransformStamped & odom_to_base_link)
{
  tf2::Transform map_to_base_link_tf;
  tf2::Transform odom_to_base_link_tf;
  tf2::fromMsg(map_to_base_link.transform, map_to_base_link_tf);
  tf2::fromMsg(odom_to_base_link.transform, odom_to_base_link_tf);

  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = stamp;
  map_to_odom.header.frame_id = global_frame_id;
  map_to_odom.child_frame_id = odom_frame_id;
  map_to_odom.transform = tf2::toMsg(map_to_base_link_tf * odom_to_base_link_tf.inverse());
  return map_to_odom;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_POSE_PUBLISH_POLICY_HPP_
