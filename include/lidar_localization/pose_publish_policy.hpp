#ifndef LIDAR_LOCALIZATION_POSE_PUBLISH_POLICY_HPP_
#define LIDAR_LOCALIZATION_POSE_PUBLISH_POLICY_HPP_

#include <cmath>
#include <cstddef>
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

// 2000 poses is roughly 100 s at 20 Hz and 200 s at a 10 Hz scan rate, which
// keeps the RViz trail useful while bounding the per-publish cost.
inline constexpr std::size_t kDefaultPathMaxPoses = 2000;

struct PathMaxPoses
{
  std::size_t value;
  bool was_adjusted;
};

// A value of 0 disables the bound and keeps the full path history.
inline PathMaxPoses normalizePathMaxPoses(int requested, std::size_t fallback)
{
  if (requested < 0) {
    return {fallback, true};
  }
  return {static_cast<std::size_t>(requested), false};
}

// Appends a pose to the visualization path, dropping the oldest entries once
// the history exceeds max_poses. This bounds both the per-publish
// serialization cost and the retained transient-local sample.
inline void appendPoseToPath(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose_stamped,
  std::size_t max_poses = kDefaultPathMaxPoses)
{
  path.poses.push_back(pose_stamped);
  if (max_poses > 0 && path.poses.size() > max_poses) {
    path.poses.erase(
      path.poses.begin(),
      path.poses.end() - static_cast<std::ptrdiff_t>(max_poses));
  }
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

// With `level`, map -> odom keeps only its yaw: when both the map and an inertial
// odometry front end are gravity-aligned, a roll/pitch error of one registration
// would otherwise tilt every pose bridged on odometry afterwards. The translation
// is chosen so the base position at this stamp is unchanged.
inline geometry_msgs::msg::TransformStamped composeMapToOdomTransform(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & global_frame_id,
  const std::string & odom_frame_id,
  const geometry_msgs::msg::TransformStamped & map_to_base_link,
  const geometry_msgs::msg::TransformStamped & odom_to_base_link,
  bool level = false)
{
  tf2::Transform map_to_base_link_tf;
  tf2::Transform odom_to_base_link_tf;
  tf2::fromMsg(map_to_base_link.transform, map_to_base_link_tf);
  tf2::fromMsg(odom_to_base_link.transform, odom_to_base_link_tf);

  tf2::Transform map_to_odom_tf = map_to_base_link_tf * odom_to_base_link_tf.inverse();
  if (level) {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(map_to_odom_tf.getRotation()).getRPY(roll, pitch, yaw);
    tf2::Quaternion yaw_only;
    yaw_only.setRPY(0.0, 0.0, yaw);
    map_to_odom_tf.setRotation(yaw_only);
    map_to_odom_tf.setOrigin(
      map_to_base_link_tf.getOrigin() - tf2::quatRotate(yaw_only, odom_to_base_link_tf.getOrigin()));
  }

  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = stamp;
  map_to_odom.header.frame_id = global_frame_id;
  map_to_odom.child_frame_id = odom_frame_id;
  map_to_odom.transform = tf2::toMsg(map_to_odom_tf);
  return map_to_odom;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_POSE_PUBLISH_POLICY_HPP_
