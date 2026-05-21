#ifndef LIDAR_LOCALIZATION_POSE_PUBLISH_POLICY_HPP_
#define LIDAR_LOCALIZATION_POSE_PUBLISH_POLICY_HPP_

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
