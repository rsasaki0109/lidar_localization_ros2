#include "lidar_localization/pose_publish_policy.hpp"

#include <cassert>
#include <cmath>
#include <string>

namespace ll = lidar_localization;

builtin_interfaces::msg::Time stamp(int32_t sec, uint32_t nanosec = 0)
{
  builtin_interfaces::msg::Time time;
  time.sec = sec;
  time.nanosec = nanosec;
  return time;
}

geometry_msgs::msg::Pose pose_xyz(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

bool near(double lhs, double rhs)
{
  return std::abs(lhs - rhs) < 1.0e-9;
}

void test_accepted_pose_publication_policy()
{
  assert(ll::shouldPublishAcceptedPoseImmediately(false, false));
  assert(ll::shouldPublishAcceptedPoseImmediately(false, true));
  assert(!ll::shouldPublishAcceptedPoseImmediately(true, false));
  assert(ll::shouldPublishAcceptedPoseImmediately(true, true));
}

void test_map_odom_anchor_fitness_gate()
{
  assert(ll::shouldFreezeMapToOdomAnchor(false, 1.5, 9.0, 10.0, 120.0));
  assert(ll::shouldFreezeMapToOdomAnchor(true, 1.5, 1.5, 10.0, 10.0));
  assert(!ll::shouldFreezeMapToOdomAnchor(true, 1.5, 1.5001, 10.0, 1.0));
  assert(!ll::shouldFreezeMapToOdomAnchor(true, 1.5, 0.1, 10.0, 10.001));
  assert(!ll::shouldFreezeMapToOdomAnchor(true, 1.5, NAN, 10.0, 1.0));
  assert(!ll::shouldFreezeMapToOdomAnchor(true, NAN, 1.0, 10.0, 1.0));
  assert(!ll::shouldFreezeMapToOdomAnchor(true, -1.0, 0.0, 10.0, 1.0));
  assert(!ll::shouldFreezeMapToOdomAnchor(true, 1.5, 1.0, NAN, 1.0));
}

void test_constant_body_motion_extrapolation()
{
  geometry_msgs::msg::TransformStamped previous;
  previous.header.stamp = stamp(1);
  previous.header.frame_id = "odom";
  previous.child_frame_id = "base_link";
  previous.transform.rotation.w = 1.0;

  geometry_msgs::msg::TransformStamped latest = previous;
  latest.header.stamp = stamp(2);
  latest.transform.translation.x = 2.0;

  const auto predicted = ll::extrapolateTransformConstantBodyMotion(
    previous, latest, stamp(3), 2.0);
  assert(predicted.has_value());
  assert(predicted->header.stamp.sec == 3);
  assert(near(predicted->transform.translation.x, 4.0));

  const auto too_far = ll::extrapolateTransformConstantBodyMotion(
    previous, latest, stamp(5), 2.0);
  assert(!too_far.has_value());
}

void test_pose_from_matrix()
{
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix(0, 3) = 1.0f;
  matrix(1, 3) = 2.0f;
  matrix(2, 3) = 3.0f;

  const auto pose = ll::poseFromMatrix(matrix);
  assert(near(pose.position.x, 1.0));
  assert(near(pose.position.y, 2.0));
  assert(near(pose.position.z, 3.0));
  assert(near(pose.orientation.w, 1.0));
}

void test_planar_odom_prediction_constraint()
{
  Eigen::Matrix4f prediction = Eigen::Matrix4f::Identity();
  prediction.block<3, 3>(0, 0) =
    (Eigen::AngleAxisf(0.8f, Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(-0.4f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.3f, Eigen::Vector3f::UnitX())).toRotationMatrix();
  prediction(0, 3) = 10.0f;
  prediction(1, 3) = 20.0f;
  prediction(2, 3) = -25.0f;

  Eigen::Matrix4f accepted = Eigen::Matrix4f::Identity();
  accepted.block<3, 3>(0, 0) =
    (Eigen::AngleAxisf(-0.2f, Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(0.05f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.03f, Eigen::Vector3f::UnitX())).toRotationMatrix();
  accepted(2, 3) = -11.5f;

  const Eigen::Matrix4f constrained =
    ll::constrainOdomPredictionToPlanarMotion(prediction, accepted);
  const float yaw = std::atan2(constrained(1, 0), constrained(0, 0));
  const float pitch = std::asin(-constrained(2, 0));
  const float roll = std::atan2(constrained(2, 1), constrained(2, 2));
  assert(std::abs(constrained(0, 3) - 10.0f) < 1.0e-5f);
  assert(std::abs(constrained(1, 3) - 20.0f) < 1.0e-5f);
  assert(std::abs(constrained(2, 3) + 11.5f) < 1.0e-5f);
  assert(std::abs(roll + 0.03f) < 1.0e-5f);
  assert(std::abs(pitch - 0.05f) < 1.0e-5f);
  assert(std::abs(yaw - 0.8f) < 1.0e-5f);
}

void test_planar_constraint_preserves_negative_yaw_branch()
{
  Eigen::Matrix4f prediction = Eigen::Matrix4f::Identity();
  prediction.block<3, 3>(0, 0) =
    (Eigen::AngleAxisf(-1.45f, Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(-0.02f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.01f, Eigen::Vector3f::UnitX())).toRotationMatrix();
  Eigen::Matrix4f accepted = Eigen::Matrix4f::Identity();
  const Eigen::Matrix4f constrained =
    ll::constrainOdomPredictionToPlanarMotion(prediction, accepted);
  const float yaw = std::atan2(constrained(1, 0), constrained(0, 0));
  assert(std::abs(yaw + 1.45f) < 1.0e-5f);
}

void test_height_only_odom_prediction_constraint()
{
  Eigen::Matrix4f prediction = Eigen::Matrix4f::Identity();
  prediction.block<3, 3>(0, 0) =
    (Eigen::AngleAxisf(-1.2f, Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(0.7f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.9f, Eigen::Vector3f::UnitX())).toRotationMatrix();
  prediction(0, 3) = 4.0f;
  prediction(1, 3) = 5.0f;
  prediction(2, 3) = 30.0f;
  Eigen::Matrix4f anchor = Eigen::Matrix4f::Identity();
  anchor(2, 3) = -11.0f;

  const Eigen::Matrix4f constrained =
    ll::constrainOdomPredictionHeightOnly(prediction, anchor);
  assert((constrained.block<3, 3>(0, 0) - prediction.block<3, 3>(0, 0)).norm() < 1.0e-6f);
  assert(std::abs(constrained(0, 3) - 4.0f) < 1.0e-6f);
  assert(std::abs(constrained(1, 3) - 5.0f) < 1.0e-6f);
  assert(std::abs(constrained(2, 3) + 11.0f) < 1.0e-6f);
}

void test_pose_stamped_and_path_append()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const auto pose = pose_xyz(1.0, 2.0, 3.0);
  const auto pose_stamped = ll::makePoseStamped(stamp(10, 20), "map", pose);

  assert(pose_stamped.header.stamp.sec == 10);
  assert(pose_stamped.header.stamp.nanosec == 20);
  assert(pose_stamped.header.frame_id == "map");
  assert(near(pose_stamped.pose.position.y, 2.0));

  ll::appendPoseToPath(path, pose_stamped);
  assert(path.poses.size() == 1);
  assert(path.poses[0].header.stamp.sec == 10);
}

void test_stamp_pose_with_covariance()
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp.sec = 1;
  pose.pose.pose = pose_xyz(1.0, 2.0, 3.0);

  const auto stamped = ll::stampPoseWithCovariance(pose, stamp(5));
  assert(stamped.header.frame_id == "map");
  assert(stamped.header.stamp.sec == 5);
  assert(near(stamped.pose.pose.position.x, 1.0));
}

void test_map_to_base_transform()
{
  const auto transform = ll::makeMapToBaseTransform(
    stamp(2), "map", "base_link", pose_xyz(4.0, 5.0, 6.0));

  assert(transform.header.stamp.sec == 2);
  assert(transform.header.frame_id == "map");
  assert(transform.child_frame_id == "base_link");
  assert(near(transform.transform.translation.x, 4.0));
  assert(near(transform.transform.rotation.w, 1.0));
}

void test_compose_map_to_odom_transform()
{
  const auto map_to_base = ll::makeMapToBaseTransform(
    stamp(3), "map", "base_link", pose_xyz(10.0, 0.0, 0.0));
  const auto odom_to_base = ll::makeMapToBaseTransform(
    stamp(3), "odom", "base_link", pose_xyz(4.0, 0.0, 0.0));

  const auto map_to_odom = ll::composeMapToOdomTransform(
    stamp(3), "map", "odom", map_to_base, odom_to_base);
  assert(map_to_odom.header.frame_id == "map");
  assert(map_to_odom.child_frame_id == "odom");
  assert(near(map_to_odom.transform.translation.x, 6.0));
  assert(near(map_to_odom.transform.rotation.w, 1.0));
}

int main()
{
  test_accepted_pose_publication_policy();
  test_map_odom_anchor_fitness_gate();
  test_constant_body_motion_extrapolation();
  test_pose_from_matrix();
  test_planar_odom_prediction_constraint();
  test_planar_constraint_preserves_negative_yaw_branch();
  test_height_only_odom_prediction_constraint();
  test_pose_stamped_and_path_append();
  test_stamp_pose_with_covariance();
  test_map_to_base_transform();
  test_compose_map_to_odom_transform();
  return 0;
}
