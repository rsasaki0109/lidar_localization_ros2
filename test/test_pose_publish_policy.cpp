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
  test_pose_from_matrix();
  test_pose_stamped_and_path_append();
  test_stamp_pose_with_covariance();
  test_map_to_base_transform();
  test_compose_map_to_odom_transform();
  return 0;
}
