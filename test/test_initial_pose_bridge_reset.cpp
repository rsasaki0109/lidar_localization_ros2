#include <lidar_localization/lidar_localization_component.hpp>
#include <cassert>
#include <limits>

void exercise(bool early_tf, int invalid)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("enable_map_odom_tf", true),
    rclcpp::Parameter("use_odom_tf_prediction", true)});
  PCLLocalization node(options);
  node.configure();
  node.pose_pub_->on_activate();
  node.tfbuffer_.setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = node.odom_frame_id_;
  tf.child_frame_id = node.base_frame_id_;
  tf.header.stamp.sec = 1;
  tf.transform.rotation.w = 1;
  assert(node.tfbuffer_.setTransform(tf, "fixture", false));
  auto pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  pose->header.frame_id = node.global_frame_id_;
  pose->header.stamp.sec = 1;
  pose->pose.pose.orientation.w = 1;
  pose->pose.pose.position.x = 1;
  node.initialPoseReceived(pose);
  assert(node.has_last_good_map_to_odom_);
  node.odom_bridge_transform_history_.push_back(tf);
  node.has_last_odom_bridge_source_advance_node_stamp_ = true;
  node.last_odom_bridge_source_advance_node_stamp_ = tf.header.stamp;
  tf.header.stamp.sec = 2;
  if (early_tf) {assert(node.tfbuffer_.setTransform(tf, "fixture", false));}
  auto reset = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(*pose);
  reset->header.stamp.sec = 2;
  reset->pose.pose.position.x = 10;
  if (invalid == 1) {reset->header.frame_id = "wrong_frame";}
  if (invalid == 2) {reset->pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();}
  node.initialPoseReceived(reset);
  assert(node.last_accepted_pose_matrix_(0, 3) == (invalid ? 1 : 10));
  assert(node.has_last_good_map_to_odom_ == (invalid || early_tf));
  assert(node.has_odom_tf_constraint_anchor_pose_ == (invalid || early_tf));
  assert(node.odom_bridge_transform_history_.empty() == !invalid);
  assert(node.has_last_odom_bridge_source_advance_node_stamp_ == bool(invalid));
  if (!early_tf) {assert(node.tfbuffer_.setTransform(tf, "fixture", false));}
  Eigen::Matrix4f seed = Eigen::Matrix4f::Zero();
  const bool available = node.lookupOdomBridgePoseMatrix(tf.header.stamp, seed, false, nullptr);
  assert(available == (invalid || early_tf));
  if (available) {assert(seed(0, 3) == (invalid ? 1 : 10));}
  assert(node.currentPoseMatrix()(0, 3) == (invalid ? 1 : 10));
  node.pose_pub_->on_deactivate();
  node.cleanup();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  for (int invalid : {0, 1, 2}) {
    exercise(true, invalid);
    exercise(false, invalid);
  }
  rclcpp::shutdown();
}
