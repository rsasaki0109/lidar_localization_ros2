// Copyright 2026 Sasaki
// SPDX-License-Identifier: BSD-2-Clause
#include <lidar_localization/lidar_localization_component.hpp>
#include <cassert>

struct Fixture
{
  PCLLocalization node{rclcpp::NodeOptions()};
  Fixture()
  {
    node.configure();
    node.enable_map_odom_tf_ = true;
    node.pose_pub_->on_activate();
    node.tfbuffer_.setUsingDedicatedThread(true);
    tf(1);
    assert(publish(1, 1.0));
  }
  ~Fixture() {if (node.pose_pub_) {node.pose_pub_->on_deactivate(); node.cleanup();}}
  void tf(int sec)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = node.odom_frame_id_; t.child_frame_id = node.base_frame_id_;
    t.header.stamp.sec = sec; t.transform.rotation.w = 1.0;
    assert(node.tfbuffer_.setTransform(t, "fixture", false));
  }
  bool publish(int sec, double x, bool trusted = true)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = node.global_frame_id_; t.child_frame_id = node.base_frame_id_;
    t.header.stamp.sec = sec; t.transform.rotation.w = 1.0; t.transform.translation.x = x;
    return node.publishMapToOdomTransform(t.header.stamp, t, trusted);
  }
  void next(int sec)
  {
    builtin_interfaces::msg::Time stamp; stamp.sec = sec;
    node.republishFrozenMapToOdomTransform(stamp);
  }
  double anchor() const {return node.last_good_map_to_odom_.transform.translation.x;}
};
int main()
{
  rclcpp::init(0, nullptr);
  {Fixture f; f.tf(2); assert(f.publish(2, 10)); f.next(2);
    assert(f.anchor() == 10 && !f.node.pending_map_to_odom_anchor_);}
  {Fixture f; assert(!f.publish(2, 10)); assert(f.node.pending_map_to_odom_anchor_);
    f.tf(2); f.next(2); assert(f.anchor() == 10 && !f.node.pending_map_to_odom_anchor_);}
  {Fixture f; assert(!f.publish(2, 10, false)); f.tf(2); f.next(2);
    assert(f.anchor() == 1 && !f.node.pending_map_to_odom_anchor_);}
  {Fixture f; assert(!f.publish(2, 10)); f.next(2);
    assert(!f.node.pending_map_to_odom_anchor_); f.tf(2); f.next(2); assert(f.anchor() == 1);}
  {Fixture f; assert(!f.publish(2, 10)); f.tf(3); assert(f.publish(3, 20));
    f.next(3); assert(f.anchor() == 20 && !f.node.pending_map_to_odom_anchor_);}
  {Fixture f; assert(!f.publish(2, 10)); assert(!f.publish(3, 20));
    f.tf(3); f.next(3); assert(f.anchor() == 20);}
  for (bool valid : {false, true}) {
    Fixture f; assert(!f.publish(2, 10));
    auto reset = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    reset->header.stamp.sec = 3;
    reset->header.frame_id = valid ? f.node.global_frame_id_ : "wrong_frame";
    reset->pose.pose.orientation.w = 1.0; reset->pose.pose.position.x = 30.0;
    f.tf(3); f.node.initialPoseReceived(reset);
    assert(bool(f.node.pending_map_to_odom_anchor_) == !valid);
    f.next(3); assert(f.anchor() == (valid ? 30.0 : 10.0));
  }
  {Fixture f; assert(!f.publish(2, 10)); assert(!f.publish(3, 20, false));
    f.tf(3); f.next(3); assert(f.anchor() == 10);}
  {Fixture f; assert(!f.publish(2, 10));
    auto reset = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    reset->header.stamp.sec = 3; reset->header.frame_id = f.node.global_frame_id_;
    reset->pose.pose.orientation.w = 1.0; reset->pose.pose.position.x = 30.0;
    f.node.initialPoseReceived(reset);
    assert(f.node.pending_map_to_odom_anchor_->header.stamp.sec == 3);
    f.tf(3); f.next(3); assert(f.anchor() == 30);}
  {Fixture f; assert(!f.publish(2, 10)); f.node.cleanup();
    assert(!f.node.pending_map_to_odom_anchor_);}
  rclcpp::shutdown();
}
