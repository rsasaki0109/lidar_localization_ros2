#ifndef LIDAR_LOCALIZATION_TRANSFORM_LISTENER_COMPAT_HPP_
#define LIDAR_LOCALIZATION_TRANSFORM_LISTENER_COMPAT_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#ifdef LIDAR_LOCALIZATION_FOXY_TF_LISTENER
#include <atomic>
#include <chrono>
#include <cstdint>
#include <sstream>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#endif

namespace lidar_localization
{

#ifndef LIDAR_LOCALIZATION_FOXY_TF_LISTENER
using TransformListener = tf2_ros::TransformListener;
#else
// Foxy's tf2_ros::TransformListener(buffer) destroys its internal node before it
// stops the thread spinning that node, and can cancel the spin before it starts:
// a node constructed and destroyed repeatedly then crashes in a parameter-event
// callback or hangs on join a few percent of the time. This listener owns the
// same kind of internal node and stops its thread before anything is destroyed.
// Later distributions fixed the upstream class, which they use unchanged.
class TransformListener
{
public:
  explicit TransformListener(tf2_ros::Buffer & buffer)
  : node_(makeNode(this)), listener_(buffer, node_, false)
  {
    executor_.add_node(node_);
    thread_ = std::thread([this]() {
        while (!stop_.load() && rclcpp::ok()) {
          executor_.spin_once(std::chrono::milliseconds(100));
        }
      });
    buffer.setUsingDedicatedThread(true);
  }

  ~TransformListener()
  {
    stop_.store(true);
    thread_.join();
    executor_.remove_node(node_);
  }

  TransformListener(const TransformListener &) = delete;
  TransformListener & operator=(const TransformListener &) = delete;

private:
  static rclcpp::Node::SharedPtr makeNode(const void * owner)
  {
    std::ostringstream name;
    name << "transform_listener_impl_" << std::hex << reinterpret_cast<std::uintptr_t>(owner);
    return rclcpp::Node::make_shared(
      name.str(),
      rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::TransformListener listener_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};
#endif

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_TRANSFORM_LISTENER_COMPAT_HPP_
