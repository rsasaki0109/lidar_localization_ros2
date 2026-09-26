#include <lidar_localization/lidar_localization_component.hpp>
#include <cassert>
#include <chrono>
#include <future>

using namespace std::chrono_literals;
using Message = geometry_msgs::msg::TwistWithCovarianceStamped;
using Subscription = rclcpp::Subscription<Message>;

void receive(const std::shared_ptr<Subscription> & subscription, double velocity)
{
  auto message = std::make_shared<Message>();
  message->header.stamp.sec = 1;
  message->header.stamp.nanosec = 500000000;
  message->twist.twist.linear.x = velocity;
  std::shared_ptr<void> erased_message = message;
  subscription->handle_message(erased_message, rclcpp::MessageInfo());
}

std::shared_ptr<Subscription> subscription(PCLLocalization & node)
{
  return std::const_pointer_cast<Subscription>(node.twist_sub_);
}

void assertEmptyHistory(PCLLocalization & node)
{
  std::lock_guard<std::mutex> lock(node.twist_history_mutex_);
  assert(!node.twist_history_.atOrBefore(2.0));
}

void exerciseRoute(const char * backend)
{
  rclcpp::NodeOptions options;
  if (backend) {options.parameter_overrides({rclcpp::Parameter(backend, true)});}
  auto node = std::make_shared<PCLLocalization>(options);
  node->configure();
  auto old_subscription = subscription(*node);
  assert(old_subscription);
  {
    auto state_lock = node->callback_state_coordinator_.lockState();
    node->twist_ekf_.initialize(0, 0, 0, 0, 1.0);
    node->gtsam_smoother_.initialize(0, 0, 0, 0, 1.0);
    std::promise<void> started;
    auto started_future = started.get_future();
    auto received = std::async(std::launch::async, [&] {
      started.set_value();
      receive(old_subscription, 1.0);
    });
    started_future.wait();
    const auto status = received.wait_for(backend ? 100ms : 2s);
    // Direct history can progress with state held; pose backend updates cannot.
    const bool expected_status = status == (
      backend ? std::future_status::timeout : std::future_status::ready);
    assert(node->twist_ekf_.px() == 0.0);
    assert(node->gtsam_smoother_.predictedPoseMatrix(0, 0)(0, 3) == 0.0f);
    state_lock.unlock();
    received.get();
    assert(expected_status);
  }
  if (node->use_twist_ekf_) {assert(node->twist_ekf_.px() == 0.5);}
  if (node->use_gtsam_smoother_) {
    assert(node->gtsam_smoother_.predictedPoseMatrix(0, 0)(0, 3) == 0.5f);
  }
  // Exercise actual lifecycle transitions and actual saved subscription closure.
  node->cleanup();
  receive(old_subscription, 999.0);
  assertEmptyHistory(*node);
  node->configure();
  node->twist_ekf_.initialize(0, 0, 0, 0, 1.0);
  node->gtsam_smoother_.initialize(0, 0, 0, 0, 1.0);
  receive(old_subscription, 999.0);
  assertEmptyHistory(*node);
  assert(node->twist_ekf_.px() == 0.0);
  assert(node->gtsam_smoother_.predictedPoseMatrix(0, 0)(0, 3) == 0.0f);
  receive(subscription(*node), 2.0);
  {
    std::lock_guard<std::mutex> lock(node->twist_history_mutex_);
    assert(node->twist_history_.atOrBefore(2.0)->linear.x() == 2.0);
  }
  if (node->use_twist_ekf_) {assert(node->twist_ekf_.px() == 1.0);}
  if (node->use_gtsam_smoother_) {
    assert(node->gtsam_smoother_.predictedPoseMatrix(0, 0)(0, 3) == 1.0f);
  }
  node->cleanup();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  exerciseRoute(nullptr);
  exerciseRoute("use_twist_ekf");
  exerciseRoute("use_gtsam_smoother");
  rclcpp::shutdown();
}
