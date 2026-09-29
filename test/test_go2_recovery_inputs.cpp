#include "go2_recovery.hpp"
#include "lidar_localization/lidar_localization_component.hpp"

#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

template<typename F>
void rejects(F operation, const std::string & reason)
{
  try {
    operation();
  } catch (const std::runtime_error & error) {
    require(std::string(error.what()).find(reason) != std::string::npos, error.what());
    return;
  }
  throw std::runtime_error("invalid recovery input was accepted");
}

sensor_msgs::msg::PointCloud2 cloud()
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "livox_frame";
  msg.header.stamp.sec = 10;
  msg.width = msg.height = 1;
  msg.point_step = msg.row_step = 20;
  for (const char * name : {"x", "y", "z", "t"}) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = msg.fields.size() * 4;
    field.count = 1;
    field.datatype = field.name == "t" ? field.FLOAT64 : field.FLOAT32;
    msg.fields.push_back(field);
  }
  msg.data.resize(20);
  const float xyz[] = {1.0f, 0.0f, 1.0f};
  const double time = 10.0;
  std::memcpy(msg.data.data(), xyz, sizeof(xyz));
  std::memcpy(msg.data.data() + 12, &time, sizeof(time));
  return msg;
}

void lifecycleChecks()
{
  rclcpp::init(0, nullptr);
  auto options = []() {
      return rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("enable_go2_confirmed_recovery", true),
        rclcpp::Parameter("registration_method", "NDT_OMP"),
        rclcpp::Parameter("base_frame_id", "livox_frame"),
        rclcpp::Parameter("use_imu_preintegration", false),
        rclcpp::Parameter("use_imu", false),
        rclcpp::Parameter("use_twist_ekf", false),
        rclcpp::Parameter("use_gtsam_smoother", false),
        rclcpp::Parameter("use_pcd_map", false),
        rclcpp::Parameter("set_initial_pose", false),
        rclcpp::Parameter("use_bond", false)});
    };
  {
    auto startup_options = options();
    for (auto & parameter : startup_options.parameter_overrides()) {
      if (parameter.get_name() == "set_initial_pose") {
        parameter = rclcpp::Parameter("set_initial_pose", true);
      }
    }
    startup_options.append_parameter_override("initial_pose_x", 2.0);
    startup_options.append_parameter_override("initial_pose_qw", 1.0);
    PCLLocalization node(startup_options);
    using Return = PCLLocalization::CallbackReturn;
    require(node.on_configure(rclcpp_lifecycle::State()) == Return::SUCCESS, "configure failed");
    auto context = std::atomic_load(&node.go2_recovery_);
    auto old_imu = node.imu_sub_;
    auto receive = [&](int sec) {
        auto msg = std::make_shared<sensor_msgs::msg::Imu>();
        msg->header.frame_id = "livox_frame";
        msg->header.stamp.sec = sec;
        std::shared_ptr<void> erased = msg;
        std::const_pointer_cast<rclcpp::Subscription<sensor_msgs::msg::Imu>>(old_imu)->
        handle_message(erased, rclcpp::MessageInfo());
      };
    receive(1);
    require(context->imu_history.empty(), "configured inactive node accepted recovery IMU");
    require(node.on_activate(rclcpp_lifecycle::State()) == Return::SUCCESS, "activate failed");
    require(node.initialpose_recieved_, "activation ignored configured initial pose");
    require(node.last_accepted_pose_matrix_(0, 3) == 2.0f,
        "activation lost configured pose position");
    receive(2);
    require(context->imu_history.size() == 1, "active IMU missing");
    context->tried = context->pending = true;
    context->previous = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud());
    context->previous_imu.assign(context->imu_history.begin(), context->imu_history.end());
    const auto generation = node.callback_state_coordinator_.initialPoseGeneration();
    require(node.on_deactivate(rclcpp_lifecycle::State()) == Return::SUCCESS, "deactivate failed");
    require(!context->pending && !context->tried && !context->previous &&
      context->previous_imu.empty() && context->imu_history.empty(),
        "deactivate retained recovery state");
    require(node.callback_state_coordinator_.initialPoseGeneration() > generation,
        "old work not cancelled");
    receive(3);
    require(context->imu_history.empty(), "inactive node accepted recovery IMU");
    require(node.on_activate(rclcpp_lifecycle::State()) == Return::SUCCESS, "reactivate failed");
    receive(4);
    require(context->imu_history.size() == 1, "reactivate did not resume IMU");
    node.on_cleanup(rclcpp_lifecycle::State());
    require(!std::atomic_load(&node.go2_recovery_), "cleanup retained context");
    require(node.on_configure(rclcpp_lifecycle::State()) == Return::SUCCESS, "reconfigure failed");
    require(node.on_activate(rclcpp_lifecycle::State()) == Return::SUCCESS,
        "second activation failed");
    auto fresh = std::atomic_load(&node.go2_recovery_);
    require(fresh && fresh != context, "reconfigure reused old context");
    const auto count = context->imu_history.size();
    receive(5);
    require(fresh->imu_history.empty() && context->imu_history.size() == count,
        "stale callback mutated recovery");
    node.on_shutdown(rclcpp_lifecycle::State());
  }
  {
    auto failed_options = options();
    for (auto & parameter : failed_options.parameter_overrides()) {
      if (parameter.get_name() == "use_pcd_map") {
        parameter = rclcpp::Parameter("use_pcd_map", true);
      }
    }
    failed_options.append_parameter_override("map_path", "/nonexistent/go2_recovery_test_map.pcd");
    PCLLocalization node(failed_options);
    require(node.on_configure(rclcpp_lifecycle::State()) ==
        PCLLocalization::CallbackReturn::SUCCESS, "failure fixture configure");
    require(node.on_activate(rclcpp_lifecycle::State()) == PCLLocalization::CallbackReturn::FAILURE,
        "missing map activation succeeded");
    require(node.shutting_down_.load(), "failed activation left recovery callbacks enabled");
  }
  for (const auto & incompatible : std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("base_frame_id", "base_link"),
      rclcpp::Parameter("registration_method", "GICP"),
      rclcpp::Parameter("use_imu", true),
      rclcpp::Parameter("use_imu_preintegration", true),
      rclcpp::Parameter("use_twist_ekf", true),
      rclcpp::Parameter("use_gtsam_smoother", true)})
  {
    auto configured = options();
    for (auto & parameter : configured.parameter_overrides()) {
      if (parameter.get_name() == incompatible.get_name()) {parameter = incompatible;}
    }
    PCLLocalization node(configured);
    bool rejected = false;
    try {
      node.on_configure(rclcpp_lifecycle::State());
    } catch (const std::invalid_argument & error) {
      rejected = std::string(error.what()).find("Go2 confirmed recovery requires") !=
        std::string::npos;
    }
    require(rejected, "unsupported recovery configuration accepted");
  }
  rclcpp::shutdown();
}
}  // namespace

int main()
{
  try {
    Go2Recovery state;
    state.reset(7);
    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = "livox_frame";
    msg.header.stamp.sec = 10;
    go2ReceiveImu(msg, state, 7);
    auto history = go2ImuSnapshot(std::numeric_limits<int64_t>::max(), state, 7);
    require(history.size() == 1, "valid IMU missing");
    auto invalid_time = msg;
    invalid_time.header.stamp.sec = -1;
    go2ReceiveImu(invalid_time, state, 7);
    invalid_time = msg;
    invalid_time.header.stamp.nanosec = 1000000000U;
    go2ReceiveImu(invalid_time, state, 7);
    const auto after_invalid_time = go2ImuSnapshot(go2RecoveryNow(), state, 7);
    require(after_invalid_time.size() == 1 &&
      after_invalid_time[0].stamp == history[0].stamp &&
      after_invalid_time[0].receipt == history[0].receipt,
      "invalid timestamp changed IMU history");
    require(go2ImuSnapshot(history[0].receipt - 1, state, 7).empty(), "future receipt leaked");
    go2ReceiveImu(msg, state, 7);
    require(go2ImuSnapshot(go2RecoveryNow(), state, 7).size() == 1, "duplicate IMU accepted");
    msg.header.stamp.nanosec = 1;
    msg.angular_velocity.x = std::numeric_limits<double>::quiet_NaN();
    go2ReceiveImu(msg, state, 7);
    msg.angular_velocity.x = 0;
    msg.header.frame_id = "other_frame";
    go2ReceiveImu(msg, state, 7);
    require(go2ImuSnapshot(go2RecoveryNow(), state, 7).size() == 1, "invalid IMU accepted");
    msg.header.frame_id = "livox_frame";
    for (uint32_t i = 1; i <= 5000; ++i) {
      msg.header.stamp.nanosec = i;
      go2ReceiveImu(msg, state, 7);
    }
    require(go2ImuSnapshot(go2RecoveryNow(), state, 7).size() == 4096, "history cap violated");
    msg.header.stamp.sec = 13;
    go2ReceiveImu(msg, state, 7);
    require(go2ImuSnapshot(go2RecoveryNow(), state, 7).size() == 1, "history age cap violated");
    state.tried = state.pending = true;
    state.previous = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud());
    state.previous_imu = history;
    state.reset(8);
    require(!state.tried && !state.pending && !state.previous && state.previous_imu.empty(),
      "reset retained candidate");
    go2ReceiveImu(msg, state, 7);
    require(go2ImuSnapshot(go2RecoveryNow(), state, 8).empty(),
      "old generation repopulated history");

    auto valid = cloud();
    auto truncated = valid;
    truncated.data.pop_back();
    rejects([&]() {go2Delta(truncated, valid, history, history);}, "valid little-endian");
    auto wrong_fields = valid;
    wrong_fields.fields.back().offset = 18;
    rejects([&]() {go2Delta(wrong_fields, valid, history, history);}, "float32 xyz");
    auto wrong_frame = valid;
    wrong_frame.header.frame_id = "base_link";
    rejects([&]() {go2Delta(wrong_frame, valid, history, history);}, "livox_frame");
    rejects([&]() {go2Delta(valid, valid, {}, {});}, "empty deskew input");
    auto stale = history;
    stale[0].stamp -= 30000000;
    rejects([&]() {go2Delta(valid, valid, stale, stale);}, "IMU coverage");
    rejects([&]() {go2Delta(valid, valid, history, history);}, "insufficient points");
    const Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    rejects([&]() {go2BbsSeeds("absent.pcd", truncated, pose, pose);}, "valid little-endian");
    lifecycleChecks();
    std::cout << "Go2 input, causal history, lifecycle and mode checks passed\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
