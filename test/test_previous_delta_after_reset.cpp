// The previous-delta seed is rescaled to the elapsed time only when the stored motion
// runs from one scan pose to the next. After a reset (an initial pose, say) the first
// stored motion is the correction to the first scan over an arbitrary interval: it is
// applied once, as before, not multiplied by that interval's ratio to the next one.
#include <lidar_localization/lidar_localization_component.hpp>

#include <cassert>
#include <cmath>

namespace
{

Eigen::Matrix4f pose_x(float x)
{
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose(0, 3) = x;
  return pose;
}

bool near(float lhs, float rhs)
{
  return std::abs(lhs - rhs) < 1.0e-4f;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  PCLLocalization node{rclcpp::NodeOptions()};
  node.previous_delta_max_extrapolation_sec_ = 1.0;

  // An initial pose stamped 10 ms before the first scan, 0.3 m off.
  node.resetPredictionState(pose_x(0.0f), 10.0);
  node.updatePredictionState(pose_x(0.3f), 10.01);
  assert(!node.last_relative_motion_spans_scans_);
  // The next scan, 0.1 s on: the correction once, not ten times.
  assert(near(node.previousDeltaMotionAt(10.11)(0, 3), 0.3f));

  // Two scans 0.1 s apart: 0.1 m of motion, rescaled to the time since.
  node.updatePredictionState(pose_x(0.4f), 10.11);
  assert(node.last_relative_motion_spans_scans_);
  assert(near(node.previousDeltaMotionAt(10.21)(0, 3), 0.1f));
  assert(near(node.previousDeltaMotionAt(10.81)(0, 3), 0.7f));

  // A rejected scan advances by the rescaled motion too.
  node.consecutive_rejected_updates_ = 0;
  node.advancePredictionWithoutMeasurement(10.51);
  assert(near(node.predicted_pose_matrix_(0, 3), 0.8f));

  // A new reset starts over.
  node.resetPredictionState(pose_x(5.0f), 20.0);
  assert(!node.last_relative_motion_spans_scans_);
  node.updatePredictionState(pose_x(5.2f), 20.001);
  assert(near(node.previousDeltaMotionAt(20.101)(0, 3), 0.2f));

  // With the limit at 0 the stored motion is applied once per scan.
  node.updatePredictionState(pose_x(5.3f), 20.101);
  node.previous_delta_max_extrapolation_sec_ = 0.0;
  assert(near(node.previousDeltaMotionAt(20.801)(0, 3), 0.1f));

  rclcpp::shutdown();
  return 0;
}
