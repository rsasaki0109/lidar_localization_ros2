#pragma once
// Temporary experiment instrumentation. No ROS parameter or prediction policy.
#include <Eigen/Core>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>

namespace prediction_trace_experiment
{
template<typename TwistPointer>
inline void record(
  const char * event, double scan_stamp, double prediction_stamp, double dt,
  std::size_t rejected, const TwistPointer & twist,
  const Eigen::Matrix4f & before, const Eigen::Matrix4f & after)
{
  static const char * path = std::getenv("JEPLO_PREDICTION_TRACE");
  if (!path || !*path) {return;}
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);
  static std::ofstream stream(path, std::ios::app);
  if (!stream) {throw std::runtime_error("Cannot write JEPLO_PREDICTION_TRACE");}
  stream << std::setprecision(17) << event << ',' << scan_stamp << ','
         << prediction_stamp << ',' << dt << ',' << rejected;
  if (twist) {
    const auto & v = twist->twist.twist;
    stream << ',' << twist->header.stamp.sec << ',' << twist->header.stamp.nanosec
           << ',' << v.linear.x << ',' << v.linear.y << ',' << v.linear.z
           << ',' << v.angular.x << ',' << v.angular.y << ',' << v.angular.z;
  } else {
    stream << ",nan,nan,nan,nan,nan,nan,nan,nan";
  }
  for (const auto * matrix : {&before, &after}) {
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {stream << ',' << (*matrix)(row, col);}
    }
  }
  stream << '\n';
  // Replay cleanup can terminate the process without running static destructors.
  stream.flush();
  if (!stream) {throw std::runtime_error("Failed to flush JEPLO_PREDICTION_TRACE");}
}
}  // namespace prediction_trace_experiment
