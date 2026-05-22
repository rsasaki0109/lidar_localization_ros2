#ifndef LIDAR_LOCALIZATION_ALIGNMENT_DIAGNOSTIC_ROS_ADAPTER_HPP_
#define LIDAR_LOCALIZATION_ALIGNMENT_DIAGNOSTIC_ROS_ADAPTER_HPP_

#include "lidar_localization/alignment_diagnostics_policy.hpp"

#include <diagnostic_msgs/msg/key_value.hpp>

#include <vector>

namespace lidar_localization
{

inline diagnostic_msgs::msg::KeyValue makeRosDiagnosticKeyValue(
  const DiagnosticKeyValue & value)
{
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = value.first;
  key_value.value = value.second;
  return key_value;
}

inline std::vector<diagnostic_msgs::msg::KeyValue> makeRosDiagnosticKeyValues(
  const std::vector<DiagnosticKeyValue> & values)
{
  std::vector<diagnostic_msgs::msg::KeyValue> key_values;
  key_values.reserve(values.size());
  for (const auto & value : values) {
    key_values.push_back(makeRosDiagnosticKeyValue(value));
  }
  return key_values;
}

inline std::vector<diagnostic_msgs::msg::KeyValue> makeRosAlignmentDiagnosticKeyValues(
  const AlignmentDiagnosticValuesInput & input)
{
  return makeRosDiagnosticKeyValues(buildAlignmentDiagnosticValues(input));
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_ALIGNMENT_DIAGNOSTIC_ROS_ADAPTER_HPP_
