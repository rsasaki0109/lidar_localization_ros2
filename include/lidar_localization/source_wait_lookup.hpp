// Copyright 2026 Sasaki
// SPDX-License-Identifier: BSD-2-Clause
#pragma once

#include "lidar_localization/source_wait_policy.hpp"
#include <tf2_ros/buffer.hpp>
#include <string>

namespace lidar_localization
{
// Experimental adapter. The caller serializes lookup/reset with its existing
// state lock. No cached transform is returned in place of an exact lookup.
class SourceWaitLookup
{
public:
  void reset() { policy_.reset(); }

  geometry_msgs::msg::TransformStamped lookup(
    tf2_ros::Buffer & buffer, const std::string & target, const std::string & source,
    const builtin_interfaces::msg::Time & stamp, const rclcpp::Duration & timeout)
  {
    if (target != target_ || source != source_) {
      reset();
      target_ = target;
      source_ = source;
    }
    try {
      auto result = buffer.lookupTransform(target, source, stamp);
      reset();
      return result;
    } catch (const tf2::TransformException &) {
      if (!policy_.allowWait(latestStamp(buffer, target, source))) {
        throw;
      }
    }
    try {
      auto result = buffer.lookupTransform(target, source, stamp, timeout);
      reset();
      return result;
    } catch (const tf2::TransformException &) {
      policy_.recordTimeout(latestStamp(buffer, target, source));
      throw;
    }
  }

private:
  static SourceWaitPolicy::SourceStamp latestStamp(
    tf2_ros::Buffer & buffer, const std::string & target, const std::string & source)
  {
    try {
      return rclcpp::Time(buffer.lookupTransform(target, source, tf2::TimePointZero)
        .header.stamp).nanoseconds();
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  SourceWaitPolicy policy_;
  std::string target_;
  std::string source_;
};
}  // namespace lidar_localization
