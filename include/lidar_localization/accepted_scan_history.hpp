#pragma once

#include <cmath>
#include <deque>
#include <limits>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_localization
{
// Stores original prepared scans, never recursively accumulated clouds.
class AcceptedScanHistory
{
public:
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;

  void clear()
  {
    entries_.clear();
    last_scan_stamp_ = std::numeric_limits<double>::quiet_NaN();
  }

  void beginScan(double stamp)
  {
    if (!std::isfinite(stamp) ||
      (std::isfinite(last_scan_stamp_) && stamp <= last_scan_stamp_))
    {
      clear();
    }
    last_scan_stamp_ = stamp;
    while (!entries_.empty() && stamp - entries_.front().stamp > 0.300001) {
      entries_.pop_front();
    }
  }

  Cloud::Ptr combine(const Cloud::Ptr & current, const Eigen::Matrix4f & seed) const
  {
    if (entries_.empty() || !current || !seed.allFinite()) {
      return current;
    }
    Cloud::Ptr combined(new Cloud(*current));
    const Eigen::Matrix4f inverse = seed.inverse();
    for (const auto & entry : entries_) {
      Cloud transformed;
      pcl::transformPointCloud(*entry.cloud, transformed, inverse * entry.pose);
      *combined += transformed;
    }
    return combined;
  }

  void accept(
    double stamp, const Cloud::ConstPtr & cloud, const Eigen::Matrix4f & pose,
    bool supported_by_history = false)
  {
    // A history-supported pose must not replenish its own supporting history.
    if (supported_by_history || !std::isfinite(stamp) || stamp != last_scan_stamp_ ||
      !cloud || cloud->empty() || !pose.allFinite())
    {
      return;
    }
    if (!entries_.empty() && stamp <= entries_.back().stamp) {
      return;
    }
    entries_.push_back({stamp, cloud, pose});
    while (entries_.size() > 3) {
      entries_.pop_front();
    }
  }

  std::size_t size() const {return entries_.size();}

private:
  struct Entry
  {
    double stamp;
    Cloud::ConstPtr cloud;
    Eigen::Matrix4f pose;
  };
  std::deque<Entry> entries_;
  double last_scan_stamp_{std::numeric_limits<double>::quiet_NaN()};
};
}  // namespace lidar_localization
