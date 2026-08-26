#ifndef LIDAR_LOCALIZATION_POINT_CLOUD_CONVERSION_HPP_
#define LIDAR_LOCALIZATION_POINT_CLOUD_CONVERSION_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "lidar_localization/point_field_read.hpp"

namespace lidar_localization
{

struct PointTimeRange
{
  bool has_time_field{false};
  bool valid{false};
  std::string field_name;
  double min_time_sec{std::numeric_limits<double>::quiet_NaN()};
  double max_time_sec{std::numeric_limits<double>::quiet_NaN()};
  std::size_t valid_point_count{0};
  std::size_t invalid_point_count{0};

  double durationSec() const
  {
    return valid ? max_time_sec - min_time_sec : std::numeric_limits<double>::quiet_NaN();
  }
};

struct PointRelativeTimes
{
  bool has_time_field{false};
  bool valid{false};
  std::string field_name;
  double reference_time_sec{std::numeric_limits<double>::quiet_NaN()};
  double duration_sec{std::numeric_limits<double>::quiet_NaN()};
  std::size_t valid_point_count{0};
  std::size_t invalid_point_count{0};
  std::vector<double> relative_times_sec;

  bool hasCompleteTimes() const
  {
    return valid &&
           invalid_point_count == 0 &&
           relative_times_sec.size() == valid_point_count;
  }
};

struct TimedXyziCloud
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  std::vector<double> relative_times_sec;
  bool has_time_field{false};
  bool time_range_valid{false};
  std::size_t valid_time_count{0};
  std::size_t invalid_time_count{0};

  bool timeVectorAlignedWithCloud() const
  {
    return !has_time_field || relative_times_sec.size() == cloud.size();
  }

  bool hasCompleteTimesForCloud() const
  {
    return has_time_field &&
           time_range_valid &&
           relative_times_sec.size() == cloud.size() &&
           invalid_time_count == 0;
  }
};


inline PointTimeRange computePointTimeRangeSeconds(const sensor_msgs::msg::PointCloud2 & input)
{
  PointTimeRange range;
  const auto * time_field = findPointTimeField(input.fields);
  if (!time_field) {
    return range;
  }
  range.has_time_field = true;
  range.field_name = time_field->name;

  const std::size_t point_count =
    static_cast<std::size_t>(input.width) * static_cast<std::size_t>(input.height);
  if (!pointFieldFitsPointStep(*time_field, input.point_step) || input.point_step == 0) {
    range.invalid_point_count = point_count;
    return range;
  }

  const std::size_t time_field_size = pointFieldDatatypeSize(time_field->datatype);
  const std::size_t required_size = point_count == 0 ?
    0 :
    (point_count - 1) * static_cast<std::size_t>(input.point_step) +
    static_cast<std::size_t>(time_field->offset) + time_field_size;
  if (input.data.size() < required_size) {
    range.invalid_point_count = point_count;
    return range;
  }

  for (std::size_t point_idx = 0; point_idx < point_count; ++point_idx) {
    const uint8_t * point_data =
      input.data.data() + point_idx * static_cast<std::size_t>(input.point_step);
    double time_seconds = 0.0;
    if (!readPointTimeSeconds(point_data, *time_field, &time_seconds) ||
      !std::isfinite(time_seconds))
    {
      ++range.invalid_point_count;
      continue;
    }
    if (!range.valid) {
      range.min_time_sec = time_seconds;
      range.max_time_sec = time_seconds;
      range.valid = true;
    } else {
      range.min_time_sec = std::min(range.min_time_sec, time_seconds);
      range.max_time_sec = std::max(range.max_time_sec, time_seconds);
    }
    ++range.valid_point_count;
  }
  return range;
}

inline PointRelativeTimes extractPointRelativeTimesSeconds(
  const sensor_msgs::msg::PointCloud2 & input)
{
  PointRelativeTimes times;
  const auto * time_field = findPointTimeField(input.fields);
  if (!time_field) {
    return times;
  }
  times.has_time_field = true;
  times.field_name = time_field->name;

  const std::size_t point_count =
    static_cast<std::size_t>(input.width) * static_cast<std::size_t>(input.height);
  times.relative_times_sec.assign(
    point_count, std::numeric_limits<double>::quiet_NaN());
  if (!pointFieldFitsPointStep(*time_field, input.point_step) || input.point_step == 0) {
    times.invalid_point_count = point_count;
    return times;
  }

  const std::size_t time_field_size = pointFieldDatatypeSize(time_field->datatype);
  const std::size_t required_size = point_count == 0 ?
    0 :
    (point_count - 1) * static_cast<std::size_t>(input.point_step) +
    static_cast<std::size_t>(time_field->offset) + time_field_size;
  if (input.data.size() < required_size) {
    times.invalid_point_count = point_count;
    return times;
  }

  double min_time_sec = std::numeric_limits<double>::quiet_NaN();
  double max_time_sec = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t point_idx = 0; point_idx < point_count; ++point_idx) {
    const uint8_t * point_data =
      input.data.data() + point_idx * static_cast<std::size_t>(input.point_step);
    double time_seconds = 0.0;
    if (!readPointTimeSeconds(point_data, *time_field, &time_seconds) ||
      !std::isfinite(time_seconds))
    {
      ++times.invalid_point_count;
      continue;
    }
    times.relative_times_sec[point_idx] = time_seconds;
    if (!times.valid) {
      min_time_sec = time_seconds;
      max_time_sec = time_seconds;
      times.valid = true;
    } else {
      min_time_sec = std::min(min_time_sec, time_seconds);
      max_time_sec = std::max(max_time_sec, time_seconds);
    }
    ++times.valid_point_count;
  }

  if (!times.valid) {
    return times;
  }
  times.reference_time_sec = min_time_sec;
  times.duration_sec = max_time_sec - min_time_sec;
  for (double & time_seconds : times.relative_times_sec) {
    if (std::isfinite(time_seconds)) {
      time_seconds -= min_time_sec;
    }
  }
  return times;
}

inline double relativeTimeOrNaN(
  const std::vector<double> & relative_times_sec,
  std::size_t point_idx)
{
  if (point_idx >= relative_times_sec.size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return relative_times_sec[point_idx];
}

inline double relativeTimeOrNaN(
  const PointRelativeTimes & relative_times,
  std::size_t point_idx)
{
  return relativeTimeOrNaN(relative_times.relative_times_sec, point_idx);
}

inline void appendRelativeTime(
  double relative_time_sec,
  TimedXyziCloud & output)
{
  if (!output.has_time_field) {
    return;
  }
  output.relative_times_sec.push_back(relative_time_sec);
  if (std::isfinite(relative_time_sec)) {
    ++output.valid_time_count;
  } else {
    ++output.invalid_time_count;
  }
}

inline void appendRelativeTimeForSourcePoint(
  const PointRelativeTimes & source_times,
  std::size_t source_point_idx,
  TimedXyziCloud & output)
{
  appendRelativeTime(relativeTimeOrNaN(source_times, source_point_idx), output);
}

inline void copyXyzToXyzi(
  const pcl::PointCloud<pcl::PointXYZ> & input,
  pcl::PointCloud<pcl::PointXYZI> & output)
{
  output.clear();
  output.reserve(input.size());
  output.header = input.header;
  output.width = input.width;
  output.height = input.height;
  output.is_dense = input.is_dense;

  for (const auto & point : input.points) {
    pcl::PointXYZI converted;
    converted.x = point.x;
    converted.y = point.y;
    converted.z = point.z;
    converted.intensity = 0.0f;
    output.push_back(converted);
  }
}

inline TimedXyziCloud convertSensorCloudToTimedXyzi(
  const sensor_msgs::msg::PointCloud2 & input,
  const PointRelativeTimes & source_times)
{
  TimedXyziCloud output;
  output.has_time_field = source_times.has_time_field;
  output.time_range_valid = source_times.valid;

  const auto * x_field = findPointField(input.fields, "x");
  const auto * y_field = findPointField(input.fields, "y");
  const auto * z_field = findPointField(input.fields, "z");
  const auto * intensity_field = findPointField(input.fields, "intensity");
  pcl_conversions::toPCL(input.header, output.cloud.header);
  if (!x_field || !y_field || !z_field) {
    output.cloud.clear();
    output.cloud.width = 0;
    output.cloud.height = 1;
    output.cloud.is_dense = false;
    return output;
  }

  output.cloud.clear();
  output.cloud.reserve(static_cast<std::size_t>(input.width) * static_cast<std::size_t>(input.height));
  output.cloud.is_dense = input.is_dense;

  const std::size_t point_count =
    static_cast<std::size_t>(input.width) * static_cast<std::size_t>(input.height);
  for (std::size_t point_idx = 0; point_idx < point_count; ++point_idx) {
    const uint8_t * point_data = input.data.data() + point_idx * input.point_step;
    pcl::PointXYZI point;
    float intensity = 0.0f;
    if (!readPointFieldAsFloat(point_data, *x_field, &point.x) ||
      !readPointFieldAsFloat(point_data, *y_field, &point.y) ||
      !readPointFieldAsFloat(point_data, *z_field, &point.z))
    {
      continue;
    }
    if (intensity_field) {
      readPointFieldAsFloat(point_data, *intensity_field, &intensity);
    }
    point.intensity = intensity;
    output.cloud.push_back(point);
    appendRelativeTimeForSourcePoint(source_times, point_idx, output);
  }
  return output;
}

inline void convertSensorCloudToXyzi(
  const sensor_msgs::msg::PointCloud2 & input,
  pcl::PointCloud<pcl::PointXYZI> & output)
{
  const auto * x_field = findPointField(input.fields, "x");
  const auto * y_field = findPointField(input.fields, "y");
  const auto * z_field = findPointField(input.fields, "z");
  const auto * intensity_field = findPointField(input.fields, "intensity");
  if (!x_field || !y_field || !z_field) {
    output.clear();
    output.width = 0;
    output.height = 1;
    output.is_dense = false;
    pcl_conversions::toPCL(input.header, output.header);
    return;
  }

  output.clear();
  output.reserve(static_cast<std::size_t>(input.width) * static_cast<std::size_t>(input.height));
  pcl_conversions::toPCL(input.header, output.header);
  output.is_dense = input.is_dense;

  const std::size_t point_count =
    static_cast<std::size_t>(input.width) * static_cast<std::size_t>(input.height);
  for (std::size_t point_idx = 0; point_idx < point_count; ++point_idx) {
    const uint8_t * point_data = input.data.data() + point_idx * input.point_step;
    pcl::PointXYZI point;
    float intensity = 0.0f;
    if (!readPointFieldAsFloat(point_data, *x_field, &point.x) ||
      !readPointFieldAsFloat(point_data, *y_field, &point.y) ||
      !readPointFieldAsFloat(point_data, *z_field, &point.z))
    {
      continue;
    }
    if (intensity_field) {
      readPointFieldAsFloat(point_data, *intensity_field, &intensity);
    }
    point.intensity = intensity;
    output.push_back(point);
  }
}

inline bool convertPclCloudToXyzi(
  const pcl::PCLPointCloud2 & input,
  pcl::PointCloud<pcl::PointXYZI> & output)
{
  if (hasPointField(input.fields, "intensity")) {
    pcl::fromPCLPointCloud2(input, output);
    return true;
  }

  pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
  pcl::fromPCLPointCloud2(input, xyz_cloud);
  copyXyzToXyzi(xyz_cloud, output);
  return false;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_POINT_CLOUD_CONVERSION_HPP_
