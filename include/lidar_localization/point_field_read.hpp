#ifndef LIDAR_LOCALIZATION_POINT_FIELD_READ_HPP_
#define LIDAR_LOCALIZATION_POINT_FIELD_READ_HPP_

// Low-level sensor_msgs::msg::PointCloud2 field lookup and typed reading
// primitives shared by the point cloud conversion helpers.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>

namespace lidar_localization
{

constexpr std::array<const char *, 4> kPointTimeFieldNames{{
  "time", "t", "timestamp", "offset_time"}};

template<typename FieldContainerT>
bool hasPointField(const FieldContainerT & fields, const std::string & field_name)
{
  return std::any_of(
    fields.begin(), fields.end(),
    [&field_name](const auto & field) {
      return field.name == field_name;
    });
}

inline const sensor_msgs::msg::PointField * findPointField(
  const std::vector<sensor_msgs::msg::PointField> & fields,
  const std::string & field_name)
{
  const auto it = std::find_if(
    fields.begin(), fields.end(),
    [&field_name](const auto & field) {
      return field.name == field_name;
    });
  return it == fields.end() ? nullptr : &(*it);
}

inline const sensor_msgs::msg::PointField * findPointTimeField(
  const std::vector<sensor_msgs::msg::PointField> & fields)
{
  for (const char * field_name : kPointTimeFieldNames) {
    if (const auto * field = findPointField(fields, field_name)) {
      return field;
    }
  }
  return nullptr;
}

inline bool readPointFieldAsFloat(
  const uint8_t * point_data,
  const sensor_msgs::msg::PointField & field,
  float * value)
{
  const uint8_t * field_ptr = point_data + field.offset;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::INT8:
      *value = static_cast<float>(*reinterpret_cast<const int8_t *>(field_ptr));
      return true;
    case sensor_msgs::msg::PointField::UINT8:
      *value = static_cast<float>(*reinterpret_cast<const uint8_t *>(field_ptr));
      return true;
    case sensor_msgs::msg::PointField::INT16: {
      int16_t raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<float>(raw);
      return true;
    }
    case sensor_msgs::msg::PointField::UINT16: {
      uint16_t raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<float>(raw);
      return true;
    }
    case sensor_msgs::msg::PointField::INT32: {
      int32_t raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<float>(raw);
      return true;
    }
    case sensor_msgs::msg::PointField::UINT32: {
      uint32_t raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<float>(raw);
      return true;
    }
    case sensor_msgs::msg::PointField::FLOAT32:
      std::memcpy(value, field_ptr, sizeof(float));
      return true;
    case sensor_msgs::msg::PointField::FLOAT64: {
      double raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<float>(raw);
      return true;
    }
    default:
      return false;
  }
}

inline bool readPointFieldAsDouble(
  const uint8_t * point_data,
  const sensor_msgs::msg::PointField & field,
  double * value)
{
  const uint8_t * field_ptr = point_data + field.offset;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::INT8: {
      int8_t raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<double>(raw);
      return true;
    }
    case sensor_msgs::msg::PointField::UINT8: {
      uint8_t raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<double>(raw);
      return true;
    }
    case sensor_msgs::msg::PointField::INT16: {
      int16_t raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<double>(raw);
      return true;
    }
    case sensor_msgs::msg::PointField::UINT16: {
      uint16_t raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<double>(raw);
      return true;
    }
    case sensor_msgs::msg::PointField::INT32: {
      int32_t raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<double>(raw);
      return true;
    }
    case sensor_msgs::msg::PointField::UINT32: {
      uint32_t raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<double>(raw);
      return true;
    }
    case sensor_msgs::msg::PointField::FLOAT32: {
      float raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<double>(raw);
      return true;
    }
    case sensor_msgs::msg::PointField::FLOAT64: {
      double raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = raw;
      return true;
    }
    default:
      return false;
  }
}

inline std::size_t pointFieldDatatypeSize(uint8_t datatype)
{
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8:
      return 1;
    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16:
      return 2;
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
    case sensor_msgs::msg::PointField::FLOAT32:
      return 4;
    case sensor_msgs::msg::PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

inline bool pointFieldFitsPointStep(
  const sensor_msgs::msg::PointField & field,
  uint32_t point_step)
{
  const std::size_t field_size = pointFieldDatatypeSize(field.datatype);
  if (field_size == 0) {
    return false;
  }
  return static_cast<std::size_t>(field.offset) + field_size <=
         static_cast<std::size_t>(point_step);
}

// livox_ros_driver2 publishes `timestamp` as FLOAT64 nanoseconds, but NOT always absolute epoch-scale
// (~1.8e18): the LIVE driver was found (2026-10-05) to emit RELATIVE within-scan nanoseconds instead
// (~1e8, i.e. up to ~0.1 s of a scan re-expressed in ns) -- a bag-converted/recorded topic can still
// carry the absolute ~1.8e18 form. Other drivers publish a FLOAT64 `timestamp` already in seconds
// (epoch ~1.8e9, or a small relative value close to scan_period). A single large-magnitude threshold
// (formerly 1e12) only catches the absolute-ns case and misses the smaller relative-ns one, silently
// disabling continuous-time deskew on live data (scan_time_status = scan_time_range_too_large).
//
// Fix: treat anything OUTSIDE the plausible "already seconds" bands as nanoseconds instead of using one
// large cutoff. A genuinely-seconds value is either small (<= a generous relative-offset bound, well
// above any real scan_period) or a real calendar-epoch value (~1e9-1e10, i.e. roughly year 2001-2286).
// Nanosecond values -- relative (~1e5-1e8) or absolute-epoch (~1e18) -- both fall outside both bands.
constexpr double kPlausibleRelativeSecondsBound = 1.0e3;
constexpr double kEpochSecondsLowerBound = 1.0e9;
constexpr double kEpochSecondsUpperBound = 1.0e10;

inline bool looksLikeAlreadySeconds(double sample_raw_value)
{
  const double magnitude = std::abs(sample_raw_value);
  return magnitude <= kPlausibleRelativeSecondsBound ||
    (magnitude >= kEpochSecondsLowerBound && magnitude <= kEpochSecondsUpperBound);
}

inline double pointTimeFieldScaleToSeconds(
  const sensor_msgs::msg::PointField & field,
  double sample_raw_value = std::numeric_limits<double>::quiet_NaN())
{
  if (field.name == "offset_time" || field.name == "t") {
    return 1.0e-9;
  }
  if (field.name == "timestamp") {
    if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 &&
      field.datatype != sensor_msgs::msg::PointField::FLOAT64)
    {
      return 1.0e-9;
    }
    if (field.datatype == sensor_msgs::msg::PointField::FLOAT64 &&
      std::isfinite(sample_raw_value) &&
      !looksLikeAlreadySeconds(sample_raw_value))
    {
      return 1.0e-9;
    }
  }
  return 1.0;
}

inline bool readPointTimeSeconds(
  const uint8_t * point_data,
  const sensor_msgs::msg::PointField & field,
  double * time_seconds)
{
  double raw_value = 0.0;
  if (!readPointFieldAsDouble(point_data, field, &raw_value)) {
    return false;
  }
  *time_seconds = raw_value * pointTimeFieldScaleToSeconds(field, raw_value);
  return true;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_POINT_FIELD_READ_HPP_
