#ifndef LIDAR_LOCALIZATION_POINT_FIELD_READ_HPP_
#define LIDAR_LOCALIZATION_POINT_FIELD_READ_HPP_

// Low-level sensor_msgs::msg::PointCloud2 field lookup and typed reading
// primitives shared by the point cloud conversion helpers.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
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

inline bool isFloatingPointField(const sensor_msgs::msg::PointField & field)
{
  return field.datatype == sensor_msgs::msg::PointField::FLOAT32 ||
         field.datatype == sensor_msgs::msg::PointField::FLOAT64;
}

inline double pointTimeFieldScaleToSeconds(const sensor_msgs::msg::PointField & field)
{
  // Integer "t" / "offset_time" / "timestamp" fields are nanoseconds (Ouster,
  // Livox).  Floating-point time fields are seconds; converted Livox clouds
  // commonly carry an absolute float64 "t" in seconds, and scaling that by
  // 1e-9 collapses the scan duration to ~0 and silently disables deskew.
  if ((field.name == "offset_time" || field.name == "t" || field.name == "timestamp") &&
    !isFloatingPointField(field))
  {
    return 1.0e-9;
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
  *time_seconds = raw_value * pointTimeFieldScaleToSeconds(field);
  return true;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_POINT_FIELD_READ_HPP_
