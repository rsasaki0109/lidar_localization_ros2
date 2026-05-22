#ifndef LIDAR_LOCALIZATION_POINT_CLOUD_CONVERSION_HPP_
#define LIDAR_LOCALIZATION_POINT_CLOUD_CONVERSION_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace lidar_localization
{

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
