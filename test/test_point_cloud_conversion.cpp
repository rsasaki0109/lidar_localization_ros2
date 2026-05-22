#include "lidar_localization/point_cloud_conversion.hpp"

#include <cassert>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

namespace ll = lidar_localization;

sensor_msgs::msg::PointField make_field(
  const std::string & name,
  uint32_t offset,
  uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

template<typename T>
void write_value(std::vector<uint8_t> & data, std::size_t offset, const T & value)
{
  std::memcpy(data.data() + offset, &value, sizeof(value));
}

sensor_msgs::msg::PointCloud2 make_sensor_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "livox_frame";
  cloud.height = 1;
  cloud.width = 2;
  cloud.is_dense = false;
  cloud.point_step = 16;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields = {
    make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
    make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
    make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
    make_field("intensity", 12, sensor_msgs::msg::PointField::UINT16)};
  cloud.data.resize(cloud.row_step);

  write_value<float>(cloud.data, 0, 1.0f);
  write_value<float>(cloud.data, 4, 2.0f);
  write_value<float>(cloud.data, 8, 3.0f);
  write_value<uint16_t>(cloud.data, 12, 42);

  write_value<float>(cloud.data, 16, -1.0f);
  write_value<float>(cloud.data, 20, -2.0f);
  write_value<float>(cloud.data, 24, 0.5f);
  write_value<uint16_t>(cloud.data, 28, 7);
  return cloud;
}

void test_field_lookup_and_numeric_conversion()
{
  const auto cloud = make_sensor_cloud();
  assert(ll::hasPointField(cloud.fields, "intensity"));
  assert(!ll::hasPointField(cloud.fields, "ring"));
  assert(ll::findPointField(cloud.fields, "x") != nullptr);
  assert(ll::findPointField(cloud.fields, "ring") == nullptr);

  float value = 0.0f;
  assert(ll::readPointFieldAsFloat(cloud.data.data(), *ll::findPointField(cloud.fields, "x"), &value));
  assert(std::abs(value - 1.0f) < 1.0e-6f);
  assert(ll::readPointFieldAsFloat(
    cloud.data.data(), *ll::findPointField(cloud.fields, "intensity"), &value));
  assert(std::abs(value - 42.0f) < 1.0e-6f);

  auto unsupported = make_field("unsupported", 0, 255);
  assert(!ll::readPointFieldAsFloat(cloud.data.data(), unsupported, &value));
}

void test_sensor_cloud_to_xyzi_uses_intensity_when_available()
{
  const auto cloud = make_sensor_cloud();
  pcl::PointCloud<pcl::PointXYZI> output;
  ll::convertSensorCloudToXyzi(cloud, output);

  assert(output.size() == 2);
  assert(std::abs(output[0].x - 1.0f) < 1.0e-6f);
  assert(std::abs(output[0].intensity - 42.0f) < 1.0e-6f);
  assert(std::abs(output[1].y + 2.0f) < 1.0e-6f);
  assert(std::abs(output[1].intensity - 7.0f) < 1.0e-6f);
  assert(output.is_dense == cloud.is_dense);
}

void test_sensor_cloud_to_xyzi_falls_back_to_zero_intensity()
{
  auto cloud = make_sensor_cloud();
  cloud.fields.pop_back();

  pcl::PointCloud<pcl::PointXYZI> output;
  ll::convertSensorCloudToXyzi(cloud, output);

  assert(output.size() == 2);
  assert(output[0].intensity == 0.0f);
  assert(output[1].intensity == 0.0f);
}

void test_sensor_cloud_to_xyzi_clears_output_when_xyz_missing()
{
  auto cloud = make_sensor_cloud();
  cloud.fields.erase(cloud.fields.begin() + 2);

  pcl::PointCloud<pcl::PointXYZI> output;
  output.push_back(pcl::PointXYZI{});
  ll::convertSensorCloudToXyzi(cloud, output);

  assert(output.empty());
  assert(output.width == 0);
  assert(output.height == 1);
  assert(!output.is_dense);
}

void test_pcl_cloud_to_xyzi_detects_or_synthesizes_intensity()
{
  pcl::PointCloud<pcl::PointXYZI> xyzi_cloud;
  xyzi_cloud.width = 1;
  xyzi_cloud.height = 1;
  xyzi_cloud.points.resize(1);
  xyzi_cloud[0].x = 1.0f;
  xyzi_cloud[0].intensity = 9.0f;
  pcl::PCLPointCloud2 xyzi_raw;
  pcl::toPCLPointCloud2(xyzi_cloud, xyzi_raw);

  pcl::PointCloud<pcl::PointXYZI> converted_xyzi;
  assert(ll::convertPclCloudToXyzi(xyzi_raw, converted_xyzi));
  assert(converted_xyzi.size() == 1);
  assert(std::abs(converted_xyzi[0].intensity - 9.0f) < 1.0e-6f);

  pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
  xyz_cloud.width = 1;
  xyz_cloud.height = 1;
  xyz_cloud.points.resize(1);
  xyz_cloud[0].x = 2.0f;
  xyz_cloud[0].y = 3.0f;
  xyz_cloud[0].z = 4.0f;
  pcl::PCLPointCloud2 xyz_raw;
  pcl::toPCLPointCloud2(xyz_cloud, xyz_raw);

  pcl::PointCloud<pcl::PointXYZI> converted_xyz;
  assert(!ll::convertPclCloudToXyzi(xyz_raw, converted_xyz));
  assert(converted_xyz.size() == 1);
  assert(std::abs(converted_xyz[0].x - 2.0f) < 1.0e-6f);
  assert(converted_xyz[0].intensity == 0.0f);
}

int main()
{
  test_field_lookup_and_numeric_conversion();
  test_sensor_cloud_to_xyzi_uses_intensity_when_available();
  test_sensor_cloud_to_xyzi_falls_back_to_zero_intensity();
  test_sensor_cloud_to_xyzi_clears_output_when_xyz_missing();
  test_pcl_cloud_to_xyzi_detects_or_synthesizes_intensity();
  return 0;
}
