#include "lidar_localization/point_cloud_conversion.hpp"

#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
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

void test_point_time_field_detection_and_unit_conversion()
{
  auto cloud = make_sensor_cloud();
  cloud.point_step = 20;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.push_back(make_field("offset_time", 16, sensor_msgs::msg::PointField::UINT32));
  cloud.data.assign(cloud.row_step, 0);
  write_value<uint32_t>(cloud.data, 16, 25000000U);
  write_value<uint32_t>(cloud.data, 36, 50000000U);

  const auto * time_field = ll::findPointTimeField(cloud.fields);
  assert(time_field != nullptr);
  assert(time_field->name == "offset_time");
  assert(std::abs(ll::pointTimeFieldScaleToSeconds(*time_field) - 1.0e-9) < 1.0e-15);

  double time_seconds = 0.0;
  assert(ll::readPointTimeSeconds(cloud.data.data(), *time_field, &time_seconds));
  assert(std::abs(time_seconds - 0.025) < 1.0e-12);
  assert(ll::readPointTimeSeconds(
    cloud.data.data() + cloud.point_step, *time_field, &time_seconds));
  assert(std::abs(time_seconds - 0.050) < 1.0e-12);
}

void test_point_time_field_accepts_float_seconds_and_integer_timestamp_nanoseconds()
{
  auto seconds_cloud = make_sensor_cloud();
  seconds_cloud.point_step = 20;
  seconds_cloud.row_step = seconds_cloud.point_step * seconds_cloud.width;
  seconds_cloud.fields.push_back(make_field("time", 16, sensor_msgs::msg::PointField::FLOAT32));
  seconds_cloud.data.assign(seconds_cloud.row_step, 0);
  write_value<float>(seconds_cloud.data, 16, 0.125f);

  double time_seconds = 0.0;
  const auto * seconds_field = ll::findPointTimeField(seconds_cloud.fields);
  assert(seconds_field != nullptr);
  assert(seconds_field->name == "time");
  assert(ll::readPointTimeSeconds(seconds_cloud.data.data(), *seconds_field, &time_seconds));
  assert(std::abs(time_seconds - 0.125) < 1.0e-6);

  auto timestamp_cloud = make_sensor_cloud();
  timestamp_cloud.point_step = 24;
  timestamp_cloud.row_step = timestamp_cloud.point_step * timestamp_cloud.width;
  timestamp_cloud.fields.push_back(
    make_field("timestamp", 16, sensor_msgs::msg::PointField::UINT32));
  timestamp_cloud.data.assign(timestamp_cloud.row_step, 0);
  write_value<uint32_t>(timestamp_cloud.data, 16, 75000000U);

  const auto * timestamp_field = ll::findPointTimeField(timestamp_cloud.fields);
  assert(timestamp_field != nullptr);
  assert(timestamp_field->name == "timestamp");
  assert(std::abs(ll::pointTimeFieldScaleToSeconds(*timestamp_field) - 1.0e-9) < 1.0e-15);
  assert(ll::readPointTimeSeconds(
    timestamp_cloud.data.data(), *timestamp_field, &time_seconds));
  assert(std::abs(time_seconds - 0.075) < 1.0e-12);
}

sensor_msgs::msg::PointCloud2 make_float64_timestamp_cloud(const std::vector<double> & stamps)
{
  // livox_ros_driver2 layout: x y z intensity (f32), tag line (u8), timestamp (f64)
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "livox_frame";
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(stamps.size());
  cloud.is_dense = false;
  cloud.point_step = 26;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields = {
    make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
    make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
    make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
    make_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32),
    make_field("tag", 16, sensor_msgs::msg::PointField::UINT8),
    make_field("line", 17, sensor_msgs::msg::PointField::UINT8),
    make_field("timestamp", 18, sensor_msgs::msg::PointField::FLOAT64)};
  cloud.data.assign(cloud.row_step, 0);
  for (std::size_t i = 0; i < stamps.size(); ++i) {
    write_value<double>(cloud.data, i * cloud.point_step + 18, stamps[i]);
  }
  return cloud;
}

void test_float64_timestamp_is_nanoseconds_for_livox_and_seconds_otherwise()
{
  // Real Mid-360 scan: absolute nanoseconds, span 99.697408 ms (header.stamp == first point).
  const double t_first = 1790817932991754752.0;
  const double t_last = 1790817933091452160.0;
  const auto livox = make_float64_timestamp_cloud({t_first, t_last, 1790817933040000000.0});

  const auto * field = ll::findPointTimeField(livox.fields);
  assert(field != nullptr);
  assert(field->name == "timestamp");
  assert(field->datatype == sensor_msgs::msg::PointField::FLOAT64);
  // Without a sample the legacy answer (seconds) is kept; a nanosecond-sized sample flips it.
  assert(std::abs(ll::pointTimeFieldScaleToSeconds(*field) - 1.0) < 1.0e-15);
  assert(std::abs(ll::pointTimeFieldScaleToSeconds(*field, t_first) - 1.0e-9) < 1.0e-15);

  const ll::PointTimeRange range = ll::computePointTimeRangeSeconds(livox);
  assert(range.valid);
  assert(std::abs(range.durationSec() - 0.099697408) < 1.0e-6);

  const ll::PointRelativeTimes times = ll::extractPointRelativeTimesSeconds(livox);
  assert(times.valid);
  assert(times.hasCompleteTimes());
  assert(std::abs(times.duration_sec - 0.099697408) < 1.0e-6);
  assert(std::abs(times.relative_times_sec[0]) < 1.0e-6);
  assert(std::abs(times.relative_times_sec[1] - 0.099697408) < 1.0e-6);
  assert(std::abs(times.relative_times_sec[2] - 0.048245248) < 1.0e-6);

  // Drivers that publish float64 epoch SECONDS keep their meaning.
  const auto epoch_seconds = make_float64_timestamp_cloud({1790817932.50, 1790817932.60});
  const ll::PointRelativeTimes seconds_times = ll::extractPointRelativeTimesSeconds(epoch_seconds);
  assert(seconds_times.valid);
  assert(std::abs(seconds_times.duration_sec - 0.10) < 1.0e-6);

  // So do small relative float64 seconds.
  const auto relative_seconds = make_float64_timestamp_cloud({0.0, 0.05, 0.1});
  const ll::PointRelativeTimes relative_times = ll::extractPointRelativeTimesSeconds(relative_seconds);
  assert(relative_times.valid);
  assert(std::abs(relative_times.duration_sec - 0.1) < 1.0e-12);
}

void test_float64_timestamp_handles_live_relative_nanoseconds()
{
  // Real live Mid-360 driver (not a recorded/converted bag) observed 2026-10-05: FLOAT64 `timestamp`
  // holding RELATIVE within-scan nanoseconds (~1e8), not the absolute epoch-nanoseconds form
  // (~1.8e18) the field once assumed. Magnitude alone (the old 1e12 cutoff) missed this smaller case
  // and silently left continuous-time deskew disabled on live data while bag replay stayed fine.
  const auto live_relative_ns = make_float64_timestamp_cloud(
    {0.0, 50172416.0, 100344832.0});
  const auto * field = ll::findPointTimeField(live_relative_ns.fields);
  assert(field != nullptr);
  assert(std::abs(ll::pointTimeFieldScaleToSeconds(*field, 100344832.0) - 1.0e-9) < 1.0e-15);

  const ll::PointRelativeTimes times = ll::extractPointRelativeTimesSeconds(live_relative_ns);
  assert(times.valid);
  assert(times.hasCompleteTimes());
  assert(std::abs(times.duration_sec - 0.100344832) < 1.0e-9);
}

void test_point_time_range_uses_min_max_even_when_points_are_unsorted()
{
  auto cloud = make_sensor_cloud();
  cloud.point_step = 20;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.push_back(make_field("offset_time", 16, sensor_msgs::msg::PointField::UINT32));
  cloud.data.assign(cloud.row_step, 0);
  write_value<uint32_t>(cloud.data, 16, 70000000U);
  write_value<uint32_t>(cloud.data, 36, 20000000U);

  const ll::PointTimeRange range = ll::computePointTimeRangeSeconds(cloud);

  assert(range.has_time_field);
  assert(range.valid);
  assert(range.field_name == "offset_time");
  assert(range.valid_point_count == 2);
  assert(range.invalid_point_count == 0);
  assert(std::abs(range.min_time_sec - 0.020) < 1.0e-12);
  assert(std::abs(range.max_time_sec - 0.070) < 1.0e-12);
  assert(std::abs(range.durationSec() - 0.050) < 1.0e-12);
}

void test_point_time_range_reports_missing_or_invalid_fields_without_crashing()
{
  const auto no_time_cloud = make_sensor_cloud();
  const ll::PointTimeRange missing = ll::computePointTimeRangeSeconds(no_time_cloud);
  assert(!missing.has_time_field);
  assert(!missing.valid);
  assert(std::isnan(missing.durationSec()));

  auto invalid_cloud = make_sensor_cloud();
  invalid_cloud.fields.push_back(
    make_field("offset_time", invalid_cloud.point_step, sensor_msgs::msg::PointField::UINT32));
  const ll::PointTimeRange invalid = ll::computePointTimeRangeSeconds(invalid_cloud);
  assert(invalid.has_time_field);
  assert(!invalid.valid);
  assert(invalid.invalid_point_count == 2);
}

void test_extract_point_relative_times_normalizes_to_scan_start()
{
  auto cloud = make_sensor_cloud();
  cloud.point_step = 20;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.push_back(make_field("offset_time", 16, sensor_msgs::msg::PointField::UINT32));
  cloud.data.assign(cloud.row_step, 0);
  write_value<uint32_t>(cloud.data, 16, 70000000U);
  write_value<uint32_t>(cloud.data, 36, 20000000U);

  const ll::PointRelativeTimes times = ll::extractPointRelativeTimesSeconds(cloud);

  assert(times.has_time_field);
  assert(times.valid);
  assert(times.field_name == "offset_time");
  assert(times.hasCompleteTimes());
  assert(times.valid_point_count == 2);
  assert(times.invalid_point_count == 0);
  assert(times.relative_times_sec.size() == 2);
  assert(std::abs(times.reference_time_sec - 0.020) < 1.0e-12);
  assert(std::abs(times.duration_sec - 0.050) < 1.0e-12);
  assert(std::abs(times.relative_times_sec[0] - 0.050) < 1.0e-12);
  assert(std::abs(times.relative_times_sec[1] - 0.000) < 1.0e-12);
}

void test_extract_point_relative_times_keeps_invalid_points_as_nan()
{
  auto cloud = make_sensor_cloud();
  cloud.point_step = 20;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.push_back(make_field("time", 16, sensor_msgs::msg::PointField::FLOAT32));
  cloud.data.assign(cloud.row_step, 0);
  write_value<float>(cloud.data, 16, 0.125f);
  write_value<float>(
    cloud.data, 36, std::numeric_limits<float>::quiet_NaN());

  const ll::PointRelativeTimes times = ll::extractPointRelativeTimesSeconds(cloud);

  assert(times.has_time_field);
  assert(times.valid);
  assert(!times.hasCompleteTimes());
  assert(times.valid_point_count == 1);
  assert(times.invalid_point_count == 1);
  assert(times.relative_times_sec.size() == 2);
  assert(std::abs(times.reference_time_sec - 0.125) < 1.0e-6);
  assert(std::abs(times.relative_times_sec[0]) < 1.0e-12);
  assert(std::isnan(times.relative_times_sec[1]));
}

void test_extract_point_relative_times_reports_missing_or_invalid_field()
{
  const auto no_time_cloud = make_sensor_cloud();
  const ll::PointRelativeTimes missing = ll::extractPointRelativeTimesSeconds(no_time_cloud);
  assert(!missing.has_time_field);
  assert(!missing.valid);
  assert(missing.relative_times_sec.empty());

  auto invalid_cloud = make_sensor_cloud();
  invalid_cloud.fields.push_back(
    make_field("offset_time", invalid_cloud.point_step, sensor_msgs::msg::PointField::UINT32));
  const ll::PointRelativeTimes invalid = ll::extractPointRelativeTimesSeconds(invalid_cloud);
  assert(invalid.has_time_field);
  assert(!invalid.valid);
  assert(invalid.invalid_point_count == 2);
  assert(invalid.relative_times_sec.size() == 2);
  assert(std::isnan(invalid.relative_times_sec[0]));
  assert(std::isnan(invalid.relative_times_sec[1]));
}

void test_timed_xyzi_conversion_preserves_relative_times()
{
  auto cloud = make_sensor_cloud();
  cloud.point_step = 20;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.push_back(make_field("offset_time", 16, sensor_msgs::msg::PointField::UINT32));
  cloud.data.assign(cloud.row_step, 0);
  write_value<float>(cloud.data, 0, 1.0f);
  write_value<float>(cloud.data, 4, 2.0f);
  write_value<float>(cloud.data, 8, 3.0f);
  write_value<uint16_t>(cloud.data, 12, 42);
  write_value<uint32_t>(cloud.data, 16, 70000000U);
  write_value<float>(cloud.data, 20, -1.0f);
  write_value<float>(cloud.data, 24, -2.0f);
  write_value<float>(cloud.data, 28, 0.5f);
  write_value<uint16_t>(cloud.data, 32, 7);
  write_value<uint32_t>(cloud.data, 36, 20000000U);

  const ll::PointRelativeTimes times = ll::extractPointRelativeTimesSeconds(cloud);
  const ll::TimedXyziCloud timed_cloud = ll::convertSensorCloudToTimedXyzi(cloud, times);

  assert(timed_cloud.cloud.size() == 2);
  assert(timed_cloud.timeVectorAlignedWithCloud());
  assert(timed_cloud.hasCompleteTimesForCloud());
  assert(timed_cloud.valid_time_count == 2);
  assert(timed_cloud.invalid_time_count == 0);
  assert(std::abs(timed_cloud.cloud[0].intensity - 42.0f) < 1.0e-6f);
  assert(std::abs(timed_cloud.relative_times_sec[0] - 0.050) < 1.0e-12);
  assert(std::abs(timed_cloud.relative_times_sec[1] - 0.000) < 1.0e-12);
}

void test_timed_xyzi_conversion_keeps_invalid_times_as_nan()
{
  auto cloud = make_sensor_cloud();
  cloud.point_step = 20;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.push_back(make_field("time", 16, sensor_msgs::msg::PointField::FLOAT32));
  cloud.data.assign(cloud.row_step, 0);
  write_value<float>(cloud.data, 0, 1.0f);
  write_value<float>(cloud.data, 4, 2.0f);
  write_value<float>(cloud.data, 8, 3.0f);
  write_value<float>(cloud.data, 16, 0.125f);
  write_value<float>(cloud.data, 20, -1.0f);
  write_value<float>(cloud.data, 24, -2.0f);
  write_value<float>(cloud.data, 28, 0.5f);
  write_value<float>(
    cloud.data, 36, std::numeric_limits<float>::quiet_NaN());

  const ll::PointRelativeTimes times = ll::extractPointRelativeTimesSeconds(cloud);
  const ll::TimedXyziCloud timed_cloud = ll::convertSensorCloudToTimedXyzi(cloud, times);

  assert(timed_cloud.cloud.size() == 2);
  assert(timed_cloud.timeVectorAlignedWithCloud());
  assert(!timed_cloud.hasCompleteTimesForCloud());
  assert(timed_cloud.valid_time_count == 1);
  assert(timed_cloud.invalid_time_count == 1);
  assert(std::abs(timed_cloud.relative_times_sec[0]) < 1.0e-12);
  assert(std::isnan(timed_cloud.relative_times_sec[1]));
  assert(std::isnan(ll::relativeTimeOrNaN(timed_cloud.relative_times_sec, 99)));
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
  test_point_time_field_detection_and_unit_conversion();
  test_point_time_field_accepts_float_seconds_and_integer_timestamp_nanoseconds();
  test_float64_timestamp_is_nanoseconds_for_livox_and_seconds_otherwise();
  test_float64_timestamp_handles_live_relative_nanoseconds();
  test_point_time_range_uses_min_max_even_when_points_are_unsorted();
  test_point_time_range_reports_missing_or_invalid_fields_without_crashing();
  test_extract_point_relative_times_normalizes_to_scan_start();
  test_extract_point_relative_times_keeps_invalid_points_as_nan();
  test_extract_point_relative_times_reports_missing_or_invalid_field();
  test_timed_xyzi_conversion_preserves_relative_times();
  test_timed_xyzi_conversion_keeps_invalid_times_as_nan();
  test_sensor_cloud_to_xyzi_uses_intensity_when_available();
  test_sensor_cloud_to_xyzi_falls_back_to_zero_intensity();
  test_sensor_cloud_to_xyzi_clears_output_when_xyz_missing();
  test_pcl_cloud_to_xyzi_detects_or_synthesizes_intensity();
  return 0;
}
