#include "go2_recovery.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/time.hpp>
#include <deque>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <array>
#include <set>
#include "lidar_localization/bbs_branch_and_bound.hpp"
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include "lidar_localization/so3_utils.hpp"
#include "lidar_localization/point_field_read.hpp"

int64_t go2RecoveryNow()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
void go2ReceiveImu(const sensor_msgs::msg::Imu & msg, Go2Recovery & state, uint64_t generation)
{
  if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000U) {return;}
  const auto received = go2RecoveryNow();
  const int64_t stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
  const Eigen::Vector3d w(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
  std::lock_guard<std::mutex> lock(state.imu_mutex);
  const bool accepted = generation == state.input_generation &&
    msg.header.frame_id == "livox_frame" && w.allFinite() &&
    (state.imu_history.empty() || stamp > state.imu_history.back().stamp);
  if (accepted) {
    state.imu_history.push_back({stamp, received, w});
    while (state.imu_history.size() > 4096 ||
      (!state.imu_history.empty() && state.imu_history.front().stamp < stamp - 2000000000LL))
    {
      state.imu_history.pop_front();
    }
  }
}
std::vector<Go2Imu> go2ImuSnapshot(int64_t cutoff, Go2Recovery & state, uint64_t generation)
{
  std::lock_guard<std::mutex> lock(state.imu_mutex);
  std::vector<Go2Imu> result;
  for (const auto & sample:state.imu_history) {
    if (generation == state.input_generation && sample.receipt <= cutoff) {
      result.push_back(sample);
    }
  }
  return result;
}
namespace
{
std::array<const sensor_msgs::msg::PointField *, 4> recoveryFields(
  const sensor_msgs::msg::PointCloud2 & msg)
{
  using namespace lidar_localization;
  if (msg.is_bigendian || msg.header.frame_id != "livox_frame" || msg.width == 0 ||
    msg.height == 0 || msg.point_step == 0 ||
    uint64_t(msg.row_step) < uint64_t(msg.width) * msg.point_step ||
    msg.data.size() < uint64_t(msg.height) * msg.row_step)
  {
    throw std::runtime_error("Go2 recovery requires a valid little-endian livox_frame cloud");
  }
  std::array<const sensor_msgs::msg::PointField *, 4> fields;
  size_t n = 0;
  for (const char * name:{"x", "y", "z", "t"}) {
    auto f = findPointField(msg.fields, name);
    const bool time = n == 3;
    const size_t size = time ? 8 : 4;
    if (!f || f->count != 1 ||
      f->datatype !=
      (time ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32) ||
      size > msg.point_step || f->offset > msg.point_step - size)
    {
      throw std::runtime_error("Go2 recovery requires float32 xyz and float64 absolute-seconds t");
    }
    fields[n++] = f;
  }
  return fields;
}
}  // namespace

std::vector<Eigen::Matrix4f> go2BbsSeeds(
  const std::string & path,
  const sensor_msgs::msg::PointCloud2 & msg, const Eigen::Matrix4d & last,
  const Eigen::Matrix4d & seed)
{
  namespace bbs = lidar_localization::bbs;
  recoveryFields(msg);
  if (!last.allFinite() || !seed.allFinite()) {
    throw std::runtime_error("non-finite recovery reference");
  }
  pcl::PointCloud<pcl::PointXYZ> raw_map, raw_cloud;
  const bool ply = path.size() >= 4 && path.substr(path.size() - 4) == ".ply";
  const int loaded = ply ? pcl::io::loadPLYFile(path, raw_map) : pcl::io::loadPCDFile(path,
    raw_map);
  if (loaded < 0) {throw std::runtime_error("BBS raw map load");}
  pcl::fromROSMsg(msg, raw_cloud);
  std::vector<std::array<float, 3>> map;
  map.reserve(raw_map.size());
  for (const auto & p:raw_map) {
    map.push_back({p.x, p.y, p.z});
  }
  std::vector<std::array<double, 3>> cloud;
  cloud.reserve(raw_cloud.size());
  for (const auto & p:raw_cloud) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      cloud.push_back({p.x, p.y, p.z});
    }
  }
  std::vector<std::array<float, 2>> band;
  float minx = std::numeric_limits<float>::infinity(), miny = minx;
  for (const auto & p:map) {
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]) || p[2] < last(2,
      3) + .5 || p[2] > last(2, 3) + 5.) {continue;}
    band.push_back({p[0], p[1]});
    minx = std::min(minx, p[0]);
    miny = std::min(miny, p[1]);
  }
  if (band.empty()) {throw std::runtime_error("empty map height band");}
  // Preserve the existing helper's float32 map raster arithmetic.
  const float ox = (std::floor(minx / .2f) - 1.f) * .2f, oy = (std::floor(miny / .2f) - 1.f) * .2f;
  std::vector<std::array<int, 2>> cells;
  int width = 0, height = 0;
  for (const auto & p:band) {
    const float cell_x = std::floor((p[0] - ox) / .2f);
    const float cell_y = std::floor((p[1] - oy) / .2f);
    // Bound allocation and conversions before constructing the occupancy pyramid.
    if (!std::isfinite(cell_x) || !std::isfinite(cell_y) ||
      cell_x < 0 || cell_y < 0 || cell_x > 16382 || cell_y > 16382)
    {
      throw std::runtime_error("recovery map extent exceeds supported grid");
    }
    const int x = static_cast<int>(cell_x), y = static_cast<int>(cell_y);
    if (x < 0 || y < 0) {throw std::runtime_error("negative map cell");}
    cells.push_back({x, y});
    width = std::max(width, x + 2);
    height = std::max(height, y + 2);
  }
  if (static_cast<uint64_t>(height) * width > 16 * 1024 * 1024) {
    throw std::runtime_error("recovery map exceeds 16M grid cells");
  }
  bbs::Grid grid(height, width);
  for (const auto & c:cells) {
    grid.set(c[1], c[0], 1);
  }
  grid = bbs::dilate_one_cell(grid);
  std::vector<std::array<double, 2>> points;
  std::set<std::array<int64_t, 2>> visited;
  for (const auto & p:cloud) {
    const Eigen::Vector3d rotated = seed.block<3, 3>(0, 0) * Eigen::Vector3d(p[0], p[1], p[2]);
    const double x = rotated.x(), y = rotated.y(), z = rotated.z();
    if (!rotated.allFinite() || z < .5 || z > 5. || std::hypot(x, y) < 1.) {continue;}
    if (std::abs(x / .2) > 1000000 || std::abs(y / .2) > 1000000) {
      throw std::runtime_error("recovery scan extent exceeds supported grid");
    }
    const std::array<int64_t, 2> cell{static_cast<int64_t>(std::floor(x / .2)),
      static_cast<int64_t>(std::floor(y / .2))};
    if (visited.insert(cell).second) {points.push_back({x, y});}
  }
  if (points.size() > 512) {
    std::vector<std::array<double, 2>> sampled;
    sampled.reserve(512);
    const double step = static_cast<double>(points.size() - 1) / 511.;
    for (size_t i = 0; i < 512; ++i) {
      sampled.push_back(points[i == 511 ? points.size() - 1 : static_cast<size_t>(i * step)]);
    }
    points.swap(sampled);
  }
  const auto candidates = bbs::branch_and_bound_candidates(grid, points, .2, 2. * std::acos(-1.), 4,
    16, 10);
  std::vector<Eigen::Matrix4f> result;
  for (const auto & c:candidates) {
    Eigen::Matrix4f pose = seed.cast<float>();
    const double x = double(ox) + (c.tx_cell + .5) * .2, y = double(oy) + (c.ty_cell + .5) * .2;
    pose(0, 3) = x;
    pose(1, 3) = y;
    pose(2, 3) = last(2, 3);
    result.push_back(pose);
  }
  if (result.size() != 16) {throw std::runtime_error("recovery BBS requires 16 candidates");}
  return result;
}
using Go2Cloud = pcl::PointCloud<pcl::PointXYZ>;
static Go2Cloud::Ptr go2Deskew(
  const sensor_msgs::msg::PointCloud2 & msg,
  const std::vector<Go2Imu> & input)
{
  using namespace lidar_localization;
  const auto fields = recoveryFields(msg);
  const int64_t origin = int64_t(msg.header.stamp.sec) * 1000000000LL + msg.header.stamp.nanosec;
  const double header = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
  struct Imu
  {
    double t;
    Eigen::Vector3d w;
  };
  std::vector<Imu> imu;
  for (const auto & r:input) {
    double t = double(r.stamp - origin) * 1e-9;
    if (!r.w.allFinite() || (!imu.empty() && t <= imu.back().t)) {
      throw std::runtime_error("invalid IMU");
    }
    imu.push_back({t, r.w});
  }
  std::vector<Eigen::Vector3d> points;
  std::vector<double> queries{0.};
  for (size_t row = 0; row < msg.height; ++row) {
    for (size_t col = 0; col < msg.width; ++col) {
      const auto * data = msg.data.data() + row * msg.row_step + col * msg.point_step;
      std::array<double, 4> v;
      for (size_t k = 0; k < 4; ++k) {
        if (!readPointFieldAsDouble(data, *fields[k], &v[k])) {
          throw std::runtime_error("field read");
        }
      }
      Eigen::Vector3d p(v[0], v[1], v[2]);
      double range = p.norm();
      if (!p.allFinite() || range < .5 || range > 60.) {continue;}
      if (!std::isfinite(v[3])) {throw std::runtime_error("point time");} points.push_back(p);
      queries.push_back(v[3] - header);
    }
  }
  if (points.empty() || imu.empty()) {throw std::runtime_error("empty deskew input");}
  std::vector<size_t> indices;
  for (double t:queries) {
    auto it = std::upper_bound(imu.begin(), imu.end(), t, [](double value, const Imu & sample) {
          return value < sample.t;
        });
    if (it == imu.begin()) {throw std::runtime_error("history starts too late");}
    const size_t i = std::distance(imu.begin(), it) - 1;
    const double age = t - imu[i].t;
    if (age < 0 || age > .02) {throw std::runtime_error("IMU coverage");}
    indices.push_back(i);
  }
  const size_t first = *std::min_element(indices.begin(), indices.end()),
    last = *std::max_element(indices.begin(), indices.end());
  std::vector<Eigen::Matrix3d> knots{Eigen::Matrix3d::Identity()};
  for (size_t i = first; i < last; ++i) {
    knots.push_back((knots.back() * so3::Exp(imu[i].w * (imu[i + 1].t - imu[i].t))).eval());
  }
  const auto rotation = [&](size_t j)->Eigen::Matrix3d {
      const size_t i = indices[j];
      return knots[i - first] * so3::Exp(imu[i].w * (queries[j] - imu[i].t));
    };
  const Eigen::Matrix3d reference = rotation(0);
  Go2Cloud::Ptr raw(new Go2Cloud), filtered(new Go2Cloud);
  for (size_t j = 0; j < points.size(); ++j) {
    const Eigen::Vector3f p = (reference.transpose() * rotation(j + 1) * points[j]).cast<float>();
    raw->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
  }
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(raw);
  voxel.setLeafSize(.2f, .2f, .2f);
  voxel.filter(*filtered);
  return filtered;
}
Eigen::Matrix4f go2Delta(
  const sensor_msgs::msg::PointCloud2 & previous,
  const sensor_msgs::msg::PointCloud2 & next, const std::vector<Go2Imu> & previous_imu,
  const std::vector<Go2Imu> & next_imu)
{
  auto target = go2Deskew(previous, previous_imu), source = go2Deskew(next, next_imu);
  if (source->size() < 20 || target->size() < 20) {
    throw std::runtime_error("insufficient points for recovery GICP covariance");
  }
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> g;
  g.setInputSource(source);
  g.setInputTarget(target);
  g.setMaxCorrespondenceDistance(2.);
  g.setCorrespondenceRandomness(20);
  g.setMaximumIterations(30);
  g.setTransformationEpsilon(1e-6);
  Go2Cloud aligned;
  g.align(aligned, Eigen::Matrix4f::Identity());
  const Eigen::Matrix4f result = g.getFinalTransformation();
  if (!g.hasConverged() || !result.allFinite()) {throw std::runtime_error("GICP failed");}
  return result;
}
