#ifndef LIDAR_LOCALIZATION_G2_NDT_CANDIDATE_SCORE_HPP_
#define LIDAR_LOCALIZATION_G2_NDT_CANDIDATE_SCORE_HPP_

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

namespace lidar_localization
{
namespace g2_ndt
{

using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using CloudPtr = Cloud::Ptr;

struct G2NdtScoreParams
{
  double ndt_resolution{1.0};
  double ndt_step_size{0.1};
  double transform_epsilon{0.01};
  int max_iterations{30};
  int num_threads{1};
  // pclomp::NeighborSearchMethod (KDTREE=0, DIRECT26=1, DIRECT7=2, DIRECT1=3).
  // Defaults to DIRECT7 to preserve the pre-A/B runtime behavior.
  int search_method{2};
  double scan_voxel_leaf_size{1.0};
  double target_voxel_leaf_size{1.0};
  double local_map_radius{150.0};
  std::size_t min_target_points{100};
};

struct G2NdtScoreResult
{
  double fitness{std::numeric_limits<double>::infinity()};
  bool converged{false};
  std::size_t target_point_count{0};
  std::size_t source_point_count{0};
  // Pose from ndt.getFinalTransformation() when converged; otherwise the seed
  // pose passed to score() (x, y, z, yaw).
  double refined_x{std::numeric_limits<double>::quiet_NaN()};
  double refined_y{std::numeric_limits<double>::quiet_NaN()};
  double refined_z{std::numeric_limits<double>::quiet_NaN()};
  double refined_yaw{std::numeric_limits<double>::quiet_NaN()};
};

inline bool has_point_field(
  const std::vector<pcl::PCLPointField> & fields,
  const std::string & name)
{
  return std::any_of(fields.begin(), fields.end(), [&](const auto & field) {
    return field.name == name;
  });
}

inline CloudPtr load_cloud_xyzi(const std::string & path)
{
  pcl::PCLPointCloud2 raw;
  const std::string lower = [&]() {
    std::string copy = path;
    std::transform(copy.begin(), copy.end(), copy.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return copy;
  }();
  int result = -1;
  if (lower.size() >= 4 && lower.substr(lower.size() - 4) == ".pcd") {
    result = pcl::io::loadPCDFile(path, raw);
  } else if (lower.size() >= 4 && lower.substr(lower.size() - 4) == ".ply") {
    result = pcl::io::loadPLYFile(path, raw);
  } else {
    throw std::runtime_error("unsupported point cloud suffix: " + path);
  }
  if (result != 0) {
    throw std::runtime_error("failed to load point cloud: " + path);
  }

  CloudPtr cloud(new Cloud());
  if (has_point_field(raw.fields, "intensity")) {
    pcl::fromPCLPointCloud2(raw, *cloud);
  } else {
    pcl::PointCloud<pcl::PointXYZ> xyz;
    pcl::fromPCLPointCloud2(raw, xyz);
    cloud->reserve(xyz.size());
    for (const auto & point : xyz.points) {
      pcl::PointXYZI xyzi;
      xyzi.x = point.x;
      xyzi.y = point.y;
      xyzi.z = point.z;
      xyzi.intensity = 0.0F;
      cloud->push_back(xyzi);
    }
  }
  return cloud;
}

inline CloudPtr voxel_downsample(const CloudPtr & input, double leaf_size)
{
  if (leaf_size <= 0.0 || input->empty()) {
    return input;
  }
  CloudPtr filtered(new Cloud());
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setLeafSize(
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size));
  voxel.setInputCloud(input);
  voxel.filter(*filtered);
  return filtered;
}

inline CloudPtr crop_map_xy(const CloudPtr & map, double x, double y, double radius)
{
  if (radius <= 0.0) {
    return map;
  }
  const double r2 = radius * radius;
  CloudPtr cropped(new Cloud());
  cropped->reserve(map->size() / 10);
  for (const auto & point : map->points) {
    const double dx = static_cast<double>(point.x) - x;
    const double dy = static_cast<double>(point.y) - y;
    if (dx * dx + dy * dy <= r2) {
      cropped->push_back(point);
    }
  }
  return cropped;
}

inline Eigen::Matrix4f pose_matrix(double x, double y, double z, double yaw)
{
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  const float c = static_cast<float>(std::cos(yaw));
  const float s = static_cast<float>(std::sin(yaw));
  matrix(0, 0) = c;
  matrix(0, 1) = -s;
  matrix(1, 0) = s;
  matrix(1, 1) = c;
  matrix(0, 3) = static_cast<float>(x);
  matrix(1, 3) = static_cast<float>(y);
  matrix(2, 3) = static_cast<float>(z);
  return matrix;
}

inline CloudPtr cloud_from_xyz(
  const double * xyz,
  std::size_t point_count,
  double scan_voxel_leaf_size)
{
  CloudPtr cloud(new Cloud());
  cloud->reserve(point_count);
  for (std::size_t i = 0; i < point_count; ++i) {
    pcl::PointXYZI point;
    point.x = static_cast<float>(xyz[3 * i]);
    point.y = static_cast<float>(xyz[3 * i + 1]);
    point.z = static_cast<float>(xyz[3 * i + 2]);
    point.intensity = 0.0F;
    cloud->push_back(point);
  }
  return voxel_downsample(cloud, scan_voxel_leaf_size);
}

struct G2NdtPose
{
  double x;
  double y;
  double z;
  double yaw;
};

// A 2D global candidate has no height of its own. The ground under it plus the
// sensor height is a seed NDT registers from on a map with hills or ramps, where
// one fixed seed height is metres off away from where it was measured.
constexpr double kGroundCellM = 1.0;

// The lowest map point in each kGroundCellM column.
class GroundHeightIndex
{
public:
  explicit GroundHeightIndex(const Cloud & map)
  {
    for (const auto & point : map.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const auto [lowest, inserted] = lowest_.try_emplace(key(point.x, point.y), point.z);
      if (!inserted && point.z < lowest->second) {
        lowest->second = point.z;
      }
    }
  }

  // Median of the lowest points of the 3 x 3 columns around (x, y), so a stray
  // return below the ground does not count; NaN where the map has no points.
  double ground_z(double x, double y) const
  {
    const std::int64_t ix = index(x);
    const std::int64_t iy = index(y);
    std::vector<float> lows;
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        const auto found = lowest_.find(key(ix + dx, iy + dy));
        if (found != lowest_.end()) {
          lows.push_back(found->second);
        }
      }
    }
    if (lows.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const auto middle = lows.begin() + static_cast<std::ptrdiff_t>(lows.size() / 2);
    std::nth_element(lows.begin(), middle, lows.end());
    return static_cast<double>(*middle);
  }

private:
  static std::int64_t index(double value)
  {
    return static_cast<std::int64_t>(std::floor(value / kGroundCellM));
  }

  static std::uint64_t key(std::int64_t ix, std::int64_t iy)
  {
    return (static_cast<std::uint64_t>(ix) << 32) ^ (static_cast<std::uint64_t>(iy) & 0xffffffffULL);
  }

  static std::uint64_t key(double x, double y)
  {
    return key(index(x), index(y));
  }

  std::unordered_map<std::uint64_t, float> lowest_;
};

// Margin (m) between the transformed scan and the edge of a shared map crop: NDT
// looks up neighbouring cells, and alignment moves the scan from its seed.
constexpr double kSharedTargetMarginM = 10.0;

class G2NdtCandidateScorer
{
public:
  G2NdtCandidateScorer(const std::string & map_path, G2NdtScoreParams params)
  : params_(params), full_map_(load_cloud_xyzi(map_path)), ground_(*full_map_)
  {
  }

  double ground_z(double x, double y) const {return ground_.ground_z(x, y);}

  // Scores several poses of one scan. Preparing the NDT target (crop, voxel
  // filter, cells and k-d tree) costs several times the alignment, so a crop is
  // shared by every pose whose scan, plus kSharedTargetMarginM, lies inside it.
  std::vector<G2NdtScoreResult> score_many(
    const CloudPtr & source,
    const std::vector<G2NdtPose> & poses) const
  {
    std::vector<G2NdtScoreResult> results(poses.size());
    for (std::size_t i = 0; i < poses.size(); ++i) {
      results[i] = seed_result(source, poses[i]);
    }
    if (!source || source->empty()) {
      return results;
    }

    double scan_extent = 0.0;
    for (const auto & point : source->points) {
      scan_extent = std::max(scan_extent, std::hypot(double{point.x}, double{point.y}));
    }
    const double share_radius = params_.local_map_radius > 0.0 ?
      params_.local_map_radius - scan_extent - kSharedTargetMarginM :
      std::numeric_limits<double>::infinity();

    std::vector<bool> scored(poses.size(), false);
    for (std::size_t i = 0; i < poses.size(); ++i) {
      if (scored[i]) {
        continue;
      }
      CloudPtr target = crop_map_xy(full_map_, poses[i].x, poses[i].y, params_.local_map_radius);
      const std::size_t target_point_count = target->size();
      Ndt ndt;
      const bool usable = target_point_count >= params_.min_target_points;
      if (usable) {
        configure(ndt, voxel_downsample(target, params_.target_voxel_leaf_size), source);
      }
      for (std::size_t j = i; j < poses.size(); ++j) {
        if (scored[j] || (j != i && std::hypot(
            poses[j].x - poses[i].x, poses[j].y - poses[i].y) > share_radius))
        {
          continue;
        }
        scored[j] = true;
        results[j].target_point_count = target_point_count;
        if (usable) {
          align(ndt, poses[j], results[j]);
        }
      }
    }
    return results;
  }

  std::vector<G2NdtScoreResult> score_xyz_many(
    const double * xyz,
    std::size_t point_count,
    const std::vector<G2NdtPose> & poses) const
  {
    return score_many(cloud_from_xyz(xyz, point_count, params_.scan_voxel_leaf_size), poses);
  }

private:
  using Ndt = pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;

  static G2NdtScoreResult seed_result(const CloudPtr & source, const G2NdtPose & pose)
  {
    G2NdtScoreResult result;
    result.refined_x = pose.x;
    result.refined_y = pose.y;
    result.refined_z = pose.z;
    result.refined_yaw = pose.yaw;
    result.source_point_count = source ? source->size() : 0;
    return result;
  }

  void configure(Ndt & ndt, const CloudPtr & target, const CloudPtr & source) const
  {
    ndt.setResolution(params_.ndt_resolution);
    ndt.setStepSize(params_.ndt_step_size);
    ndt.setTransformationEpsilon(params_.transform_epsilon);
    ndt.setMaximumIterations(params_.max_iterations);
    ndt.setNumThreads(std::max(1, params_.num_threads));
    ndt.setNeighborhoodSearchMethod(
      static_cast<pclomp::NeighborSearchMethod>(params_.search_method));
    ndt.setInputTarget(target);
    ndt.setInputSource(source);
  }

  static void align(Ndt & ndt, const G2NdtPose & pose, G2NdtScoreResult & result)
  {
    const Eigen::Matrix4f init = pose_matrix(pose.x, pose.y, pose.z, pose.yaw);
    Cloud output;
    ndt.align(output, init);
    result.converged = ndt.hasConverged();
    const double fitness = ndt.getFitnessScore();
    result.fitness = std::isfinite(fitness) ? fitness : std::numeric_limits<double>::infinity();
    if (result.converged) {
      const Eigen::Matrix4f final = ndt.getFinalTransformation();
      result.refined_x = static_cast<double>(final(0, 3));
      result.refined_y = static_cast<double>(final(1, 3));
      result.refined_z = static_cast<double>(final(2, 3));
      result.refined_yaw = std::atan2(
        static_cast<double>(final(1, 0)),
        static_cast<double>(final(0, 0)));
    }
  }

  G2NdtScoreParams params_;
  CloudPtr full_map_;
  GroundHeightIndex ground_;
};

}  // namespace g2_ndt
}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_G2_NDT_CANDIDATE_SCORE_HPP_
