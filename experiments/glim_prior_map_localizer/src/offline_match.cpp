#include "glim_prior_map_localizer/ply_loader.hpp"
#include "glim_prior_map_localizer/vgicp_refiner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/mapping/sub_map.hpp>
#include <kiss_matcher/KISSMatcher.hpp>

namespace
{

std::vector<Eigen::Vector3f> loadSubmapPoints(const glim::SubMap & submap)
{
  std::vector<Eigen::Vector3f> points;
  points.reserve(submap.frame->size());
  for (std::size_t index = 0; index < submap.frame->size(); ++index) {
    const auto point = submap.frame->points[index].head<3>().cast<float>();
    if (point.allFinite()) {
      points.push_back(point);
    }
  }
  return points;
}

double rotationDegrees(const Eigen::Matrix3d & rotation)
{
  constexpr double radians_to_degrees = 57.295779513082320876;
  const double cosine = std::clamp((rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cosine) * radians_to_degrees;
}

std::vector<Eigen::Vector3f> cropMap(
  const std::vector<Eigen::Vector3f> & map,
  const Eigen::Vector3d & center,
  double radius_m)
{
  if (radius_m <= 0.0) {
    return map;
  }
  std::vector<Eigen::Vector3f> cropped;
  const float squared_radius = static_cast<float>(radius_m * radius_m);
  const Eigen::Vector3f center_f = center.cast<float>();
  for (const auto & point : map) {
    if ((point - center_f).squaredNorm() <= squared_radius) {
      cropped.push_back(point);
    }
  }
  return cropped;
}

}  // namespace

int main(int argc, char ** argv)
try
{
  if (argc < 3 || argc > 6) {
    std::cerr << "usage: " << argv[0]
              << " SUBMAP_DIRECTORY PRIOR_MAP.ply [VOXEL_M] [CROP_RADIUS_M] [direct]\n";
    return 2;
  }
  const float voxel_size = argc == 4 ? std::stof(argv[3]) : 0.5F;
  const auto submap = glim::SubMap::load(argv[1]);
  if (!submap || !submap->frame) {
    throw std::runtime_error("failed to load GLIM submap: " + std::string(argv[1]));
  }
  const auto source = loadSubmapPoints(*submap);
  const double crop_radius_m = argc >= 5 ? std::stod(argv[4]) : 0.0;
  const bool direct_tracking = argc == 6 && std::string(argv[5]) == "direct";
  const auto full_map = glim_prior_map_localizer::loadPlyXyz(argv[2]);
  const auto target = cropMap(full_map, submap->T_world_origin.translation(), crop_radius_m);

  kiss_matcher::KISSMatcherConfig config(voxel_size);
  config.use_quatro_ = true;
  config.use_ratio_test_ = true;
  config.num_max_corr_ = 5000;
  kiss_matcher::KISSMatcher matcher(config);

  const auto start = std::chrono::steady_clock::now();
  const auto solution = matcher.estimate(source, target);
  const auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start).count();

  Eigen::Isometry3d estimate = Eigen::Isometry3d::Identity();
  estimate.linear() = solution.rotation;
  estimate.translation() = solution.translation;
  const auto refinement = glim_prior_map_localizer::refineWithVgicp(
    target, submap->frame,
    direct_tracking ? submap->T_world_origin : estimate, 1.0, 10);
  const Eigen::Isometry3d refined = refinement.valid ? refinement.pose : estimate;
  const Eigen::Isometry3d error = refined * submap->T_world_origin.inverse();

  std::cout << std::boolalpha << std::fixed << std::setprecision(6)
            << "{\"valid\":" << solution.valid
            << ",\"submap_id\":" << submap->id
            << ",\"source_points\":" << source.size()
            << ",\"target_points\":" << target.size()
            << ",\"final_inliers\":" << matcher.getNumFinalInliers()
            << ",\"refinement_valid\":" << refinement.valid
            << ",\"refinement_inliers\":" << refinement.num_inliers
            << ",\"refinement_inlier_fraction\":" << refinement.inlier_fraction
            << ",\"refinement_normalized_error\":" << refinement.normalized_error
            << ",\"elapsed_s\":" << elapsed
            << ",\"recorded_pose_translation_error_m\":" << error.translation().norm()
            << ",\"recorded_pose_rotation_error_deg\":" << rotationDegrees(error.linear())
            << ",\"matrix\":[";
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      if (row != 0 || column != 0) {
        std::cout << ',';
      }
      std::cout << refined.matrix()(row, column);
    }
  }
  std::cout << "]}\n";
  return solution.valid ? 0 : 1;
}
catch (const std::exception & error)
{
  std::cerr << error.what() << '\n';
  return 2;
}
#include "glim_prior_map_localizer/ply_loader.hpp"
