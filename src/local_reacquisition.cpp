#include "lidar_localization/local_reacquisition.hpp"

#include <cmath>

#include <Eigen/Geometry>

namespace lidar_localization
{

LocalReacquisitionProposal proposeLocalReacquisition(
  const OccupancyGridMap & map, const std::vector<std::array<double, 3>> & scan_points,
  const Eigen::Matrix4f & bridged, const LocalReacquisitionParams & params,
  double score_threshold, const LocalReacquisitionRefine & refine)
{
  LocalReacquisitionProposal proposal;
  const double bridged_yaw = std::atan2(bridged(1, 0), bridged(0, 0));
  const auto scan_xy = prepareReacquisitionScanXy(scan_points, params);
  const auto window =
    occupancyWindowAround(map, bridged(0, 3), bridged(1, 3), params.search_radius_m);
  if (scan_xy.empty() || window.width <= 0 || window.height <= 0) {
    proposal.reason = "empty_scan_or_window";
    return proposal;
  }
  const auto bbs_candidates = bbs::branch_and_bound_candidates(
    cropGrid(map.grid, window),
    scan_xy,
    map.resolution_m,
    params.angular_resolution_deg * M_PI / 180.0,
    params.pyramid_depth,
    16,
    static_cast<int>(std::lround(1.0 / map.resolution_m)),
    bridged_yaw,
    params.yaw_window_deg * M_PI / 180.0);
  const auto candidates =
    selectHeadingConsistentCandidates(bbs_candidates, map, window, bridged_yaw, params);
  proposal.bbs_candidates = bbs_candidates.size();
  proposal.heading_candidates = candidates.size();

  std::vector<RefinedReacquisitionCandidate> refined;
  std::vector<Eigen::Matrix4f> refined_poses;
  for (const auto & candidate : candidates) {
    Eigen::Matrix4f guess = bridged;
    const Eigen::Matrix3f yaw_delta = Eigen::AngleAxisf(
      static_cast<float>(wrapAngleRad(candidate.yaw_rad - bridged_yaw)),
      Eigen::Vector3f::UnitZ()).toRotationMatrix();
    guess.block<3, 3>(0, 0) = yaw_delta * bridged.block<3, 3>(0, 0);
    guess(0, 3) = static_cast<float>(candidate.x_m);
    guess(1, 3) = static_cast<float>(candidate.y_m);
    const auto refinement = refine(guess);
    if (!refinement) {
      proposal.interrupted = true;
      proposal.reason = "interrupted";
      return proposal;
    }
    RefinedReacquisitionCandidate result;
    result.converged = refinement->converged && refinement->pose.allFinite();
    result.fitness = result.converged ? refinement->fitness :
      std::numeric_limits<double>::infinity();
    result.x_m = refinement->pose(0, 3);
    result.y_m = refinement->pose(1, 3);
    refined.push_back(result);
    refined_poses.push_back(refinement->pose);
  }

  const auto decision = decideLocalReacquisition(refined, score_threshold, params);
  proposal.reason = decision.reason;
  if (decision.propose) {
    proposal.propose = true;
    proposal.pose = refined_poses[decision.index];
    proposal.fitness = refined[decision.index].fitness;
  }
  return proposal;
}

OccupancyGridMap occupancyGridFromGenerated(const GeneratedOccupancyMap & generated)
{
  OccupancyGridMap map;
  map.grid = bbs::Grid(generated.height, generated.width);
  map.resolution_m = generated.resolution_m;
  map.origin_x_m = generated.origin_x_m;
  map.origin_y_m = generated.origin_y_m;
  for (int row = 0; row < generated.height; ++row) {
    for (int col = 0; col < generated.width; ++col) {
      const auto pixel = generated.pixels[static_cast<std::size_t>(row) * generated.width + col];
      map.grid.set(generated.height - 1 - row, col, pixel == kOccupancyOccupied ? 1 : 0);
    }
  }
  return map;
}

}  // namespace lidar_localization
