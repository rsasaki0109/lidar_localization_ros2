// Local re-acquisition without ROS: the search the node runs after an
// odometry-bridged outage (local_reacquisition_policy.hpp), over an
// occupancy grid, with the refinement of each candidate left to the caller
// (the node's alignment attempt, or the C API's).
#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "lidar_localization/local_reacquisition_policy.hpp"
#include "lidar_localization/occupancy_map_generation.hpp"

namespace lidar_localization
{

// One candidate refined by the registration.
struct LocalReacquisitionRefinement
{
  bool converged{false};
  double fitness{std::numeric_limits<double>::infinity()};
  Eigen::Matrix4f pose{Eigen::Matrix4f::Identity()};
};

struct LocalReacquisitionProposal
{
  bool propose{false};
  // The refinement stopped the search (the node: a newer initial pose).
  bool interrupted{false};
  // decideLocalReacquisition's reason, or "empty_scan_or_window".
  std::string reason{"no_candidates"};
  std::size_t bbs_candidates{0};
  std::size_t heading_candidates{0};
  // The proposed pose and its fitness.
  Eigen::Matrix4f pose{Eigen::Matrix4f::Identity()};
  double fitness{std::numeric_limits<double>::infinity()};
};

// nullopt stops the search.
using LocalReacquisitionRefine =
  std::function<std::optional<LocalReacquisitionRefinement>(const Eigen::Matrix4f & guess)>;

// Searches a window of the occupancy grid around the bridged pose with 2D
// branch-and-bound, keeps the candidates near the bridged heading, refines
// each from the bridged pose's height and attitude, and proposes the best one
// when it passes the score threshold and is not ambiguous. scan_points are in
// the base frame.
LocalReacquisitionProposal proposeLocalReacquisition(
  const OccupancyGridMap & map, const std::vector<std::array<double, 3>> & scan_points,
  const Eigen::Matrix4f & bridged, const LocalReacquisitionParams & params,
  double score_threshold, const LocalReacquisitionRefine & refine);

// The grid generateOccupancyMap made, as loadOccupancyGridMap reads its PGM:
// occupied cells set, row 0 the lowest y.
OccupancyGridMap occupancyGridFromGenerated(const GeneratedOccupancyMap & generated);

}  // namespace lidar_localization
