// Global localization without ROS: the G2 engine "bbs3d" search over the whole
// map (bbs3d_topk_search.hpp, on the voxel maps of 3D-BBS), each candidate
// refined by the caller's registration, and the decision of local
// re-acquisition (decideLocalReacquisition): the best refined candidate is
// proposed when it passes the score threshold and no other place fits nearly as
// well. global_localization_query.py runs the same search through bbs3d_cpp and
// refines with the G2 NDT scorer; lidar_localization_core's C API runs it with its
// registration engine (lidarloc_global_localize).
#pragma once

#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "lidar_localization/bbs3d_topk_search.hpp"
#include "lidar_localization/local_reacquisition.hpp"

namespace lidar_localization
{

struct GlobalLocalizationParams
{
  // The scan as the search sees it: points between these ranges, thinned to
  // voxel centroids.
  double scan_min_range_m{1.0};
  double scan_max_range_m{30.0};
  double scan_leaf_m{0.5};
  bbs3d::SearchParams search;
  // The decision (local_reacquisition_policy.hpp): a candidate that refined
  // farther than candidate_separation_m from the best must fit worse than
  // min_fitness_ratio times the best's fitness.
  double candidate_separation_m{2.0};
  double min_fitness_ratio{1.2};
};

struct GlobalCandidate
{
  // The search's pose (on its voxel and angle lattice) and score.
  Eigen::Matrix4f search_pose{Eigen::Matrix4f::Identity()};
  int search_score{0};
  // The registration's refinement of it.
  LocalReacquisitionRefinement refined;
};

struct GlobalLocalizationResult
{
  bool found{false};
  // decideLocalReacquisition's reason ("proposed", "ambiguous", ...), or
  // "no_map", "no_scan_points", "search_failed", "interrupted".
  std::string reason{"no_map"};
  // The pose found, or else the best refined candidate's (fitness infinite
  // when none converged).
  Eigen::Matrix4f pose{Eigen::Matrix4f::Identity()};
  double fitness{std::numeric_limits<double>::infinity()};
  std::vector<GlobalCandidate> candidates;
  // The search's own report (without its candidates).
  bbs3d::SearchResult search;
};

class GlobalLocalizer
{
public:
  // Builds the voxel hierarchy of the map, thinned to map_leaf_m voxel
  // centroids first (0: not thinned). Throws on an empty map or bad levels.
  void build(
    bbs3d::Points map_points, double map_leaf_m, double min_level_res_m, int max_level);
  bool ready() const {return static_cast<bool>(search_);}
  // The parameters the hierarchy was built with.
  bool builtWith(double map_leaf_m, double min_level_res_m, int max_level) const;
  void clear() {search_.reset();}

  // Searches the whole map for the scan (base frame, levelled), refines the
  // candidates with `refine` (nullopt stops), and decides.
  GlobalLocalizationResult localize(
    const bbs3d::Points & scan, const GlobalLocalizationParams & params, double score_threshold,
    const LocalReacquisitionRefine & refine) const;

private:
  std::unique_ptr<bbs3d::TopkSearch> search_;
  double map_leaf_m_{0.0};
  double min_level_res_m_{0.0};
  int max_level_{0};
};

}  // namespace lidar_localization
