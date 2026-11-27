#include "lidar_localization/global_localization.hpp"

#include <utility>

namespace lidar_localization
{

void GlobalLocalizer::build(
  bbs3d::Points map_points, double map_leaf_m, double min_level_res_m, int max_level)
{
  if (map_leaf_m > 0.0) {
    map_points = bbs3d::voxel_centroid_filter(map_points, map_leaf_m);
  }
  search_ = std::make_unique<bbs3d::TopkSearch>(map_points, min_level_res_m, max_level);
  map_leaf_m_ = map_leaf_m;
  min_level_res_m_ = min_level_res_m;
  max_level_ = max_level;
}

bool GlobalLocalizer::builtWith(double map_leaf_m, double min_level_res_m, int max_level) const
{
  return search_ && map_leaf_m_ == map_leaf_m && min_level_res_m_ == min_level_res_m &&
         max_level_ == max_level;
}

GlobalLocalizationResult GlobalLocalizer::localize(
  const bbs3d::Points & scan, const GlobalLocalizationParams & params, double score_threshold,
  const LocalReacquisitionRefine & refine) const
{
  GlobalLocalizationResult result;
  if (!search_) {
    return result;
  }
  const auto prepared = bbs3d::voxel_centroid_filter(
    bbs3d::range_filter(scan, params.scan_min_range_m, params.scan_max_range_m),
    params.scan_leaf_m);
  if (prepared.empty()) {
    result.reason = "no_scan_points";
    return result;
  }
  try {
    result.search = search_->search(prepared, params.search);
  } catch (const std::exception &) {
    result.reason = "search_failed";
    return result;
  }
  std::vector<RefinedReacquisitionCandidate> refined;
  for (const auto & candidate : result.search.candidates) {
    GlobalCandidate global;
    global.search_pose = candidate.pose.cast<float>();
    global.search_score = candidate.score;
    const auto refinement = refine(global.search_pose);
    if (!refinement) {
      result.reason = "interrupted";
      return result;
    }
    global.refined = *refinement;
    RefinedReacquisitionCandidate summary;
    summary.converged = refinement->converged && refinement->pose.allFinite();
    summary.fitness = summary.converged ? refinement->fitness :
      std::numeric_limits<double>::infinity();
    summary.x_m = refinement->pose(0, 3);
    summary.y_m = refinement->pose(1, 3);
    refined.push_back(summary);
    result.candidates.push_back(global);
  }
  result.search.candidates.clear();
  LocalReacquisitionParams decision_params;
  decision_params.candidate_separation_m = params.candidate_separation_m;
  decision_params.min_fitness_ratio = params.min_fitness_ratio;
  const auto decision = decideLocalReacquisition(refined, score_threshold, decision_params);
  result.reason = decision.reason;
  if (decision.propose) {
    result.found = true;
    result.pose = result.candidates[decision.index].refined.pose;
    result.fitness = refined[decision.index].fitness;
  }
  return result;
}

}  // namespace lidar_localization
