#ifndef LIDAR_LOCALIZATION_LOCAL_REACQUISITION_POLICY_HPP_
#define LIDAR_LOCALIZATION_LOCAL_REACQUISITION_POLICY_HPP_

// Local re-acquisition after an odometry-bridged outage.
//
// While scans are rejected and the pose is carried by external odometry
// (use_odom_tf_prediction), the bridged pose drifts.  Once the sensor is back
// in mapped space the drift can exceed the NDT convergence basin, so normal
// registration never recovers.  This policy searches only a window around the
// bridged pose: 2D branch-and-bound on the occupancy grid proposes candidates,
// candidates far from the bridged heading are discarded, each remaining
// candidate is refined with the normal registration, and one is proposed only
// when it fits and is not ambiguous.  The node then seeds the next scan from
// the proposal and keeps it only if that scan passes the normal measurement
// gate.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "lidar_localization/bbs_branch_and_bound.hpp"

namespace lidar_localization
{

struct LocalReacquisitionParams
{
  bool enable{false};
  int min_rejections{10};
  int attempt_interval_scans{10};
  double search_radius_m{15.0};
  double yaw_window_deg{20.0};
  int max_candidates{6};
  double candidate_separation_m{2.0};
  double min_fitness_ratio{1.2};
  double scan_min_z_m{0.5};
  double scan_max_z_m{5.0};
  double scan_min_range_m{1.0};
  double scan_voxel_m{0.2};
  int scan_max_points{512};
  double angular_resolution_deg{2.0};
  int pyramid_depth{4};
};

struct LocalReacquisitionTriggerInput
{
  bool odom_bridge_seed{false};
  bool occupancy_ready{false};
  bool proposal_pending{false};
  std::size_t consecutive_rejected_updates{0};
  std::size_t scans_since_last_attempt{0};
};

inline bool shouldAttemptLocalReacquisition(
  const LocalReacquisitionParams & params, const LocalReacquisitionTriggerInput & input)
{
  return params.enable && input.odom_bridge_seed && input.occupancy_ready &&
         !input.proposal_pending &&
         input.consecutive_rejected_updates >=
         static_cast<std::size_t>(std::max(1, params.min_rejections)) &&
         input.scans_since_last_attempt >=
         static_cast<std::size_t>(std::max(1, params.attempt_interval_scans));
}

// Occupancy grid in the map frame. Row 0 is the lowest y (the image is
// flipped on load, matching scripts/make_bbs_relocalization_attempts.py).
struct OccupancyGridMap
{
  bbs::Grid grid;
  double resolution_m{0.0};
  double origin_x_m{0.0};
  double origin_y_m{0.0};
};

namespace local_reacquisition_detail
{

inline std::string trim(const std::string & value)
{
  const auto begin = value.find_first_not_of(" \t\r\"'");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\"'");
  return value.substr(begin, end - begin + 1);
}

inline std::vector<double> parseList(const std::string & value)
{
  std::string inner = trim(value);
  if (!inner.empty() && inner.front() == '[') {
    inner = inner.substr(1);
  }
  if (!inner.empty() && inner.back() == ']') {
    inner.pop_back();
  }
  std::vector<double> numbers;
  std::stringstream stream(inner);
  std::string token;
  while (std::getline(stream, token, ',')) {
    numbers.push_back(std::stod(trim(token)));
  }
  return numbers;
}

}  // namespace local_reacquisition_detail

// Reads a ROS map_server YAML + 8-bit binary PGM (P5). Throws on unsupported input.
inline OccupancyGridMap loadOccupancyGridMap(const std::string & yaml_path)
{
  namespace detail = local_reacquisition_detail;
  std::ifstream yaml(yaml_path);
  if (!yaml) {
    throw std::runtime_error("cannot open occupancy yaml: " + yaml_path);
  }
  std::string image;
  double resolution = 0.0;
  std::vector<double> origin{0.0, 0.0, 0.0};
  int negate = 0;
  double occupied_thresh = 0.65;
  std::string line;
  while (std::getline(yaml, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) {
      line = line.substr(0, hash);
    }
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = detail::trim(line.substr(0, colon));
    const std::string value = detail::trim(line.substr(colon + 1));
    if (key == "image") {
      image = value;
    } else if (key == "resolution") {
      resolution = std::stod(value);
    } else if (key == "origin") {
      origin = detail::parseList(value);
    } else if (key == "negate") {
      negate = std::stoi(value);
    } else if (key == "occupied_thresh") {
      occupied_thresh = std::stod(value);
    }
  }
  if (image.empty() || !(resolution > 0.0) || origin.size() < 2) {
    throw std::runtime_error("incomplete occupancy yaml: " + yaml_path);
  }
  if (origin.size() > 2 && std::abs(origin[2]) > 1e-9) {
    throw std::runtime_error("rotated occupancy origins are not supported: " + yaml_path);
  }
  if (image.front() != '/') {
    const auto slash = yaml_path.find_last_of('/');
    image = (slash == std::string::npos ? std::string() : yaml_path.substr(0, slash + 1)) + image;
  }

  std::ifstream pgm(image, std::ios::binary);
  if (!pgm) {
    throw std::runtime_error("cannot open occupancy image: " + image);
  }
  std::string magic;
  pgm >> magic;
  if (magic != "P5") {
    throw std::runtime_error("only binary PGM (P5) occupancy images are supported: " + image);
  }
  std::array<int, 3> header{};
  for (int & field : header) {
    while (pgm >> std::ws && pgm.peek() == '#') {
      std::getline(pgm, line);
    }
    pgm >> field;
  }
  const int width = header[0];
  const int height = header[1];
  const int max_value = header[2];
  if (width <= 0 || height <= 0 || max_value <= 0 || max_value > 255) {
    throw std::runtime_error("unsupported PGM header: " + image);
  }
  pgm.get();
  std::vector<unsigned char> pixels(static_cast<std::size_t>(width) * height);
  pgm.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
  if (pgm.gcount() != static_cast<std::streamsize>(pixels.size())) {
    throw std::runtime_error("truncated PGM pixels: " + image);
  }

  OccupancyGridMap map;
  map.grid = bbs::Grid(height, width);
  map.resolution_m = resolution;
  map.origin_x_m = origin[0];
  map.origin_y_m = origin[1];
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      const double value =
        static_cast<double>(pixels[static_cast<std::size_t>(row) * width + col]) *
        255.0 / max_value;
      const double occupied = negate ? value / 255.0 : (255.0 - value) / 255.0;
      // Flip vertically so that grid row 0 is the lowest map y.
      map.grid.set(height - 1 - row, col, occupied >= occupied_thresh ? 1 : 0);
    }
  }
  return map;
}

struct GridWindow
{
  int x0{0};
  int y0{0};
  int width{0};
  int height{0};
};

// Cells covering a square of half-size radius_m around (x, y), clipped to the map.
inline GridWindow occupancyWindowAround(
  const OccupancyGridMap & map, double x_m, double y_m, double radius_m)
{
  GridWindow window;
  if (!(map.resolution_m > 0.0) || !(radius_m > 0.0)) {
    return window;
  }
  const int x0 = static_cast<int>(std::floor((x_m - radius_m - map.origin_x_m) / map.resolution_m));
  const int y0 = static_cast<int>(std::floor((y_m - radius_m - map.origin_y_m) / map.resolution_m));
  const int size = static_cast<int>(std::ceil(2.0 * radius_m / map.resolution_m));
  const int x1 = std::min(map.grid.width, x0 + size);
  const int y1 = std::min(map.grid.height, y0 + size);
  window.x0 = std::max(0, x0);
  window.y0 = std::max(0, y0);
  window.width = std::max(0, x1 - window.x0);
  window.height = std::max(0, y1 - window.y0);
  return window;
}

inline bbs::Grid cropGrid(const bbs::Grid & grid, const GridWindow & window)
{
  bbs::Grid out(window.height, window.width);
  for (int y = 0; y < window.height; ++y) {
    for (int x = 0; x < window.width; ++x) {
      out.set(y, x, grid.at(window.y0 + y, window.x0 + x));
    }
  }
  return out;
}

inline double wrapAngleRad(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

struct LocalReacquisitionCandidate
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double bbs_score{0.0};
};

// Converts window-relative BBS candidates to map coordinates, keeps those whose
// heading is within yaw_window_deg of the bridged heading, and returns at most
// max_candidates in BBS order.
inline std::vector<LocalReacquisitionCandidate> selectHeadingConsistentCandidates(
  const std::vector<bbs::BbsGridCandidate> & candidates,
  const OccupancyGridMap & map,
  const GridWindow & window,
  double bridged_yaw_rad,
  const LocalReacquisitionParams & params)
{
  std::vector<LocalReacquisitionCandidate> kept;
  const double window_rad = params.yaw_window_deg * M_PI / 180.0;
  for (const auto & candidate : candidates) {
    if (std::abs(wrapAngleRad(candidate.yaw_rad - bridged_yaw_rad)) > window_rad) {
      continue;
    }
    kept.push_back({
        map.origin_x_m + (window.x0 + candidate.tx_cell + 0.5) * map.resolution_m,
        map.origin_y_m + (window.y0 + candidate.ty_cell + 0.5) * map.resolution_m,
        candidate.yaw_rad,
        candidate.score});
    if (static_cast<int>(kept.size()) >= std::max(1, params.max_candidates)) {
      break;
    }
  }
  return kept;
}

// Scan points (base frame) projected to a thinned 2D set, as in the G2 query.
inline std::vector<std::array<double, 2>> prepareReacquisitionScanXy(
  const std::vector<std::array<double, 3>> & points, const LocalReacquisitionParams & params)
{
  std::vector<std::array<double, 2>> xy;
  std::set<std::pair<std::int64_t, std::int64_t>> seen;
  const double voxel = params.scan_voxel_m > 0.0 ? params.scan_voxel_m : 0.2;
  for (const auto & point : points) {
    if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) {
      continue;
    }
    if (point[2] < params.scan_min_z_m || point[2] > params.scan_max_z_m) {
      continue;
    }
    if (std::hypot(point[0], point[1]) < std::max(0.0, params.scan_min_range_m)) {
      continue;
    }
    const std::pair<std::int64_t, std::int64_t> cell{
      static_cast<std::int64_t>(std::floor(point[0] / voxel)),
      static_cast<std::int64_t>(std::floor(point[1] / voxel))};
    if (!seen.insert(cell).second) {
      continue;
    }
    xy.push_back({point[0], point[1]});
  }
  if (params.scan_max_points > 0 && static_cast<int>(xy.size()) > params.scan_max_points) {
    std::vector<std::array<double, 2>> thinned;
    const double step =
      static_cast<double>(xy.size() - 1) / static_cast<double>(params.scan_max_points - 1);
    for (int i = 0; i < params.scan_max_points; ++i) {
      thinned.push_back(xy[static_cast<std::size_t>(i * step)]);
    }
    xy.swap(thinned);
  }
  return xy;
}

struct RefinedReacquisitionCandidate
{
  bool converged{false};
  double fitness{std::numeric_limits<double>::infinity()};
  double x_m{0.0};
  double y_m{0.0};
};

struct LocalReacquisitionDecision
{
  bool propose{false};
  std::size_t index{0};
  std::string reason{"no_candidates"};
};

// Proposes the best-fitting refined candidate only if it passes the normal
// score threshold and every candidate that refined to a different place
// (farther than candidate_separation_m) fits clearly worse.
inline LocalReacquisitionDecision decideLocalReacquisition(
  const std::vector<RefinedReacquisitionCandidate> & refined,
  double score_threshold,
  const LocalReacquisitionParams & params)
{
  LocalReacquisitionDecision decision;
  std::optional<std::size_t> best;
  for (std::size_t i = 0; i < refined.size(); ++i) {
    const auto & candidate = refined[i];
    if (!candidate.converged || !std::isfinite(candidate.fitness)) {
      continue;
    }
    if (!best || candidate.fitness < refined[*best].fitness) {
      best = i;
    }
  }
  if (!best) {
    decision.reason = refined.empty() ? "no_candidates" : "no_converged_candidate";
    return decision;
  }
  const auto & winner = refined[*best];
  if (!(winner.fitness < score_threshold)) {
    decision.reason = "best_fitness_above_threshold";
    return decision;
  }
  for (std::size_t i = 0; i < refined.size(); ++i) {
    const auto & other = refined[i];
    if (i == *best || !other.converged || !std::isfinite(other.fitness)) {
      continue;
    }
    const double separation = std::hypot(other.x_m - winner.x_m, other.y_m - winner.y_m);
    if (separation > params.candidate_separation_m &&
      other.fitness < winner.fitness * params.min_fitness_ratio)
    {
      decision.reason = "ambiguous";
      return decision;
    }
  }
  decision.propose = true;
  decision.index = *best;
  decision.reason = "proposed";
  return decision;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_LOCAL_REACQUISITION_POLICY_HPP_
