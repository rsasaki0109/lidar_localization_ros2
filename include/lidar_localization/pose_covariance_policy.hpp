#ifndef LIDAR_LOCALIZATION_POSE_COVARIANCE_POLICY_HPP_
#define LIDAR_LOCALIZATION_POSE_COVARIANCE_POLICY_HPP_

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Core>

namespace lidar_localization
{

using PoseCovariance = std::array<double, 36>;
using EkfStateCovariance = Eigen::Matrix<double, 9, 9>;

// Error-floor covariance model, calibrated against ground truth on the public
// Koide outdoor_hard_01a and Autoware Istanbul replays (2026-06-12). Within the
// accepted-fitness range the actual error is dominated by a per-dataset floor,
// not by fitness, so the model is a floor plus a mild fitness slope:
//   std = clamp(floor + per_fitness * fitness, floor, max)
// Defaults target ~1 sigma coverage on the worst observed floor (Istanbul) and
// are conservative on tighter maps. Multi-meter errors on accepted poses exist
// in the tails (systematically biased degenerate matches); no fitness-based
// model can bound those, which is why max_std stays finite and consumers must
// also watch /alignment_status failure_category.
struct ErrorFloorCovarianceParams
{
  double xy_floor_std_m{0.2};
  double xy_std_per_fitness_m{0.1};
  double xy_max_std_m{5.0};
  double z_floor_std_m{0.3};
  double yaw_floor_std_deg{2.0};
  double yaw_std_per_fitness_deg{1.0};
  double yaw_max_std_deg{30.0};
  double roll_pitch_floor_std_deg{1.5};
};

inline double errorFloorStd(
  double floor_std, double std_per_fitness, double max_std, double fitness_score)
{
  const double fitness =
    std::isfinite(fitness_score) ? std::max(0.0, fitness_score) : 0.0;
  return std::clamp(floor_std + std_per_fitness * fitness, floor_std, max_std);
}

inline PoseCovariance makeErrorFloorPoseCovariance(
  const ErrorFloorCovarianceParams & params, double fitness_score)
{
  const double xy_std = errorFloorStd(
    params.xy_floor_std_m, params.xy_std_per_fitness_m, params.xy_max_std_m,
    fitness_score);
  const double z_std = errorFloorStd(
    params.z_floor_std_m, params.xy_std_per_fitness_m, params.xy_max_std_m,
    fitness_score);
  const double yaw_std_deg = errorFloorStd(
    params.yaw_floor_std_deg, params.yaw_std_per_fitness_deg,
    params.yaw_max_std_deg, fitness_score);
  const double roll_pitch_std_deg = errorFloorStd(
    params.roll_pitch_floor_std_deg, params.yaw_std_per_fitness_deg,
    params.yaw_max_std_deg, fitness_score);
  const double yaw_std = yaw_std_deg * M_PI / 180.0;
  const double roll_pitch_std = roll_pitch_std_deg * M_PI / 180.0;

  PoseCovariance covariance{};
  covariance[0] = xy_std * xy_std;
  covariance[7] = xy_std * xy_std;
  covariance[14] = z_std * z_std;
  covariance[21] = roll_pitch_std * roll_pitch_std;
  covariance[28] = roll_pitch_std * roll_pitch_std;
  covariance[35] = yaw_std * yaw_std;
  return covariance;
}

inline PoseCovariance makeEkfErrorFloorPoseCovariance(
  const EkfStateCovariance & ekf_covariance,
  const ErrorFloorCovarianceParams & params,
  double fitness_score)
{
  auto covariance = makeErrorFloorPoseCovariance(params, fitness_score);
  covariance[0] = ekf_covariance(0, 0);
  covariance[1] = ekf_covariance(0, 1);
  covariance[6] = ekf_covariance(1, 0);
  covariance[7] = ekf_covariance(1, 1);
  covariance[14] = ekf_covariance(2, 2);
  covariance[35] = ekf_covariance(6, 6);
  return covariance;
}

inline double poseCovarianceScale(double fitness_score)
{
  return std::max(1.0, fitness_score);
}

inline PoseCovariance makeFitnessPoseCovariance(double fitness_score)
{
  PoseCovariance covariance{};
  const double scale = poseCovarianceScale(fitness_score);
  covariance[0] = 0.01 * scale;
  covariance[7] = 0.01 * scale;
  covariance[14] = 0.05 * scale;
  covariance[21] = 0.001 * scale;
  covariance[28] = 0.001 * scale;
  covariance[35] = 0.0005 * scale;
  return covariance;
}

inline PoseCovariance makeEkfPoseCovariance(
  const EkfStateCovariance & ekf_covariance,
  double fitness_score)
{
  PoseCovariance covariance{};
  covariance[0] = ekf_covariance(0, 0);
  covariance[1] = ekf_covariance(0, 1);
  covariance[6] = ekf_covariance(1, 0);
  covariance[7] = ekf_covariance(1, 1);
  covariance[14] = ekf_covariance(2, 2);
  covariance[35] = ekf_covariance(6, 6);

  const double scale = poseCovarianceScale(fitness_score);
  covariance[21] = 0.001 * scale;
  covariance[28] = 0.001 * scale;
  return covariance;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_POSE_COVARIANCE_POLICY_HPP_
