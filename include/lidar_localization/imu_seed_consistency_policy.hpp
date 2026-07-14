#ifndef LIDAR_LOCALIZATION_IMU_SEED_CONSISTENCY_POLICY_HPP_
#define LIDAR_LOCALIZATION_IMU_SEED_CONSISTENCY_POLICY_HPP_

#include <cmath>
#include <cstddef>

namespace lidar_localization
{

struct ImuSeedConsistencyParams
{
  double max_translation_error_m{0.5};
  double max_rotation_error_deg{5.0};
  std::size_t required_consecutive_passes{5};
};

struct ImuSeedConsistencyState
{
  std::size_t valid_comparison_count{0};
  std::size_t consecutive_pass_count{0};
  bool seed_allowed{false};
};

struct ImuSeedConsistencyInput
{
  bool lidar_observation_accepted{false};
  bool imu_prediction_available{false};
  double translation_error_m{0.0};
  double rotation_error_deg{0.0};
};

struct ImuSeedConsistencyUpdate
{
  ImuSeedConsistencyState state;
  bool comparison_valid{false};
  bool sample_passed{false};
};

inline bool validImuSeedConsistencyParams(const ImuSeedConsistencyParams & params)
{
  return std::isfinite(params.max_translation_error_m) &&
         params.max_translation_error_m > 0.0 &&
         std::isfinite(params.max_rotation_error_deg) &&
         params.max_rotation_error_deg > 0.0 &&
         params.required_consecutive_passes > 0;
}

inline ImuSeedConsistencyUpdate updateImuSeedConsistency(
  const ImuSeedConsistencyState & current,
  const ImuSeedConsistencyParams & params,
  const ImuSeedConsistencyInput & input)
{
  ImuSeedConsistencyUpdate update;
  update.state = current;
  update.comparison_valid =
    validImuSeedConsistencyParams(params) &&
    input.lidar_observation_accepted &&
    input.imu_prediction_available &&
    std::isfinite(input.translation_error_m) &&
    input.translation_error_m >= 0.0 &&
    std::isfinite(input.rotation_error_deg) &&
    input.rotation_error_deg >= 0.0;

  if (!validImuSeedConsistencyParams(params)) {
    update.state.consecutive_pass_count = 0;
    update.state.seed_allowed = false;
    return update;
  }

  if (!update.comparison_valid) {
    // A temporarily unavailable prediction supplies no evidence against a gate
    // that has already passed its warm-up comparisons. The caller cannot use
    // the seed while it is unavailable, so retaining the established state is
    // safe and avoids another full warm-up after a scheduling/scan gap.
    // Before the gate is established, a missing comparison still breaks the
    // required consecutive chain. A present but non-finite prediction is a
    // numerical failure and always fails closed.
    const bool missing_evidence =
      input.lidar_observation_accepted && !input.imu_prediction_available;
    if (!(current.seed_allowed && missing_evidence)) {
      update.state.consecutive_pass_count = 0;
      update.state.seed_allowed = false;
    }
    return update;
  }

  ++update.state.valid_comparison_count;
  update.sample_passed =
    input.translation_error_m <= params.max_translation_error_m &&
    input.rotation_error_deg <= params.max_rotation_error_deg;
  update.state.consecutive_pass_count = update.sample_passed ?
    current.consecutive_pass_count + 1 : 0;
  update.state.seed_allowed =
    update.state.consecutive_pass_count >= params.required_consecutive_passes;
  return update;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_IMU_SEED_CONSISTENCY_POLICY_HPP_
