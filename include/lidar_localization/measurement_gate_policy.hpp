#ifndef LIDAR_LOCALIZATION_MEASUREMENT_GATE_POLICY_HPP_
#define LIDAR_LOCALIZATION_MEASUREMENT_GATE_POLICY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <cstdint>
#include <string>

namespace lidar_localization
{

constexpr std::uint8_t kMeasurementGateOk = 0;
constexpr std::uint8_t kMeasurementGateWarn = 1;

struct MeasurementGateParams
{
  double score_threshold{2.0};
  bool reject_above_score_threshold{true};

  bool enable_post_reject_strict_score_threshold{false};
  int post_reject_strict_min_rejections{100};
  double post_reject_strict_score_threshold{5.5};

  bool enable_borderline_seed_rejection_gate{false};
  double borderline_seed_gate_score_threshold{5.25};
  double borderline_seed_gate_min_seed_translation_m{1.0};

  // Seed-agnostic jump guard.  Whatever produced the seed (previous delta,
  // twist, IMU, odom), a legged or wheeled robot cannot move meters between
  // two scans, so a result that lands far from the seed is an aliased
  // solution even with a good fitness.  Accepting it also poisons the
  // previous-delta prediction and the estimate runs away.  After
  // release_rejections consecutive rejections the guard stops applying so a
  // genuinely drifted seed can still re-lock.
  bool enable_seed_correction_guard{false};
  double seed_correction_guard_translation_m{0.5};
  double seed_correction_guard_yaw_deg{15.0};
  int seed_correction_guard_release_rejections{10};
  // The guard only protects an established track: it waits for this many
  // consecutive accepted updates whose corrections stayed within its limits
  // (see nextSettledAccepts). A pose from global search can be metres off,
  // and NDT must be free to pull it in before the guard holds it.
  int seed_correction_guard_warmup_accepts{5};
  // When the first accepted update after an initial pose corrects it by at
  // most this much (and within the guard's yaw limit), the pose needed no
  // pulling in, so the warmup counts as done and the guard holds from the
  // next scan. 0 disables this. Aliased corrections right after an exact
  // start, e.g. while the scene differs from the map, are then rejected.
  double seed_correction_guard_exact_start_translation_m{0.0};

  bool enable_rejected_seed_update{false};
  int rejected_seed_update_min_rejections{0};
  double rejected_seed_update_max_fitness{10.0};
  double rejected_seed_update_max_correction_translation_m{2.0};
  double rejected_seed_update_max_correction_yaw_deg{2.0};
};

struct MeasurementGateInput
{
  double fitness_score{0.0};
  double accepted_gap_sec{0.0};
  double seed_translation_since_accept_m{0.0};
  double correction_translation_m{0.0};
  double correction_yaw_deg{0.0};
  std::size_t consecutive_rejected_updates{0};
  // Consecutive accepted updates within the seed correction guard's limits
  // since the last reset or odometry gap (see nextSettledAccepts).
  std::size_t accepted_updates_since_reset{std::numeric_limits<std::size_t>::max()};
};

struct MeasurementGateParamConfig : MeasurementGateParams
{
};

struct EffectiveScoreThresholdDecision
{
  double effective_score_threshold{0.0};
  bool post_reject_strict_active{false};
};

struct MeasurementGateDecision
{
  std::uint8_t status_level{kMeasurementGateOk};
  std::string status_message{"ok"};
  bool reject_measurement{false};
  bool post_reject_strict_active{false};
  bool borderline_seed_gate_active{false};
  bool rejected_seed_update_applied{false};
  double effective_score_threshold{0.0};
};

inline MeasurementGateParams makeMeasurementGateParams(
  const MeasurementGateParamConfig & config)
{
  return config;
}

inline MeasurementGateInput makeMeasurementGateInput(
  double fitness_score,
  double accepted_gap_sec,
  double seed_translation_since_accept_m,
  double correction_translation_m,
  double correction_yaw_deg,
  std::size_t consecutive_rejected_updates,
  std::size_t accepted_updates_since_reset = std::numeric_limits<std::size_t>::max())
{
  return {
    fitness_score,
    accepted_gap_sec,
    seed_translation_since_accept_m,
    correction_translation_m,
    correction_yaw_deg,
    consecutive_rejected_updates,
    accepted_updates_since_reset};
}

inline EffectiveScoreThresholdDecision computeEffectiveScoreThreshold(
  const MeasurementGateParams & params,
  const MeasurementGateInput & input)
{
  EffectiveScoreThresholdDecision decision;
  decision.post_reject_strict_active =
    params.enable_post_reject_strict_score_threshold &&
    static_cast<int>(input.consecutive_rejected_updates) >=
    params.post_reject_strict_min_rejections &&
    params.post_reject_strict_score_threshold < params.score_threshold;

  decision.effective_score_threshold = params.score_threshold;
  if (
    decision.post_reject_strict_active &&
    params.post_reject_strict_score_threshold < decision.effective_score_threshold)
  {
    decision.effective_score_threshold = params.post_reject_strict_score_threshold;
  }
  return decision;
}

inline bool isBorderlineSeedGateActive(
  const MeasurementGateParams & params,
  const MeasurementGateInput & input,
  double effective_score_threshold)
{
  return params.enable_borderline_seed_rejection_gate &&
         params.borderline_seed_gate_score_threshold < effective_score_threshold &&
         input.fitness_score > params.borderline_seed_gate_score_threshold &&
         input.fitness_score <= effective_score_threshold &&
         input.seed_translation_since_accept_m >=
         params.borderline_seed_gate_min_seed_translation_m;
}

// Count of consecutive accepted updates that needed only a small correction:
// an accept that moved the pose further than the seed correction guard allows
// means the track is still converging, so the count starts over.
// first_accept_since_initial_pose marks the first accepted update after an
// initial pose; see seed_correction_guard_exact_start_translation_m.
inline std::size_t nextSettledAccepts(
  std::size_t settled_accepts,
  const MeasurementGateParams & params,
  double correction_translation_m,
  double correction_yaw_deg,
  bool first_accept_since_initial_pose = false)
{
  const bool settled =
    correction_translation_m <= params.seed_correction_guard_translation_m &&
    correction_yaw_deg <= params.seed_correction_guard_yaw_deg;
  if (!settled) {
    return 0;
  }
  if (
    first_accept_since_initial_pose &&
    params.seed_correction_guard_exact_start_translation_m > 0.0 &&
    correction_translation_m <= params.seed_correction_guard_exact_start_translation_m)
  {
    return std::max(
      settled_accepts + 1,
      static_cast<std::size_t>(std::max(0, params.seed_correction_guard_warmup_accepts)));
  }
  return settled_accepts < std::numeric_limits<std::size_t>::max() ?
         settled_accepts + 1 : settled_accepts;
}

// Odometry seeds more than this far apart in time bridge a gap (a stalled or
// restarted front end, a covered sensor): the seed may be far off afterwards.
constexpr double kOdomSeedGapSec = 1.0;

inline bool isOdomSeedGap(double previous_seed_stamp_sec, double seed_stamp_sec)
{
  return std::isfinite(previous_seed_stamp_sec) &&
         seed_stamp_sec - previous_seed_stamp_sec > kOdomSeedGapSec;
}

inline MeasurementGateDecision evaluateMeasurementGate(
  const MeasurementGateParams & params,
  const MeasurementGateInput & input)
{
  MeasurementGateDecision gate;
  const auto threshold_decision = computeEffectiveScoreThreshold(params, input);
  gate.effective_score_threshold = threshold_decision.effective_score_threshold;
  gate.post_reject_strict_active = threshold_decision.post_reject_strict_active;
  if (!std::isfinite(input.fitness_score)) {
    gate.status_level = kMeasurementGateWarn;
    gate.status_message = "registration_fitness_non_finite";
    gate.reject_measurement = true;
    return gate;
  }
  gate.borderline_seed_gate_active =
    isBorderlineSeedGateActive(params, input, gate.effective_score_threshold);

  const bool seed_correction_guard_active =
    params.enable_seed_correction_guard &&
    input.accepted_updates_since_reset >=
    static_cast<std::size_t>(std::max(0, params.seed_correction_guard_warmup_accepts)) &&
    static_cast<int>(input.consecutive_rejected_updates) <
    params.seed_correction_guard_release_rejections &&
    ((input.correction_translation_m > params.seed_correction_guard_translation_m) ||
    (input.correction_yaw_deg > params.seed_correction_guard_yaw_deg));
  if (seed_correction_guard_active) {
    gate.status_level = kMeasurementGateWarn;
    gate.status_message = "seed_correction_guard_rejected";
    gate.reject_measurement = true;
    return gate;
  }

  const bool threshold_exceeded = input.fitness_score > gate.effective_score_threshold;
  if (!threshold_exceeded && !gate.borderline_seed_gate_active) {
    return gate;
  }

  gate.status_level = kMeasurementGateWarn;
  if (gate.borderline_seed_gate_active) {
    gate.status_message = params.reject_above_score_threshold ?
      "fitness_score_over_borderline_seed_gate_rejected" :
      "fitness_score_over_borderline_seed_gate";
  } else if (gate.post_reject_strict_active) {
    gate.status_message = params.reject_above_score_threshold ?
      "fitness_score_over_post_reject_strict_threshold_rejected" :
      "fitness_score_over_post_reject_strict_threshold";
  } else {
    gate.status_message = params.reject_above_score_threshold ?
      "fitness_score_over_threshold_rejected" :
      "fitness_score_over_threshold";
  }
  gate.reject_measurement = params.reject_above_score_threshold;

  const bool rejected_seed_update_candidate =
    params.enable_rejected_seed_update &&
    gate.reject_measurement &&
    static_cast<int>(input.consecutive_rejected_updates) >=
    params.rejected_seed_update_min_rejections &&
    input.fitness_score <= params.rejected_seed_update_max_fitness &&
    input.correction_translation_m <= params.rejected_seed_update_max_correction_translation_m &&
    input.correction_yaw_deg <= params.rejected_seed_update_max_correction_yaw_deg;
  if (rejected_seed_update_candidate) {
    gate.rejected_seed_update_applied = true;
    gate.status_message += "_seeded";
  }
  return gate;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_MEASUREMENT_GATE_POLICY_HPP_
