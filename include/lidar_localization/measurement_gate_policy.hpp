#ifndef LIDAR_LOCALIZATION_MEASUREMENT_GATE_POLICY_HPP_
#define LIDAR_LOCALIZATION_MEASUREMENT_GATE_POLICY_HPP_

#include <cstddef>
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

  bool enable_consistency_recovery_gate{false};
  int consistency_recovery_min_rejections{10};
  double consistency_recovery_score_margin{2.0};
  double consistency_recovery_max_translation_m{0.05};
  double consistency_recovery_max_yaw_deg{0.5};

  bool enable_post_reject_strict_score_threshold{false};
  int post_reject_strict_min_rejections{100};
  double post_reject_strict_score_threshold{5.5};

  bool enable_open_loop_strict_score_threshold{false};
  double open_loop_strict_min_accepted_gap_sec{15.0};
  double open_loop_strict_min_seed_translation_m{100.0};
  double open_loop_strict_score_threshold{5.25};

  bool enable_borderline_seed_rejection_gate{false};
  double borderline_seed_gate_score_threshold{5.25};
  double borderline_seed_gate_min_seed_translation_m{1.0};

  // An external odometry seed is a strong motion prior.  A very low NDT
  // fitness can still be an aliased solution, so reject a result that jumps
  // implausibly far away from that prior.  This is independently opt-in to
  // preserve all existing configurations.
  bool enable_odom_tf_prediction_correction_guard{false};
  double odom_tf_prediction_correction_guard_translation_m{2.0};
  double odom_tf_prediction_correction_guard_yaw_deg{30.0};
  // After a sustained rejection streak, permit a larger correction only for
  // a very strong scan match. This lets a slowly drifting odom bridge snap
  // back to the map when distinctive geometry returns, without weakening the
  // normal alias guard during healthy tracking.
  bool enable_odom_tf_prediction_recovery_correction_guard{false};
  int odom_tf_prediction_recovery_min_rejections{30};
  double odom_tf_prediction_recovery_max_fitness{1.5};
  double odom_tf_prediction_recovery_guard_translation_m{5.0};
  double odom_tf_prediction_recovery_guard_yaw_deg{30.0};

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
  // True while the external-odom motion-model mode has an established
  // map->odom anchor.  It deliberately remains true for a scan that falls
  // back to previous_delta because the live TF lookup briefly missed; that
  // is exactly when an aliased NDT result must not replace the good anchor.
  bool odom_tf_prediction_guard_applicable{false};
};

struct MeasurementGateParamConfig
{
  double score_threshold{2.0};
  bool reject_above_score_threshold{true};

  bool enable_consistency_recovery_gate{false};
  int consistency_recovery_min_rejections{10};
  double consistency_recovery_score_margin{2.0};
  double consistency_recovery_max_translation_m{0.05};
  double consistency_recovery_max_yaw_deg{0.5};

  bool enable_post_reject_strict_score_threshold{false};
  int post_reject_strict_min_rejections{100};
  double post_reject_strict_score_threshold{5.5};

  bool enable_open_loop_strict_score_threshold{false};
  double open_loop_strict_min_accepted_gap_sec{15.0};
  double open_loop_strict_min_seed_translation_m{100.0};
  double open_loop_strict_score_threshold{5.25};

  bool enable_borderline_seed_rejection_gate{false};
  double borderline_seed_gate_score_threshold{5.25};
  double borderline_seed_gate_min_seed_translation_m{1.0};

  bool enable_odom_tf_prediction_correction_guard{false};
  double odom_tf_prediction_correction_guard_translation_m{2.0};
  double odom_tf_prediction_correction_guard_yaw_deg{30.0};
  bool enable_odom_tf_prediction_recovery_correction_guard{false};
  int odom_tf_prediction_recovery_min_rejections{30};
  double odom_tf_prediction_recovery_max_fitness{1.5};
  double odom_tf_prediction_recovery_guard_translation_m{5.0};
  double odom_tf_prediction_recovery_guard_yaw_deg{30.0};

  bool enable_rejected_seed_update{false};
  int rejected_seed_update_min_rejections{0};
  double rejected_seed_update_max_fitness{10.0};
  double rejected_seed_update_max_correction_translation_m{2.0};
  double rejected_seed_update_max_correction_yaw_deg{2.0};
};

struct EffectiveScoreThresholdDecision
{
  double effective_score_threshold{0.0};
  bool post_reject_strict_active{false};
  bool open_loop_strict_active{false};
};

struct MeasurementGateDecision
{
  std::uint8_t status_level{kMeasurementGateOk};
  std::string status_message{"ok"};
  bool reject_measurement{false};
  bool post_reject_strict_active{false};
  bool open_loop_strict_active{false};
  bool borderline_seed_gate_active{false};
  bool rejected_seed_update_applied{false};
  double effective_score_threshold{0.0};
};

inline MeasurementGateParams makeMeasurementGateParams(
  const MeasurementGateParamConfig & config)
{
  MeasurementGateParams params;
  params.score_threshold = config.score_threshold;
  params.reject_above_score_threshold = config.reject_above_score_threshold;
  params.enable_consistency_recovery_gate = config.enable_consistency_recovery_gate;
  params.consistency_recovery_min_rejections =
    config.consistency_recovery_min_rejections;
  params.consistency_recovery_score_margin = config.consistency_recovery_score_margin;
  params.consistency_recovery_max_translation_m =
    config.consistency_recovery_max_translation_m;
  params.consistency_recovery_max_yaw_deg =
    config.consistency_recovery_max_yaw_deg;
  params.enable_post_reject_strict_score_threshold =
    config.enable_post_reject_strict_score_threshold;
  params.post_reject_strict_min_rejections =
    config.post_reject_strict_min_rejections;
  params.post_reject_strict_score_threshold =
    config.post_reject_strict_score_threshold;
  params.enable_open_loop_strict_score_threshold =
    config.enable_open_loop_strict_score_threshold;
  params.open_loop_strict_min_accepted_gap_sec =
    config.open_loop_strict_min_accepted_gap_sec;
  params.open_loop_strict_min_seed_translation_m =
    config.open_loop_strict_min_seed_translation_m;
  params.open_loop_strict_score_threshold =
    config.open_loop_strict_score_threshold;
  params.enable_borderline_seed_rejection_gate =
    config.enable_borderline_seed_rejection_gate;
  params.borderline_seed_gate_score_threshold =
    config.borderline_seed_gate_score_threshold;
  params.borderline_seed_gate_min_seed_translation_m =
    config.borderline_seed_gate_min_seed_translation_m;
  params.enable_odom_tf_prediction_correction_guard =
    config.enable_odom_tf_prediction_correction_guard;
  params.odom_tf_prediction_correction_guard_translation_m =
    config.odom_tf_prediction_correction_guard_translation_m;
  params.odom_tf_prediction_correction_guard_yaw_deg =
    config.odom_tf_prediction_correction_guard_yaw_deg;
  params.enable_odom_tf_prediction_recovery_correction_guard =
    config.enable_odom_tf_prediction_recovery_correction_guard;
  params.odom_tf_prediction_recovery_min_rejections =
    config.odom_tf_prediction_recovery_min_rejections;
  params.odom_tf_prediction_recovery_max_fitness =
    config.odom_tf_prediction_recovery_max_fitness;
  params.odom_tf_prediction_recovery_guard_translation_m =
    config.odom_tf_prediction_recovery_guard_translation_m;
  params.odom_tf_prediction_recovery_guard_yaw_deg =
    config.odom_tf_prediction_recovery_guard_yaw_deg;
  params.enable_rejected_seed_update = config.enable_rejected_seed_update;
  params.rejected_seed_update_min_rejections =
    config.rejected_seed_update_min_rejections;
  params.rejected_seed_update_max_fitness = config.rejected_seed_update_max_fitness;
  params.rejected_seed_update_max_correction_translation_m =
    config.rejected_seed_update_max_correction_translation_m;
  params.rejected_seed_update_max_correction_yaw_deg =
    config.rejected_seed_update_max_correction_yaw_deg;
  return params;
}

inline MeasurementGateInput makeMeasurementGateInput(
  double fitness_score,
  double accepted_gap_sec,
  double seed_translation_since_accept_m,
  double correction_translation_m,
  double correction_yaw_deg,
  std::size_t consecutive_rejected_updates,
  bool odom_tf_prediction_guard_applicable = false)
{
  return {
    fitness_score,
    accepted_gap_sec,
    seed_translation_since_accept_m,
    correction_translation_m,
    correction_yaw_deg,
    consecutive_rejected_updates,
    odom_tf_prediction_guard_applicable};
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
  decision.open_loop_strict_active =
    params.enable_open_loop_strict_score_threshold &&
    input.accepted_gap_sec >= params.open_loop_strict_min_accepted_gap_sec &&
    input.seed_translation_since_accept_m >= params.open_loop_strict_min_seed_translation_m &&
    params.open_loop_strict_score_threshold < params.score_threshold;

  decision.effective_score_threshold = params.score_threshold;
  if (
    decision.post_reject_strict_active &&
    params.post_reject_strict_score_threshold < decision.effective_score_threshold)
  {
    decision.effective_score_threshold = params.post_reject_strict_score_threshold;
  }
  if (
    decision.open_loop_strict_active &&
    params.open_loop_strict_score_threshold < decision.effective_score_threshold)
  {
    decision.effective_score_threshold = params.open_loop_strict_score_threshold;
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

inline MeasurementGateDecision evaluateMeasurementGate(
  const MeasurementGateParams & params,
  const MeasurementGateInput & input)
{
  MeasurementGateDecision gate;
  const auto threshold_decision = computeEffectiveScoreThreshold(params, input);
  gate.effective_score_threshold = threshold_decision.effective_score_threshold;
  gate.post_reject_strict_active = threshold_decision.post_reject_strict_active;
  gate.open_loop_strict_active = threshold_decision.open_loop_strict_active;
  gate.borderline_seed_gate_active =
    isBorderlineSeedGateActive(params, input, gate.effective_score_threshold);

  const bool odom_recovery_correction_window =
    params.enable_odom_tf_prediction_recovery_correction_guard &&
    static_cast<int>(input.consecutive_rejected_updates) >=
    params.odom_tf_prediction_recovery_min_rejections &&
    input.fitness_score <= params.odom_tf_prediction_recovery_max_fitness;
  const double odom_translation_limit = odom_recovery_correction_window ?
    params.odom_tf_prediction_recovery_guard_translation_m :
    params.odom_tf_prediction_correction_guard_translation_m;
  const double odom_yaw_limit = odom_recovery_correction_window ?
    params.odom_tf_prediction_recovery_guard_yaw_deg :
    params.odom_tf_prediction_correction_guard_yaw_deg;
  const bool odom_correction_guard_active =
    params.enable_odom_tf_prediction_correction_guard &&
    input.odom_tf_prediction_guard_applicable &&
    ((input.correction_translation_m > odom_translation_limit) ||
    (input.correction_yaw_deg > odom_yaw_limit));
  if (odom_correction_guard_active) {
    gate.status_level = kMeasurementGateWarn;
    gate.status_message = "odom_tf_prediction_correction_guard_rejected";
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
  } else if (
    gate.open_loop_strict_active &&
    gate.effective_score_threshold == params.open_loop_strict_score_threshold)
  {
    gate.status_message = params.reject_above_score_threshold ?
      "fitness_score_over_open_loop_strict_threshold_rejected" :
      "fitness_score_over_open_loop_strict_threshold";
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

  const bool recovery_candidate =
    params.enable_consistency_recovery_gate &&
    gate.reject_measurement &&
    static_cast<int>(input.consecutive_rejected_updates) >=
    params.consistency_recovery_min_rejections &&
    input.fitness_score <= params.score_threshold + params.consistency_recovery_score_margin &&
    input.correction_translation_m <= params.consistency_recovery_max_translation_m &&
    input.correction_yaw_deg <= params.consistency_recovery_max_yaw_deg;
  if (recovery_candidate) {
    gate.reject_measurement = false;
    gate.status_message = "fitness_score_over_threshold_consistency_recovered";
  }

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
