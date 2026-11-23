#ifndef LIDAR_LOCALIZATION_MEASUREMENT_GATE_POLICY_HPP_
#define LIDAR_LOCALIZATION_MEASUREMENT_GATE_POLICY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <cstdint>
#include <deque>
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
  // An initial pose whose covariance claims a horizontal standard deviation of
  // at most this (and above 0) is trusted, e.g. a global localization fix the
  // startup has already registered and confirmed: when the first scan after it
  // is seeded from odometry, the guards hold from that scan instead of waiting
  // for the warmup. Right after a trusted pose the odometry is better than a
  // registration of a degraded scan (a legged robot stepping down pitches its
  // scan by 30-45 degrees). 0 disables this. A pose that is in fact off is
  // corrected only at the guards' release.
  double seed_correction_guard_trusted_start_position_std_m{0.0};

  // Windowed seed correction guard.  The guard above bounds one correction;
  // a run of degraded scans (occluded, or seeing things the map lacks) can
  // each pull the pose a little the same way, along a weakly constrained
  // axis, until the track has slid metres away from a seed that was right.
  // This guard bounds the horizontal map-frame sum of the corrections
  // accepted within window_sec plus the current one, i.e. how far the
  // registration moved the pose away from the prediction over the window.
  // It shares the guard's warmup and release.
  bool enable_seed_correction_window_guard{false};
  double seed_correction_window_guard_sec{1.0};
  double seed_correction_window_guard_translation_m{0.3};

  // Release agreement.  At release the guards accept whatever the registration
  // returns on that scan; if the scene briefly aliases then (another place in
  // the map that fits as well), the release locks onto it.  With
  // release_agreement_scans > 0 a released answer is accepted only when it
  // agrees with each of the last release_agreement_scans answers the guards
  // rejected (map-frame correction within release_agreement_translation_m
  // horizontally and release_agreement_yaw_deg), i.e. the registration has kept
  // pointing at the same pose; otherwise the guards keep holding the odometry
  // and report seed_correction_guard_release_disagreed.  0 releases as before.
  int seed_correction_guard_release_agreement_scans{0};
  double seed_correction_guard_release_agreement_translation_m{0.3};
  double seed_correction_guard_release_agreement_yaw_deg{5.0};

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
  // Horizontal norm of the summed map-frame corrections accepted within the
  // window plus this scan's (see SeedCorrectionWindow).
  double window_correction_translation_m{0.0};
  // Whether this scan's correction agrees with the recent guard-rejected ones
  // (see SeedCorrectionReleaseHistory); read only once the guards would release.
  bool release_agreed{true};
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

// Whether an initial pose with this position covariance (xx, yy, m^2) is
// trusted; see seed_correction_guard_trusted_start_position_std_m.
inline bool isTrustedInitialPose(
  const MeasurementGateParams & params, double covariance_xx, double covariance_yy)
{
  const double limit_m = params.seed_correction_guard_trusted_start_position_std_m;
  if (!(limit_m > 0.0) || !std::isfinite(limit_m)) {
    return false;
  }
  // A zero or invalid covariance says nothing about the pose.
  if (
    !std::isfinite(covariance_xx) || !std::isfinite(covariance_yy) ||
    covariance_xx <= 0.0 || covariance_yy <= 0.0)
  {
    return false;
  }
  return std::max(covariance_xx, covariance_yy) <= limit_m * limit_m;
}

// Settled-accept count for the first scan after a trusted initial pose: the
// warmup counts as done when that scan is seeded from odometry.
inline std::size_t trustedStartSettledAccepts(
  std::size_t settled_accepts,
  const MeasurementGateParams & params,
  bool odometry_seeded)
{
  if (!odometry_seeded) {
    return settled_accepts;
  }
  return std::max(
    settled_accepts,
    static_cast<std::size_t>(std::max(0, params.seed_correction_guard_warmup_accepts)));
}

// Map-frame horizontal corrections (result minus seed) of accepted updates,
// kept for seed_correction_window_guard_sec.
class SeedCorrectionWindow
{
public:
  void clear() {entries_.clear();}

  void add(double stamp_sec, double dx, double dy, double window_sec)
  {
    if (!std::isfinite(stamp_sec) || !std::isfinite(dx) || !std::isfinite(dy)) {
      return;
    }
    entries_.push_back({stamp_sec, dx, dy});
    expire(stamp_sec, window_sec);
  }

  // Norm of the corrections within window_sec before stamp_sec plus (dx, dy).
  double sumNormWith(double stamp_sec, double dx, double dy, double window_sec) const
  {
    double sum_x = dx;
    double sum_y = dy;
    for (const auto & entry : entries_) {
      if (entry.stamp_sec > stamp_sec - window_sec && entry.stamp_sec <= stamp_sec) {
        sum_x += entry.dx;
        sum_y += entry.dy;
      }
    }
    return std::hypot(sum_x, sum_y);
  }

  std::size_t size() const {return entries_.size();}

private:
  struct Entry
  {
    double stamp_sec;
    double dx;
    double dy;
  };

  void expire(double stamp_sec, double window_sec)
  {
    while (!entries_.empty() && entries_.front().stamp_sec <= stamp_sec - window_sec) {
      entries_.pop_front();
    }
  }

  std::deque<Entry> entries_;
};

// Map-frame corrections (result minus seed: dx, dy, signed yaw) of the most
// recent scans the seed correction guards rejected, for the release agreement.
class SeedCorrectionReleaseHistory
{
public:
  void clear() {entries_.clear();}

  void add(double dx, double dy, double dyaw_deg, std::size_t capacity)
  {
    if (capacity == 0 || !std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dyaw_deg)) {
      return;
    }
    entries_.push_back({dx, dy, dyaw_deg});
    while (entries_.size() > capacity) {
      entries_.pop_front();
    }
  }

  // True when at least `scans` corrections are held and the latest `scans` all
  // lie within translation_m (horizontal) and yaw_deg of (dx, dy, dyaw_deg).
  bool agrees(
    double dx, double dy, double dyaw_deg, std::size_t scans,
    double translation_m, double yaw_deg) const
  {
    if (scans == 0) {
      return true;
    }
    if (entries_.size() < scans || !std::isfinite(dx) || !std::isfinite(dy) ||
      !std::isfinite(dyaw_deg))
    {
      return false;
    }
    for (auto it = entries_.end() - static_cast<std::ptrdiff_t>(scans); it != entries_.end(); ++it) {
      const double yaw_diff =
        std::abs(std::remainder(it->dyaw_deg - dyaw_deg, 360.0));
      if (std::hypot(it->dx - dx, it->dy - dy) > translation_m || yaw_diff > yaw_deg) {
        return false;
      }
    }
    return true;
  }

  std::size_t size() const {return entries_.size();}

private:
  struct Entry
  {
    double dx;
    double dy;
    double dyaw_deg;
  };
  std::deque<Entry> entries_;
};

inline bool isSeedCorrectionGuardRejection(const std::string & status_message)
{
  return status_message == "seed_correction_guard_rejected" ||
         status_message == "seed_correction_window_guard_rejected" ||
         status_message == "seed_correction_guard_release_disagreed";
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

  const bool guard_warmed_up =
    input.accepted_updates_since_reset >=
    static_cast<std::size_t>(std::max(0, params.seed_correction_guard_warmup_accepts));
  const bool guard_released =
    static_cast<int>(input.consecutive_rejected_updates) >=
    params.seed_correction_guard_release_rejections;
  const bool per_scan_tripped =
    params.enable_seed_correction_guard && guard_warmed_up &&
    ((input.correction_translation_m > params.seed_correction_guard_translation_m) ||
    (input.correction_yaw_deg > params.seed_correction_guard_yaw_deg));
  const bool window_tripped =
    params.enable_seed_correction_window_guard && guard_warmed_up &&
    input.window_correction_translation_m > params.seed_correction_window_guard_translation_m;
  if (
    guard_released && (per_scan_tripped || window_tripped) &&
    params.seed_correction_guard_release_agreement_scans > 0 && !input.release_agreed)
  {
    gate.status_level = kMeasurementGateWarn;
    gate.status_message = "seed_correction_guard_release_disagreed";
    gate.reject_measurement = true;
    return gate;
  }

  if (per_scan_tripped && !guard_released) {
    gate.status_level = kMeasurementGateWarn;
    gate.status_message = "seed_correction_guard_rejected";
    gate.reject_measurement = true;
    return gate;
  }

  if (window_tripped && !guard_released) {
    gate.status_level = kMeasurementGateWarn;
    gate.status_message = "seed_correction_window_guard_rejected";
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
