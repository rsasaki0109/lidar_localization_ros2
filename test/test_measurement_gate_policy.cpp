#include "lidar_localization/measurement_gate_policy.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <string>

namespace ll = lidar_localization;

ll::MeasurementGateParams default_params()
{
  ll::MeasurementGateParams params;
  params.score_threshold = 10.0;
  params.reject_above_score_threshold = true;
  return params;
}

ll::MeasurementGateInput default_input()
{
  return ll::makeMeasurementGateInput(1.0, 0.0, 0.0, 0.0, 0.0, 0);
}

void test_param_and_input_builders_map_fields()
{
  ll::MeasurementGateParamConfig config;
  config.score_threshold = 9.0;
  config.reject_above_score_threshold = false;
  config.enable_post_reject_strict_score_threshold = true;
  config.post_reject_strict_min_rejections = 5;
  config.post_reject_strict_score_threshold = 7.0;
  config.enable_borderline_seed_rejection_gate = true;
  config.borderline_seed_gate_score_threshold = 6.25;
  config.borderline_seed_gate_min_seed_translation_m = 1.25;
  config.enable_rejected_seed_update = true;
  config.rejected_seed_update_min_rejections = 8;
  config.rejected_seed_update_max_fitness = 10.5;
  config.rejected_seed_update_max_correction_translation_m = 1.75;
  config.rejected_seed_update_max_correction_yaw_deg = 2.25;

  const auto params = ll::makeMeasurementGateParams(config);
  assert(params.score_threshold == 9.0);
  assert(!params.reject_above_score_threshold);
  assert(params.enable_post_reject_strict_score_threshold);
  assert(params.post_reject_strict_min_rejections == 5);
  assert(params.post_reject_strict_score_threshold == 7.0);
  assert(params.enable_borderline_seed_rejection_gate);
  assert(params.borderline_seed_gate_score_threshold == 6.25);
  assert(params.borderline_seed_gate_min_seed_translation_m == 1.25);
  assert(params.enable_rejected_seed_update);
  assert(params.rejected_seed_update_min_rejections == 8);
  assert(params.rejected_seed_update_max_fitness == 10.5);
  assert(params.rejected_seed_update_max_correction_translation_m == 1.75);
  assert(params.rejected_seed_update_max_correction_yaw_deg == 2.25);

  const auto input = ll::makeMeasurementGateInput(1.0, 2.0, 3.0, 4.0, 5.0, 6);
  assert(input.fitness_score == 1.0);
  assert(input.accepted_gap_sec == 2.0);
  assert(input.seed_translation_since_accept_m == 3.0);
  assert(input.correction_translation_m == 4.0);
  assert(input.correction_yaw_deg == 5.0);
  assert(input.consecutive_rejected_updates == 6);
}

void test_seed_correction_guard_rejects_jumps_until_release()
{
  auto params = default_params();
  auto input = default_input();
  input.correction_translation_m = 2.5;
  auto gate = ll::evaluateMeasurementGate(params, input);
  assert(!gate.reject_measurement);  // disabled by default

  params.enable_seed_correction_guard = true;
  params.seed_correction_guard_translation_m = 0.5;
  params.seed_correction_guard_yaw_deg = 15.0;
  params.seed_correction_guard_release_rejections = 10;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.reject_measurement);
  assert(gate.status_message == "seed_correction_guard_rejected");

  input.correction_translation_m = 0.1;
  input.correction_yaw_deg = 20.0;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.reject_measurement);

  input.correction_translation_m = 0.5;
  input.correction_yaw_deg = 15.0;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(!gate.reject_measurement);

  // Not applied before the track is established.
  params.seed_correction_guard_warmup_accepts = 5;
  input.correction_translation_m = 2.5;
  input.accepted_updates_since_reset = 4;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(!gate.reject_measurement);
  input.accepted_updates_since_reset = 5;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.reject_measurement);

  // Released after a sustained rejection streak.
  input.correction_translation_m = 2.5;
  input.consecutive_rejected_updates = 10;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(!gate.reject_measurement);
}

void test_seed_correction_window_guard_bounds_summed_corrections()
{
  auto params = default_params();
  auto input = default_input();
  input.window_correction_translation_m = 0.8;
  auto gate = ll::evaluateMeasurementGate(params, input);
  assert(!gate.reject_measurement);  // disabled by default

  params.enable_seed_correction_window_guard = true;
  params.seed_correction_window_guard_translation_m = 0.3;
  params.seed_correction_guard_warmup_accepts = 5;
  params.seed_correction_guard_release_rejections = 30;
  input.accepted_updates_since_reset = 5;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.reject_measurement);
  assert(gate.status_level == ll::kMeasurementGateWarn);
  assert(gate.status_message == "seed_correction_window_guard_rejected");

  // Independent of the per-scan guard: each correction is small.
  input.correction_translation_m = 0.1;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.reject_measurement);

  input.window_correction_translation_m = 0.3;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(!gate.reject_measurement);

  // Shares the per-scan guard's warmup and release.
  input.window_correction_translation_m = 0.8;
  input.accepted_updates_since_reset = 4;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(!gate.reject_measurement);
  input.accepted_updates_since_reset = 5;
  input.consecutive_rejected_updates = 30;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(!gate.reject_measurement);
}

void test_seed_correction_window_sums_within_window()
{
  ll::SeedCorrectionWindow window;
  assert(window.sumNormWith(10.0, 0.1, 0.0, 1.0) == 0.1);
  window.add(10.0, 0.2, 0.0, 1.0);
  window.add(10.5, 0.2, 0.0, 1.0);
  // Same direction adds up; the current correction counts too.
  assert(std::abs(window.sumNormWith(10.6, 0.1, 0.0, 1.0) - 0.5) < 1e-9);
  // Opposite corrections cancel: a pull back toward the prediction passes.
  assert(std::abs(window.sumNormWith(10.6, -0.4, 0.0, 1.0)) < 1e-9);
  // Horizontal norm of the vector sum.
  assert(std::abs(window.sumNormWith(10.6, -0.4, 0.3, 1.0) - 0.3) < 1e-9);
  // Entries older than the window no longer count, and are dropped on add.
  assert(std::abs(window.sumNormWith(11.2, 0.0, 0.0, 1.0) - 0.2) < 1e-9);
  window.add(11.2, 0.0, 0.0, 1.0);
  assert(window.size() == 2);
  window.add(12.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0);
  assert(window.size() == 2);
  window.clear();
  assert(window.size() == 0);
  assert(window.sumNormWith(12.0, 0.0, 0.0, 1.0) == 0.0);
}

void test_seed_correction_release_history_agreement()
{
  ll::SeedCorrectionReleaseHistory history;
  // Off: always agrees.
  assert(history.agrees(2.4, 0.0, 0.0, 0, 0.3, 5.0));
  // Not enough answers yet.
  assert(!history.agrees(0.4, 0.0, 0.0, 3, 0.3, 5.0));
  history.add(0.40, 0.00, 1.0, 3);
  history.add(0.45, 0.05, 0.0, 3);
  assert(!history.agrees(0.4, 0.0, 0.0, 3, 0.3, 5.0));
  history.add(0.35, -0.05, -1.0, 3);
  assert(history.agrees(0.42, 0.0, 0.0, 3, 0.3, 5.0));
  // An aliased answer far from the recent ones does not agree.
  assert(!history.agrees(-2.4, 0.1, 0.0, 3, 0.3, 5.0));
  // Nor one turned beyond the yaw tolerance; yaw wraps.
  assert(!history.agrees(0.42, 0.0, 7.0, 3, 0.3, 5.0));
  history.clear();
  history.add(0.0, 0.0, 179.0, 2);
  history.add(0.0, 0.0, -179.0, 2);
  assert(history.agrees(0.0, 0.0, 180.0, 2, 0.3, 5.0));
  // Capacity keeps only the latest answers; one alias among them breaks agreement
  // until it has left.
  history.clear();
  history.add(-2.4, 0.0, 0.0, 2);
  history.add(0.4, 0.0, 0.0, 2);
  assert(!history.agrees(0.4, 0.0, 0.0, 2, 0.3, 5.0));
  history.add(0.4, 0.0, 0.0, 2);
  assert(history.size() == 2);
  assert(history.agrees(0.4, 0.0, 0.0, 2, 0.3, 5.0));
  // Non-finite answers are not kept and never agree.
  history.add(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 2);
  assert(history.size() == 2);
  assert(!history.agrees(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 2, 0.3, 5.0));
  history.add(0.4, 0.0, 0.0, 0);
  assert(history.size() == 2);

  assert(ll::isSeedCorrectionGuardRejection("seed_correction_guard_rejected"));
  assert(ll::isSeedCorrectionGuardRejection("seed_correction_window_guard_rejected"));
  assert(ll::isSeedCorrectionGuardRejection("seed_correction_guard_release_disagreed"));
  assert(!ll::isSeedCorrectionGuardRejection("fitness_score_over_threshold_rejected"));
}

void test_seed_correction_release_agreement_holds_disagreeing_releases()
{
  auto params = default_params();
  params.enable_seed_correction_guard = true;
  params.seed_correction_guard_translation_m = 0.3;
  params.seed_correction_guard_release_rejections = 30;
  params.seed_correction_guard_warmup_accepts = 5;
  auto input = default_input();
  input.accepted_updates_since_reset = 5;
  input.correction_translation_m = 2.4;
  input.consecutive_rejected_updates = 30;
  input.release_agreed = false;

  // Off by default: the release accepts whatever the registration returns.
  auto gate = ll::evaluateMeasurementGate(params, input);
  assert(!gate.reject_measurement);

  params.seed_correction_guard_release_agreement_scans = 10;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.reject_measurement);
  assert(gate.status_level == ll::kMeasurementGateWarn);
  assert(gate.status_message == "seed_correction_guard_release_disagreed");
  // Held for as long as it disagrees.
  input.consecutive_rejected_updates = 80;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.reject_measurement);

  // An answer that agrees with the recent rejected ones is released.
  input.release_agreed = true;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(!gate.reject_measurement);
  assert(gate.status_message == "ok");

  // Before the release the guards reject as before, agreement or not.
  input.consecutive_rejected_updates = 29;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.status_message == "seed_correction_guard_rejected");

  // A released answer within the guards' limits needs no agreement.
  input.consecutive_rejected_updates = 30;
  input.release_agreed = false;
  input.correction_translation_m = 0.2;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(!gate.reject_measurement);

  // Applies to the window guard's release too.
  params.enable_seed_correction_window_guard = true;
  params.seed_correction_window_guard_translation_m = 0.3;
  input.window_correction_translation_m = 0.8;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.status_message == "seed_correction_guard_release_disagreed");
  input.release_agreed = true;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(!gate.reject_measurement);

  // Not applied before the guards' warmup.
  input.release_agreed = false;
  input.accepted_updates_since_reset = 4;
  input.correction_translation_m = 2.4;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(!gate.reject_measurement);
}

void test_default_ok_gate()
{
  const auto gate = ll::evaluateMeasurementGate(default_params(), default_input());
  assert(gate.status_level == ll::kMeasurementGateOk);
  assert(gate.status_message == "ok");
  assert(!gate.reject_measurement);
  assert(gate.effective_score_threshold == 10.0);


}

void test_post_reject_strict_threshold_lowers_the_effective_threshold()
{
  auto params = default_params();
  params.enable_post_reject_strict_score_threshold = true;
  params.post_reject_strict_min_rejections = 3;
  params.post_reject_strict_score_threshold = 8.0;

  auto input = default_input();
  input.consecutive_rejected_updates = 2;
  assert(!ll::computeEffectiveScoreThreshold(params, input).post_reject_strict_active);

  input.consecutive_rejected_updates = 3;
  const auto threshold = ll::computeEffectiveScoreThreshold(params, input);
  assert(threshold.post_reject_strict_active);
  assert(threshold.effective_score_threshold == 8.0);
}

void test_threshold_rejection_messages()
{
  auto params = default_params();
  auto input = default_input();
  input.fitness_score = 11.0;

  auto gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.status_level == ll::kMeasurementGateWarn);
  assert(gate.reject_measurement);
  assert(gate.status_message == "fitness_score_over_threshold_rejected");

  params.reject_above_score_threshold = false;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.status_level == ll::kMeasurementGateWarn);
  assert(!gate.reject_measurement);
  assert(gate.status_message == "fitness_score_over_threshold");
}

void test_post_reject_message()
{
  auto params = default_params();
  params.enable_post_reject_strict_score_threshold = true;
  params.post_reject_strict_min_rejections = 2;
  params.post_reject_strict_score_threshold = 5.0;

  auto input = default_input();
  input.fitness_score = 7.0;
  input.consecutive_rejected_updates = 2;
  const auto gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.post_reject_strict_active);
  assert(gate.status_message == "fitness_score_over_post_reject_strict_threshold_rejected");
}

void test_borderline_seed_gate()
{
  auto params = default_params();
  params.enable_borderline_seed_rejection_gate = true;
  params.borderline_seed_gate_score_threshold = 8.0;
  params.borderline_seed_gate_min_seed_translation_m = 1.0;

  auto input = default_input();
  input.fitness_score = 9.0;
  input.seed_translation_since_accept_m = 1.0;

  const auto gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.borderline_seed_gate_active);
  assert(gate.reject_measurement);
  assert(gate.status_message == "fitness_score_over_borderline_seed_gate_rejected");
}

void test_rejected_seed_update_marks_rejection_for_prediction_update()
{
  auto params = default_params();
  params.enable_rejected_seed_update = true;
  params.rejected_seed_update_min_rejections = 2;
  params.rejected_seed_update_max_fitness = 12.0;
  params.rejected_seed_update_max_correction_translation_m = 1.0;
  params.rejected_seed_update_max_correction_yaw_deg = 2.0;

  auto input = default_input();
  input.fitness_score = 11.0;
  input.consecutive_rejected_updates = 2;
  input.correction_translation_m = 0.5;
  input.correction_yaw_deg = 1.0;

  const auto gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.reject_measurement);
  assert(gate.rejected_seed_update_applied);
  assert(gate.status_message == "fitness_score_over_threshold_rejected_seeded");
}

void test_nonfinite_fitness_is_never_accepted_or_seeded()
{
  for (const double score : {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()})
  {
    for (const bool reject_above : {false, true}) {
      auto params = default_params();
      params.reject_above_score_threshold = reject_above;
      params.enable_rejected_seed_update = true;
      auto input = default_input();
      input.fitness_score = score;
      input.consecutive_rejected_updates = 100;
      const auto gate = ll::evaluateMeasurementGate(params, input);
      assert(gate.reject_measurement);
      assert(!gate.rejected_seed_update_applied);
      assert(gate.status_level == ll::kMeasurementGateWarn);
      assert(gate.status_message == "registration_fitness_non_finite");
      assert(gate.effective_score_threshold == params.score_threshold);
    }
  }
}

void test_seed_guard_warmup_counts_only_settled_accepts()
{
  ll::MeasurementGateParams params;
  params.seed_correction_guard_translation_m = 0.3;
  params.seed_correction_guard_yaw_deg = 15.0;

  // A global-search pose 2.6 m off converges over a few large corrections; none
  // of them counts towards the warmup.
  std::size_t settled = 0;
  for (double correction : {2.6, 1.4, 0.7}) {
    settled = ll::nextSettledAccepts(settled, params, correction, 1.0);
    assert(settled == 0);
  }
  for (int i = 1; i <= 5; ++i) {
    settled = ll::nextSettledAccepts(settled, params, 0.05, 0.5);
    assert(settled == static_cast<std::size_t>(i));
  }
  // A large accepted correction (e.g. after the guard released) restarts it.
  assert(ll::nextSettledAccepts(settled, params, 0.05, 20.0) == 0);
  assert(ll::nextSettledAccepts(settled, params, 1.2, 0.0) == 0);
}

void test_seed_guard_exact_start_completes_warmup()
{
  ll::MeasurementGateParams params;
  params.seed_correction_guard_translation_m = 0.3;
  params.seed_correction_guard_yaw_deg = 15.0;
  params.seed_correction_guard_warmup_accepts = 5;

  // Off by default: the first accept counts once.
  assert(ll::nextSettledAccepts(0, params, 0.05, 0.5, true) == 1);

  params.seed_correction_guard_exact_start_translation_m = 0.1;
  // An initial pose that needed no pulling in completes the warmup.
  assert(ll::nextSettledAccepts(0, params, 0.05, 0.5, true) == 5);
  // Only for the first accept after the initial pose.
  assert(ll::nextSettledAccepts(0, params, 0.05, 0.5, false) == 1);
  // A coarse pose being pulled in keeps the normal warmup.
  assert(ll::nextSettledAccepts(0, params, 0.25, 0.5, true) == 1);
  assert(ll::nextSettledAccepts(0, params, 1.2, 0.5, true) == 0);
  assert(ll::nextSettledAccepts(0, params, 0.05, 20.0, true) == 0);
}

void test_seed_guard_trusted_start()
{
  ll::MeasurementGateParams params;
  params.enable_seed_correction_guard = true;
  params.seed_correction_guard_translation_m = 0.3;
  params.seed_correction_guard_yaw_deg = 15.0;
  params.seed_correction_guard_warmup_accepts = 5;
  params.seed_correction_guard_release_rejections = 30;

  // Off by default.
  assert(!ll::isTrustedInitialPose(params, 0.01, 0.01));

  params.seed_correction_guard_trusted_start_position_std_m = 0.1;
  assert(ll::isTrustedInitialPose(params, 0.1 * 0.1, 0.1 * 0.1));
  assert(ll::isTrustedInitialPose(params, 0.0025, 0.0049));
  // RViz's 2D Pose Estimate (0.25 m^2), a looser fix, or one axis loose: not trusted.
  assert(!ll::isTrustedInitialPose(params, 0.25, 0.25));
  assert(!ll::isTrustedInitialPose(params, 0.0025, 0.04));
  // A zero or invalid covariance says nothing.
  assert(!ll::isTrustedInitialPose(params, 0.0, 0.0));
  assert(!ll::isTrustedInitialPose(params, -0.01, 0.01));
  assert(!ll::isTrustedInitialPose(
      params, std::numeric_limits<double>::quiet_NaN(), 0.01));

  // The first odometry-seeded scan completes the warmup; without odometry it does not.
  assert(ll::trustedStartSettledAccepts(0, params, true) == 5);
  assert(ll::trustedStartSettledAccepts(0, params, false) == 0);
  assert(ll::trustedStartSettledAccepts(7, params, true) == 7);

  // So the first scan's 0.6 m correction is rejected instead of accepted.
  auto input = ll::makeMeasurementGateInput(0.13, 0.08, 0.14, 0.6, 2.0, 0, 0);
  assert(!ll::evaluateMeasurementGate(params, input).reject_measurement);
  input.accepted_updates_since_reset = ll::trustedStartSettledAccepts(0, params, true);
  const auto gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.reject_measurement);
  assert(gate.status_message == "seed_correction_guard_rejected");
}

void test_odom_seed_gap_detection()
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  assert(!ll::isOdomSeedGap(nan, 100.0));  // first odometry seed
  assert(!ll::isOdomSeedGap(100.0, 100.1));
  assert(!ll::isOdomSeedGap(100.0, 101.0));
  assert(ll::isOdomSeedGap(100.0, 101.2));  // e.g. a covered sensor
}

int main()
{
  test_seed_guard_warmup_counts_only_settled_accepts();
  test_seed_guard_exact_start_completes_warmup();
  test_seed_guard_trusted_start();
  test_odom_seed_gap_detection();
  test_seed_correction_guard_rejects_jumps_until_release();
  test_seed_correction_window_guard_bounds_summed_corrections();
  test_seed_correction_window_sums_within_window();
  test_seed_correction_release_history_agreement();
  test_seed_correction_release_agreement_holds_disagreeing_releases();
  test_param_and_input_builders_map_fields();
  test_default_ok_gate();
  test_nonfinite_fitness_is_never_accepted_or_seeded();
  test_post_reject_strict_threshold_lowers_the_effective_threshold();
  test_threshold_rejection_messages();
  test_post_reject_message();
  test_borderline_seed_gate();
  test_rejected_seed_update_marks_rejection_for_prediction_update();
  return 0;
}
