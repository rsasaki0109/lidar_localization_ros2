#include "lidar_localization/soft_odom_correction_gate.hpp"
#include <cassert>
#include <cmath>
#include <limits>
namespace ll = lidar_localization;
ll::AlignmentPipelineResult attempt(const ll::MeasurementGateParams & p, const ll::MeasurementGateInput & in)
{
  ll::AlignmentPipelineResult r;
  r.selected_attempt.target_ready = true;
  r.selected_attempt.has_converged = true;
  r.selected_attempt.fitness_score = in.fitness_score;
  r.selected_attempt.correction_translation_m = in.correction_translation_m;
  r.selected_attempt.correction_yaw_deg = in.correction_yaw_deg;
  r.selected_attempt.final_transformation(0, 3) = in.correction_translation_m;
  r.gate_result = ll::evaluateMeasurementGate(p, in);
  return r;
}
int main()
{
  ll::MeasurementGateParams p;
  p.enable_odom_tf_prediction_correction_guard = true;
  p.odom_tf_prediction_correction_guard_translation_m = .3;
  p.odom_tf_prediction_correction_guard_yaw_deg = 5;
  ll::MeasurementGateInput in;
  in.fitness_score = .1;
  in.correction_translation_m = .6;
  in.odom_tf_prediction_guard_applicable = true;
  auto r = attempt(p, in);
  ll::applySoftOdomCorrectionGate(r, p, in, false);
  assert(r.gate_result.reject_measurement);
  ll::applySoftOdomCorrectionGate(r, p, in, true);
  assert(!r.gate_result.reject_measurement);
  assert(std::abs(r.selected_attempt.final_transformation(0, 3) - .3) < 1e-6);
  assert(std::abs(r.selected_attempt.correction_translation_m - .3) < 1e-6);
  in.fitness_score = 100;
  r = attempt(p, in);
  ll::applySoftOdomCorrectionGate(r, p, in, true);
  assert(r.gate_result.reject_measurement);
  assert(r.selected_attempt.final_transformation(0, 3) == .6f);
  // Preserve independent seed, strict and borderline guards, not only score.
  in.fitness_score = .1;
  p.enable_seed_correction_guard = true;
  p.seed_correction_guard_translation_m = .5;
  r = attempt(p, in);
  ll::applySoftOdomCorrectionGate(r, p, in, true);
  assert(r.gate_result.reject_measurement);
  p.enable_seed_correction_guard = false;
  p.enable_post_reject_strict_score_threshold = true;
  p.post_reject_strict_min_rejections = 0;
  p.post_reject_strict_score_threshold = .05;
  r = attempt(p, in);
  ll::applySoftOdomCorrectionGate(r, p, in, true);
  assert(r.gate_result.reject_measurement);
  p.enable_post_reject_strict_score_threshold = false;
  p.enable_borderline_seed_rejection_gate = true;
  p.borderline_seed_gate_score_threshold = .05;
  in.seed_translation_since_accept_m = 2;
  r = attempt(p, in);
  ll::applySoftOdomCorrectionGate(r, p, in, true);
  assert(r.gate_result.reject_measurement);
  p.enable_borderline_seed_rejection_gate = false;
  in.correction_translation_m = 15;
  r = attempt(p, in);
  ll::applySoftOdomCorrectionGate(r, p, in, true);
  assert(r.gate_result.reject_measurement);
  // A non-finite fitness and disabled/invalid odom thresholds remain rejected.
  in.correction_translation_m = .6;
  in.fitness_score = std::numeric_limits<double>::quiet_NaN();
  r = attempt(p, in);
  ll::applySoftOdomCorrectionGate(r, p, in, true);
  assert(r.gate_result.reject_measurement);
  in.fitness_score = .1;
  p.odom_tf_prediction_correction_guard_translation_m = 0;
  r = attempt(p, in);
  ll::applySoftOdomCorrectionGate(r, p, in, true);
  assert(r.gate_result.reject_measurement);
  p.odom_tf_prediction_correction_guard_translation_m = .3;
  in.correction_translation_m = 0;
  in.correction_yaw_deg = 10;
  r = attempt(p, in);
  r.selected_attempt.final_transformation.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(static_cast<float>(10 * M_PI / 180), Eigen::Vector3f::UnitZ()).toRotationMatrix();
  ll::applySoftOdomCorrectionGate(r, p, in, true);
  assert(!r.gate_result.reject_measurement);
  assert(std::abs(r.selected_attempt.correction_yaw_deg - 5) < 1e-5);

}
