// ScanTracker's measurement gate over a sequence of scans: the agreement
// release of the seed correction guards (seed_correction_guard_agreement_release_*).
// Each scan is aligned from an odometry seed (the tracked pose moved by the
// odometry, which is right here unless a test says otherwise); the
// registration's answer is given, and the scan is accepted or rejected as the
// node's raw registration backend does.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

#include <Eigen/Core>

#include "lidar_localization/scan_tracker.hpp"

namespace ll = lidar_localization;

namespace
{

Eigen::Matrix4f pose(double x, double y, double yaw = 0.0, double z = 0.0)
{
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m(0, 0) = static_cast<float>(std::cos(yaw));
  m(0, 1) = static_cast<float>(-std::sin(yaw));
  m(1, 0) = static_cast<float>(std::sin(yaw));
  m(1, 1) = static_cast<float>(std::cos(yaw));
  m(0, 3) = static_cast<float>(x);
  m(1, 3) = static_cast<float>(y);
  m(2, 3) = static_cast<float>(z);
  return m;
}

ll::MeasurementGateParamConfig gateConfig(int agreement_release_scans)
{
  ll::MeasurementGateParamConfig config;
  config.score_threshold = 2.0;
  config.enable_seed_correction_guard = true;
  config.seed_correction_guard_translation_m = 0.3;
  config.seed_correction_guard_release_rejections = 30;
  config.seed_correction_guard_warmup_accepts = 5;
  config.seed_correction_guard_agreement_release_scans = agreement_release_scans;
  config.seed_correction_guard_agreement_release_translation_m = 0.1;
  config.seed_correction_guard_agreement_release_yaw_deg = 1.0;
  config.seed_correction_guard_agreement_release_window_sec = 1.0;
  config.seed_correction_guard_agreement_release_drift_sec = 10.0;
  config.seed_correction_guard_agreement_release_drift_tolerance_m = 0.2;
  return config;
}

struct Step
{
  bool accepted;
  std::string status;
};

// One scan: aligned from seed, the registration answers result.
Step scan(
  ll::ScanTracker & tracker, const ll::MeasurementGateParamConfig & config, double stamp_sec,
  const Eigen::Matrix4f & seed, const Eigen::Matrix4f & result, double fitness = 0.05)
{
  ll::AlignmentAttempt attempt;
  attempt.target_ready = true;
  attempt.has_converged = true;
  attempt.fitness_score = fitness;
  attempt.init_guess = seed;
  attempt.final_transformation = result;
  const auto correction = ll::computeAlignmentCorrectionMetrics(seed, result);
  attempt.correction_translation_m = correction.translation_m;
  attempt.correction_yaw_deg = correction.yaw_deg;
  tracker.gate_scan_stamp_sec = stamp_sec;
  tracker.fillSeedMetrics(attempt, stamp_sec);
  const auto gate = tracker.evaluateGate(attempt, config);
  tracker.recordReleaseHistory(attempt, gate, config);
  if (gate.reject_measurement) {
    tracker.rejectMeasurement(seed, stamp_sec);
  } else {
    tracker.acceptMeasurement(result, stamp_sec, config);
  }
  return {!gate.reject_measurement, gate.status_message};
}

// The scenario's clock and position: the truth is (x, 0); offset is how far
// the tracked pose is from it along y.
struct Track
{
  double stamp{0.0};
  double x{0.0};
  double offset{0.0};

  void advance(double dt = 0.1, double dx = 0.1)
  {
    stamp += dt;
    x += dx;
  }
  Eigen::Matrix4f seed() const {return pose(x, offset);}
  Eigen::Matrix4f truth() const {return pose(x, 0.0);}
};

// Tracks 10 scans along x with right seeds and answers: the guards are armed.
Track track(ll::ScanTracker & tracker, const ll::MeasurementGateParamConfig & config)
{
  Track t;
  tracker.reset(t.truth(), t.stamp);
  for (int k = 1; k <= 10; ++k) {
    t.advance();
    const auto step = scan(tracker, config, t.stamp, t.seed(), t.truth());
    assert(step.accepted && step.status == "ok");
  }
  assert(tracker.settled_accepts_since_reset >= 5);
  return t;
}

// Four degraded scans each register 0.15 m off along y, within the guard: the
// accepted pose slides 0.6 m away from the odometry.
void slide(ll::ScanTracker & tracker, const ll::MeasurementGateParamConfig & config, Track & t)
{
  for (int k = 1; k <= 4; ++k) {
    t.advance();
    const auto step = scan(tracker, config, t.stamp, t.seed(), pose(t.x, t.offset + 0.15));
    assert(step.accepted && step.status == "ok");
    t.offset += 0.15;
  }
}

// After the slide the registration answers the truth again, 0.6 m back. Off,
// the guard rejects that until its release; with 4 scans, the fourth agreeing
// answer is accepted.
void test_agreeing_corrections_back_to_the_odometry_are_released()
{
  for (const int scans : {0, 4}) {
    const auto config = gateConfig(scans);
    ll::ScanTracker tracker;
    Track t = track(tracker, config);
    slide(tracker, config, t);
    const std::size_t settled = tracker.settled_accepts_since_reset;
    int released_at = 0;
    for (int k = 1; k <= 8; ++k) {
      t.advance();
      const auto step = scan(tracker, config, t.stamp, t.seed(), t.truth());
      if (step.accepted) {
        assert(step.status == "seed_correction_guard_agreement_released");
        released_at = k;
        break;
      }
      assert(step.status == "seed_correction_guard_rejected");
    }
    if (scans == 0) {
      assert(released_at == 0);
      assert(tracker.prediction.consecutive_rejected_updates == 8);
      continue;
    }
    assert(released_at == 4);
    t.offset = 0.0;
    // The pose is on the truth; the guards stay armed (no new warmup) and the
    // rejected corrections are forgotten.
    assert(std::abs(tracker.prediction.last_accepted_pose_matrix(1, 3)) < 1e-5);
    assert(tracker.settled_accepts_since_reset == settled);
    assert(tracker.seed_correction_agreement_history.size() == 0);
    // Tracking goes on from there, and a jump is rejected again.
    t.advance();
    auto step = scan(tracker, config, t.stamp, t.seed(), pose(t.x, 0.02));
    assert(step.accepted && step.status == "ok");
    t.advance();
    step = scan(tracker, config, t.stamp, t.seed(), pose(t.x + 1.5, 0.0));
    assert(!step.accepted && step.status == "seed_correction_guard_rejected");
  }
}

// The odometry jumped 0.6 m while the pose was right (or, alike, the
// registration keeps answering a place 0.6 m away): those corrections agree,
// but they move the pose away from the odometry, so they are held.
void test_corrections_away_from_the_odometry_are_held()
{
  const auto config = gateConfig(4);
  ll::ScanTracker tracker;
  Track t = track(tracker, config);
  t.offset = 0.6;
  for (int k = 1; k <= 8; ++k) {
    t.advance();
    const auto step = scan(tracker, config, t.stamp, t.seed(), t.truth());
    assert(!step.accepted && step.status == "seed_correction_guard_rejected");
  }
  // With a looser drift tolerance it is released.
  auto loose = config;
  loose.seed_correction_guard_agreement_release_drift_tolerance_m = 1.0;
  t.advance();
  const auto step = scan(tracker, loose, t.stamp, t.seed(), t.truth());
  assert(step.accepted && step.status == "seed_correction_guard_agreement_released");
}

// The accepted drift counts only within the drift horizon.
void test_old_drift_does_not_count()
{
  auto config = gateConfig(4);
  config.seed_correction_guard_agreement_release_drift_sec = 1.0;
  ll::ScanTracker tracker;
  Track t = track(tracker, config);
  slide(tracker, config, t);
  // 1 s later, the slide is beyond the horizon.
  for (int k = 1; k <= 8; ++k) {
    t.advance(0.2, 0.2);
    const auto step = scan(tracker, config, t.stamp, t.seed(), t.truth());
    assert(!step.accepted);
  }
}

// An alias flickering between two places never agrees four times in a row.
void test_disagreeing_answers_are_held()
{
  const auto config = gateConfig(4);
  ll::ScanTracker tracker;
  Track t = track(tracker, config);
  slide(tracker, config, t);
  for (int k = 1; k <= 12; ++k) {
    t.advance();
    // The truth twice, then a place 1.5 m along the room, in turn.
    const auto answer = (k % 3 == 0) ? pose(t.x + 1.5, t.offset) : t.truth();
    const auto step = scan(tracker, config, t.stamp, t.seed(), answer);
    assert(!step.accepted && step.status == "seed_correction_guard_rejected");
  }
}

// Turned answers, and answers over the fitness threshold, do not count.
void test_turned_and_poorly_fitting_answers_are_held()
{
  {
    const auto config = gateConfig(4);
    ll::ScanTracker tracker;
    Track t = track(tracker, config);
    slide(tracker, config, t);
    for (int k = 1; k <= 8; ++k) {
      t.advance();
      // The truth, its yaw 2 degrees apart scan to scan.
      const double yaw = (k % 2 ? 1.0 : -1.0) * M_PI / 180.0;
      const auto step = scan(tracker, config, t.stamp, t.seed(), pose(t.x, 0.0, yaw));
      assert(!step.accepted);
    }
  }
  {
    const auto config = gateConfig(4);
    ll::ScanTracker tracker;
    Track t = track(tracker, config);
    slide(tracker, config, t);
    for (int k = 1; k <= 3; ++k) {
      t.advance();
      const auto step = scan(tracker, config, t.stamp, t.seed(), t.truth(), 3.0);
      assert(!step.accepted && step.status == "seed_correction_guard_rejected");
    }
    assert(tracker.seed_correction_agreement_history.size() == 0);
    // The fourth fits well but has nothing to agree with yet.
    t.advance();
    assert(!scan(tracker, config, t.stamp, t.seed(), t.truth()).accepted);
  }
}

// A correction the per-scan guard rejects for its height alone is not released:
// the agreement and the drift are horizontal.
void test_vertical_corrections_are_held()
{
  const auto config = gateConfig(4);
  ll::ScanTracker tracker;
  Track t = track(tracker, config);
  for (int k = 1; k <= 8; ++k) {
    t.advance();
    const auto step = scan(tracker, config, t.stamp, t.seed(), pose(t.x, 0.0, 0.0, 0.5));
    assert(!step.accepted && step.status == "seed_correction_guard_rejected");
  }
}

// Rejected corrections older than the agreement window do not count.
void test_old_rejections_expire()
{
  const auto config = gateConfig(4);
  ll::ScanTracker tracker;
  Track t = track(tracker, config);
  slide(tracker, config, t);
  for (int k = 1; k <= 8; ++k) {
    // One scan in 0.4 s: four span 1.2 s, beyond the 1.0 s window.
    t.advance(0.4, 0.4);
    const auto step = scan(tracker, config, t.stamp, t.seed(), t.truth());
    assert(!step.accepted);
  }
}

// An accepted scan near the offset pose in between moves the held corrections
// by its own: the next answers, now 0.35 m from their seeds, agree with them.
void test_accepted_scans_in_between_move_the_corrections()
{
  const auto config = gateConfig(4);
  ll::ScanTracker tracker;
  Track t = track(tracker, config);
  slide(tracker, config, t);
  for (int k = 1; k <= 2; ++k) {
    t.advance();
    assert(!scan(tracker, config, t.stamp, t.seed(), t.truth()).accepted);
  }
  // Accepted 0.25 m toward the truth: the pose is now 0.35 m off.
  t.advance();
  auto step = scan(tracker, config, t.stamp, t.seed(), pose(t.x, 0.35));
  assert(step.accepted && step.status == "ok");
  t.offset = 0.35;
  t.advance();
  assert(!scan(tracker, config, t.stamp, t.seed(), t.truth()).accepted);
  t.advance();
  step = scan(tracker, config, t.stamp, t.seed(), t.truth());
  assert(step.accepted && step.status == "seed_correction_guard_agreement_released");
}

// A reset forgets the corrections and the drift.
void test_reset_clears_the_corrections()
{
  const auto config = gateConfig(4);
  ll::ScanTracker tracker;
  Track t = track(tracker, config);
  slide(tracker, config, t);
  for (int k = 1; k <= 3; ++k) {
    t.advance();
    assert(!scan(tracker, config, t.stamp, t.seed(), t.truth()).accepted);
  }
  assert(tracker.seed_correction_agreement_history.size() == 3);
  assert(tracker.accepted_correction_history.size() > 0);
  tracker.reset(t.truth(), t.stamp);
  assert(tracker.seed_correction_agreement_history.size() == 0);
  assert(tracker.accepted_correction_history.size() == 0);
}

// Off, none of this state is kept.
void test_off_keeps_no_state()
{
  const auto config = gateConfig(0);
  ll::ScanTracker tracker;
  Track t = track(tracker, config);
  slide(tracker, config, t);
  for (int k = 1; k <= 3; ++k) {
    t.advance();
    assert(!scan(tracker, config, t.stamp, t.seed(), t.truth()).accepted);
  }
  assert(tracker.seed_correction_agreement_history.size() == 0);
  assert(tracker.accepted_correction_history.size() == 0);
}

}  // namespace

int main()
{
  test_agreeing_corrections_back_to_the_odometry_are_released();
  test_corrections_away_from_the_odometry_are_held();
  test_old_drift_does_not_count();
  test_disagreeing_answers_are_held();
  test_turned_and_poorly_fitting_answers_are_held();
  test_vertical_corrections_are_held();
  test_old_rejections_expire();
  test_accepted_scans_in_between_move_the_corrections();
  test_reset_clears_the_corrections();
  test_off_keeps_no_state();
  std::printf("scan tracker tests passed\n");
  return 0;
}
