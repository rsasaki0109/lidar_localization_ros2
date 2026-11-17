#ifndef LIDAR_LOCALIZATION_STALE_RESET_REPLAY_POLICY_HPP_
#define LIDAR_LOCALIZATION_STALE_RESET_REPLAY_POLICY_HPP_

// Bringing a stale reset up to the newest scan (opt-in).
//
// A global localization answer is computed on a scan seconds older than the one
// the localizer applies it to: on a moving vehicle the reset is metres behind. The
// localizer would restart there with no velocity, seed every later scan there and
// reject it. Instead it keeps the scans of the last max_age_sec, and when a reset is
// at least min_age_sec older than the scan it is applied to, it tracks from the
// reset through the kept scans from the reset's stamp on, each seeded from the one
// before as in normal tracking. A scan callback replays for about budget_sec and
// keeps its own scan for the replay, so the scans that arrive meanwhile are not
// lost; the replay ends when it has caught up with the newest scan. The reset then
// moves to that scan, with the velocity tracked on the way. A replay that loses
// track (too many rejections in a row, too few accepted, the last scan rejected)
// or falls max_age_sec behind changes nothing: the reset is applied as it is.

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace lidar_localization
{

struct StaleResetReplayParams
{
  bool enable{false};
  double min_age_sec{0.5};
  // How long scans are kept, and the oldest reset that is replayed.
  double max_age_sec{20.0};
  // The first kept scan must follow the reset this closely: it is seeded at the
  // reset pose without a velocity.
  double max_first_gap_sec{0.25};
  int max_consecutive_rejections{3};
  double min_accepted_ratio{0.9};
  // The first scans are all replayed, until a motion is tracked; after them one
  // scan per step_sec, which halves the replay at 10 Hz. Seeded with the tracked
  // motion, a 0.2 s step stays inside the registration's reach in a 20 deg/s turn
  // that tightens; a 0.5 s step there does not (Boreas, 2026-11-17).
  int warmup_scans{3};
  double step_sec{0.2};
  // Replay time per scan callback.
  double budget_sec{0.2};
};

struct StaleResetReplayPlan
{
  bool replay{false};
  std::string reason{"disabled"};
};

// first_kept_stamp_sec: the stamp of the first kept scan at or after the reset,
// NaN when none is kept.
inline StaleResetReplayPlan planStaleResetReplay(
  const StaleResetReplayParams & params,
  double reset_stamp_sec,
  double scan_stamp_sec,
  double first_kept_stamp_sec)
{
  StaleResetReplayPlan plan;
  if (!params.enable) {
    return plan;
  }
  const double age_sec = scan_stamp_sec - reset_stamp_sec;
  if (!std::isfinite(age_sec) || age_sec < params.min_age_sec) {
    plan.reason = "fresh";
    return plan;
  }
  if (age_sec > params.max_age_sec) {
    plan.reason = "too_old";
    return plan;
  }
  const double first_gap_sec = first_kept_stamp_sec - reset_stamp_sec;
  if (!std::isfinite(first_gap_sec) || first_gap_sec < 0.0 ||
    first_gap_sec > params.max_first_gap_sec)
  {
    plan.reason = "reset_scan_not_kept";
    return plan;
  }
  plan.replay = true;
  plan.reason = "replay";
  return plan;
}

// Scan stamps jitter by milliseconds: two scans 0.2 s apart can be 0.199 s apart.
constexpr double kStampToleranceSec = 0.01;

// The index of the next kept scan (stamps ascending) to replay after the one
// stamped last_replayed_sec (-inf before the first): during the first warmup_scans
// steps the next scan, then the first at least step_sec later, and the newest scan
// when none is. -1 when the replay has caught up.
inline int nextStaleResetReplayScan(
  const StaleResetReplayParams & params, const std::vector<double> & stamps_sec,
  double last_replayed_sec, int steps_done)
{
  int newest_later = -1;
  for (std::size_t i = 0; i < stamps_sec.size(); ++i) {
    if (!(stamps_sec[i] > last_replayed_sec)) {
      continue;
    }
    if (steps_done < params.warmup_scans ||
      stamps_sec[i] - last_replayed_sec >= params.step_sec - kStampToleranceSec)
    {
      return static_cast<int>(i);
    }
    newest_later = static_cast<int>(i);
  }
  return newest_later;
}

// A replay still this far behind the newest scan does not catch up in time.
inline bool staleResetReplayFallenBehind(
  const StaleResetReplayParams & params, double last_replayed_sec, double newest_scan_sec)
{
  return newest_scan_sec - last_replayed_sec > params.max_age_sec;
}

// A replayed scan counts like a normal one: converged, finite and under the score
// threshold (the inlier-ratio score when min_inlier_ratio is set).
inline bool staleResetReplayStepAccepted(
  bool converged, bool finite, double fitness_score, double score_threshold)
{
  return converged && finite && std::isfinite(fitness_score) &&
         fitness_score < score_threshold;
}

struct StaleResetReplayProgress
{
  int steps{0};
  int accepted{0};
  int consecutive_rejections{0};
  bool last_accepted{false};
};

inline StaleResetReplayProgress recordStaleResetReplayStep(
  StaleResetReplayProgress progress, bool accepted)
{
  ++progress.steps;
  progress.last_accepted = accepted;
  if (accepted) {
    ++progress.accepted;
    progress.consecutive_rejections = 0;
  } else {
    ++progress.consecutive_rejections;
  }
  return progress;
}

inline bool staleResetReplayLost(
  const StaleResetReplayParams & params, const StaleResetReplayProgress & progress)
{
  return progress.consecutive_rejections > params.max_consecutive_rejections;
}

// Empty when the replay brought the reset up to the last scan, else why not.
inline std::string staleResetReplayFailure(
  const StaleResetReplayParams & params, const StaleResetReplayProgress & progress)
{
  if (progress.steps <= 0) {
    return "no_scans";
  }
  if (staleResetReplayLost(params, progress)) {
    return "lost_track";
  }
  if (!progress.last_accepted) {
    return "last_scan_rejected";
  }
  if (static_cast<double>(progress.accepted) <
    params.min_accepted_ratio * static_cast<double>(progress.steps))
  {
    return "too_few_accepted";
  }
  return "";
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_STALE_RESET_REPLAY_POLICY_HPP_
