#include "lidar_localization/causal_twist_history.hpp"
#include <cassert>
#include <limits>
#include "lidar_localization/registration_seed_policy.hpp"
#include "lidar_localization/prediction_state_policy.hpp"

using lidar_localization::CausalTwistHistory;
using lidar_localization::TimestampedTwist;

TimestampedTwist sample(double stamp, double vx)
{
  TimestampedTwist value;
  value.stamp_sec = stamp;
  value.linear.x() = vx;
  return value;
}

int main()
{
  // Future-only velocity must select the existing fallback in both policies.
  CausalTwistHistory future_only;
  assert(future_only.insert(sample(10.2, -3)));
  lidar_localization::RegistrationSeedPolicyInput seed_input;
  seed_input.use_twist_prediction = true;
  seed_input.have_last_accepted_pose = true;
  seed_input.has_latest_twist = future_only.atOrBefore(10.1).has_value();
  assert(lidar_localization::chooseRegistrationSeed(seed_input).source ==
    lidar_localization::RegistrationSeedSource::kCurrentPose);
  assert(lidar_localization::choosePredictionAdvanceMode(
    true, true, seed_input.has_latest_twist, false) ==
    lidar_localization::PredictionAdvanceMode::kNone);
  assert(future_only.insert(sample(10.1, 1)));
  seed_input.has_latest_twist = future_only.atOrBefore(10.1).has_value();
  assert(lidar_localization::chooseRegistrationSeed(seed_input).source ==
    lidar_localization::RegistrationSeedSource::kTwistPrediction);
  assert(lidar_localization::choosePredictionAdvanceMode(
    true, true, seed_input.has_latest_twist, false) ==
    lidar_localization::PredictionAdvanceMode::kTwistPrediction);

  // A scan owns a copy even if callbacks replace or evict its history entry.
  CausalTwistHistory snapshot_history(1);
  assert(snapshot_history.insert(sample(10.0, 2)));
  const auto snapshot = snapshot_history.atOrBefore(10.1);
  assert(snapshot_history.insert(sample(10.0, 7)));
  assert(snapshot_history.insert(sample(11.0, 9)));
  snapshot_history.clear();
  assert(snapshot && snapshot->stamp_sec == 10.0 && snapshot->linear.x() == 2);

  CausalTwistHistory history(3);
  assert(!history.atOrBefore(10.1));
  assert(history.insert(sample(10.1, 1)));
  const double timely_x = history.atOrBefore(10.1)->linear.x() * .1;
  assert(history.insert(sample(10.2, -3)));
  assert(history.atOrBefore(10.1)->linear.x() * .1 == timely_x);
  assert(!history.atOrBefore(10.0)); // Future-only history is unavailable.
  assert(history.insert(sample(10.0, 2))); // Out-of-order delivery.
  assert(history.atOrBefore(10.05)->linear.x() == 2);
  assert(history.insert(sample(10.1, 4))); // Replacement, not extra capacity.
  assert(history.size() == 3 && history.atOrBefore(10.1)->linear.x() == 4);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  assert(!history.insert(sample(10.15, nan)));
  assert(!history.insert(sample(nan, 5)));
  assert(history.atOrBefore(10.15)->linear.x() == 4);
  assert(!history.atOrBefore(nan));
  assert(history.insert(sample(10.3, 6)));
  assert(history.size() == 3 && !history.atOrBefore(10.0));
  history.clear();
  assert(!history.atOrBefore(10.3));
  assert(history.insert(sample(0.0, 1))); // Explicit reset permits clock restart.
  assert(history.atOrBefore(0.0)->linear.x() == 1);
}
