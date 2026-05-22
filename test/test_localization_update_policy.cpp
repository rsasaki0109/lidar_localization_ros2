#include "lidar_localization/localization_update_policy.hpp"

#include <cassert>

namespace ll = lidar_localization;

ll::MeasurementGateDecision make_gate(bool reject_measurement, bool rejected_seed_update_applied)
{
  ll::MeasurementGateDecision gate;
  gate.reject_measurement = reject_measurement;
  gate.rejected_seed_update_applied = rejected_seed_update_applied;
  return gate;
}

void test_accept_measurement_updates_pose_and_prediction()
{
  const auto decision = ll::decideLocalizationUpdate(make_gate(false, false));

  assert(decision.action == ll::LocalizationUpdateAction::kAcceptMeasurement);
  assert(decision.update_current_pose);
  assert(decision.update_prediction_from_accepted_measurement);
  assert(!decision.advance_prediction_without_measurement);
  assert(!decision.update_prediction_from_rejected_measurement);
  assert(decision.continue_to_pose_publish);
}

void test_rejected_measurement_advances_prediction_without_pose_publish()
{
  const auto decision = ll::decideLocalizationUpdate(make_gate(true, false));

  assert(decision.action == ll::LocalizationUpdateAction::kAdvancePredictionWithoutMeasurement);
  assert(!decision.update_current_pose);
  assert(!decision.update_prediction_from_accepted_measurement);
  assert(decision.advance_prediction_without_measurement);
  assert(!decision.update_prediction_from_rejected_measurement);
  assert(!decision.continue_to_pose_publish);
}

void test_rejected_seed_update_keeps_pose_publish_closed()
{
  const auto decision = ll::decideLocalizationUpdate(make_gate(true, true));

  assert(decision.action == ll::LocalizationUpdateAction::kUpdatePredictionFromRejectedMeasurement);
  assert(!decision.update_current_pose);
  assert(!decision.update_prediction_from_accepted_measurement);
  assert(!decision.advance_prediction_without_measurement);
  assert(decision.update_prediction_from_rejected_measurement);
  assert(!decision.continue_to_pose_publish);
}

int main()
{
  test_accept_measurement_updates_pose_and_prediction();
  test_rejected_measurement_advances_prediction_without_pose_publish();
  test_rejected_seed_update_keeps_pose_publish_closed();
  return 0;
}
