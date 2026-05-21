#ifndef LIDAR_LOCALIZATION_LOCALIZATION_UPDATE_POLICY_HPP_
#define LIDAR_LOCALIZATION_LOCALIZATION_UPDATE_POLICY_HPP_

#include "lidar_localization/measurement_gate_policy.hpp"

namespace lidar_localization
{

enum class LocalizationUpdateAction
{
  kAcceptMeasurement = 0,
  kAdvancePredictionWithoutMeasurement,
  kUpdatePredictionFromRejectedMeasurement,
};

struct LocalizationUpdateDecision
{
  LocalizationUpdateAction action{LocalizationUpdateAction::kAcceptMeasurement};
  bool update_current_pose{true};
  bool update_prediction_from_accepted_measurement{true};
  bool advance_prediction_without_measurement{false};
  bool update_prediction_from_rejected_measurement{false};
  bool continue_to_pose_publish{true};
};

inline LocalizationUpdateDecision makeAcceptedLocalizationUpdateDecision()
{
  return {};
}

inline LocalizationUpdateDecision makePredictionAdvanceLocalizationUpdateDecision()
{
  LocalizationUpdateDecision decision;
  decision.action = LocalizationUpdateAction::kAdvancePredictionWithoutMeasurement;
  decision.update_current_pose = false;
  decision.update_prediction_from_accepted_measurement = false;
  decision.advance_prediction_without_measurement = true;
  decision.continue_to_pose_publish = false;
  return decision;
}

inline LocalizationUpdateDecision makeRejectedSeedLocalizationUpdateDecision()
{
  LocalizationUpdateDecision decision;
  decision.action = LocalizationUpdateAction::kUpdatePredictionFromRejectedMeasurement;
  decision.update_current_pose = false;
  decision.update_prediction_from_accepted_measurement = false;
  decision.update_prediction_from_rejected_measurement = true;
  decision.continue_to_pose_publish = false;
  return decision;
}

inline LocalizationUpdateDecision decideLocalizationUpdate(
  const MeasurementGateDecision & gate_decision)
{
  if (!gate_decision.reject_measurement) {
    return makeAcceptedLocalizationUpdateDecision();
  }

  if (gate_decision.rejected_seed_update_applied) {
    return makeRejectedSeedLocalizationUpdateDecision();
  }

  return makePredictionAdvanceLocalizationUpdateDecision();
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_LOCALIZATION_UPDATE_POLICY_HPP_
