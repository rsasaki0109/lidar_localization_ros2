#ifndef LIDAR_LOCALIZATION_NDT_INITIALIZER_POLICY_HPP_
#define LIDAR_LOCALIZATION_NDT_INITIALIZER_POLICY_HPP_

#include <algorithm>

namespace lidar_localization
{

struct NdtInitializerRunInput
{
  bool enabled{false};
  bool initializer_available{false};
  int scan_count{0};
  int scans_required{0};
};

struct NdtInitializerProgressInput
{
  NdtInitializerRunInput run_input;
  bool has_converged{false};
};

struct NdtInitializerProgressDecision
{
  bool should_run{false};
  bool should_accept_refined_seed{false};
  int next_scan_count{0};
  bool completed{false};
  bool should_reset_initializer{false};
};

inline bool shouldRunNdtInitializer(const NdtInitializerRunInput & input)
{
  return input.enabled &&
         input.initializer_available &&
         input.scans_required > 0 &&
         input.scan_count < input.scans_required;
}

inline NdtInitializerProgressDecision updateNdtInitializerProgress(
  const NdtInitializerProgressInput & input)
{
  NdtInitializerProgressDecision decision;
  decision.should_run = shouldRunNdtInitializer(input.run_input);
  decision.next_scan_count = input.run_input.scan_count;
  if (!decision.should_run || !input.has_converged) {
    return decision;
  }

  decision.should_accept_refined_seed = true;
  decision.next_scan_count = std::max(0, input.run_input.scan_count) + 1;
  decision.completed = decision.next_scan_count >= input.run_input.scans_required;
  decision.should_reset_initializer = decision.completed;
  return decision;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_NDT_INITIALIZER_POLICY_HPP_
