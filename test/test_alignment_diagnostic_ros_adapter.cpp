#include "lidar_localization/alignment_diagnostic_ros_adapter.hpp"

#include <cassert>
#include <vector>

namespace ll = lidar_localization;

void test_make_ros_diagnostic_key_value_preserves_pair()
{
  const auto key_value = ll::makeRosDiagnosticKeyValue({"recovery_state", "tracking"});

  assert(key_value.key == "recovery_state");
  assert(key_value.value == "tracking");
}

void test_make_ros_diagnostic_key_values_preserves_order()
{
  const std::vector<ll::DiagnosticKeyValue> values{
    {"first", "1"},
    {"second", "2"}};

  const auto key_values = ll::makeRosDiagnosticKeyValues(values);

  assert(key_values.size() == 2);
  assert(key_values[0].key == "first");
  assert(key_values[0].value == "1");
  assert(key_values[1].key == "second");
  assert(key_values[1].value == "2");
}

void test_make_ros_alignment_diagnostic_key_values_uses_policy_values()
{
  ll::AlignmentDiagnosticValuesInput input;
  input.registration_method = "NDT";
  input.has_converged = true;
  input.recovery_state = "tracking";
  input.recovery_action = "accept_measurement";
  input.reinitialization_request_reason = "not_requested";

  const auto key_values = ll::makeRosAlignmentDiagnosticKeyValues(input);

  assert(!key_values.empty());
  assert(key_values.front().key == "registration_method");
  assert(key_values.front().value == "NDT");
}

int main()
{
  test_make_ros_diagnostic_key_value_preserves_pair();
  test_make_ros_diagnostic_key_values_preserves_order();
  test_make_ros_alignment_diagnostic_key_values_uses_policy_values();
  return 0;
}
