#include "lidar_localization/imu_seed_consistency_policy.hpp"

#include <cassert>
#include <limits>

namespace ll = lidar_localization;

void test_seed_is_allowed_only_after_required_consecutive_passes()
{
  const ll::ImuSeedConsistencyParams params{0.5, 5.0, 3};
  ll::ImuSeedConsistencyState state;

  for (std::size_t pass = 1; pass <= 3; ++pass) {
    const auto update = ll::updateImuSeedConsistency(
      state, params, {true, true, 0.2, 2.0});
    assert(update.comparison_valid);
    assert(update.sample_passed);
    assert(update.state.valid_comparison_count == pass);
    assert(update.state.consecutive_pass_count == pass);
    assert(update.state.seed_allowed == (pass == 3));
    state = update.state;
  }
}

void test_failed_comparison_revokes_seed_and_resets_streak()
{
  const ll::ImuSeedConsistencyParams params{0.5, 5.0, 2};
  ll::ImuSeedConsistencyState state{2, 2, true};

  const auto update = ll::updateImuSeedConsistency(
    state, params, {true, true, 0.6, 2.0});

  assert(update.comparison_valid);
  assert(!update.sample_passed);
  assert(update.state.valid_comparison_count == 3);
  assert(update.state.consecutive_pass_count == 0);
  assert(!update.state.seed_allowed);
}

void test_rotation_must_also_pass()
{
  const ll::ImuSeedConsistencyParams params{0.5, 5.0, 1};
  const auto update = ll::updateImuSeedConsistency(
    {}, params, {true, true, 0.1, 5.1});

  assert(update.comparison_valid);
  assert(!update.sample_passed);
  assert(!update.state.seed_allowed);
}

void test_missing_or_non_finite_evidence_breaks_consecutive_chain()
{
  const ll::ImuSeedConsistencyParams params{0.5, 5.0, 2};
  const ll::ImuSeedConsistencyState one_pass{1, 1, false};

  const auto missing = ll::updateImuSeedConsistency(
    one_pass, params, {true, false, 0.0, 0.0});
  assert(!missing.comparison_valid);
  assert(missing.state.valid_comparison_count == 1);
  assert(missing.state.consecutive_pass_count == 0);
  assert(!missing.state.seed_allowed);

  const auto non_finite = ll::updateImuSeedConsistency(
    one_pass, params,
    {true, true, std::numeric_limits<double>::quiet_NaN(), 0.0});
  assert(!non_finite.comparison_valid);
  assert(non_finite.state.consecutive_pass_count == 0);
}

void test_established_gate_survives_temporarily_missing_prediction()
{
  const ll::ImuSeedConsistencyParams params{0.5, 5.0, 2};
  const ll::ImuSeedConsistencyState established{8, 2, true};

  const auto missing = ll::updateImuSeedConsistency(
    established, params, {true, false, 0.0, 0.0});

  assert(!missing.comparison_valid);
  assert(missing.state.valid_comparison_count == 8);
  assert(missing.state.consecutive_pass_count == 2);
  assert(missing.state.seed_allowed);

  const auto resumed = ll::updateImuSeedConsistency(
    missing.state, params, {true, true, 0.2, 2.0});
  assert(resumed.comparison_valid);
  assert(resumed.sample_passed);
  assert(resumed.state.valid_comparison_count == 9);
  assert(resumed.state.consecutive_pass_count == 3);
  assert(resumed.state.seed_allowed);
}

void test_established_gate_fails_closed_on_non_finite_prediction()
{
  const ll::ImuSeedConsistencyParams params{0.5, 5.0, 2};
  const ll::ImuSeedConsistencyState established{8, 2, true};

  const auto non_finite = ll::updateImuSeedConsistency(
    established, params,
    {true, true, std::numeric_limits<double>::quiet_NaN(), 0.0});

  assert(!non_finite.comparison_valid);
  assert(non_finite.state.consecutive_pass_count == 0);
  assert(!non_finite.state.seed_allowed);
}

void test_invalid_parameters_fail_closed()
{
  ll::ImuSeedConsistencyParams params;
  params.required_consecutive_passes = 0;
  const auto update = ll::updateImuSeedConsistency(
    {5, 5, true}, params, {true, true, 0.0, 0.0});

  assert(!update.comparison_valid);
  assert(!update.state.seed_allowed);
  assert(update.state.consecutive_pass_count == 0);
}

int main()
{
  test_seed_is_allowed_only_after_required_consecutive_passes();
  test_failed_comparison_revokes_seed_and_resets_streak();
  test_rotation_must_also_pass();
  test_missing_or_non_finite_evidence_breaks_consecutive_chain();
  test_established_gate_survives_temporarily_missing_prediction();
  test_established_gate_fails_closed_on_non_finite_prediction();
  test_invalid_parameters_fail_closed();
  return 0;
}
