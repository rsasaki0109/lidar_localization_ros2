#include "lidar_localization/registration_cloud_keep_alive_policy.hpp"

#include <cassert>
#include <deque>
#include <memory>

namespace ll = lidar_localization;

void test_keep_alive_retains_only_recent_entries()
{
  std::deque<std::shared_ptr<int>> recent;

  for (int i = 0; i < 10; ++i) {
    ll::keepRegistrationCloudAlive(recent, std::make_shared<int>(i), 3);
  }

  assert(recent.size() == 3);
  assert(*recent[0] == 7);
  assert(*recent[1] == 8);
  assert(*recent[2] == 9);
}

void test_keep_alive_ignores_empty_inputs()
{
  std::deque<std::shared_ptr<int>> recent;

  ll::keepRegistrationCloudAlive(recent, std::shared_ptr<int>{}, 3);
  assert(recent.empty());

  ll::keepRegistrationCloudAlive(recent, std::make_shared<int>(1), 0);
  assert(recent.empty());
}

void test_default_limits_are_small_bounded_windows()
{
  assert(ll::kDefaultRegistrationSourceCloudKeepAliveCount == 8);
  assert(ll::kDefaultRegistrationTargetCloudKeepAliveCount == 8);
}

void test_normalize_registration_cloud_keep_alive_count()
{
  const auto negative = ll::normalizeRegistrationCloudKeepAliveCount(-1, 8);
  assert(negative.value == 8);
  assert(negative.was_adjusted);

  const auto zero = ll::normalizeRegistrationCloudKeepAliveCount(0, 8);
  assert(zero.value == 0);
  assert(!zero.was_adjusted);

  const auto positive = ll::normalizeRegistrationCloudKeepAliveCount(4, 8);
  assert(positive.value == 4);
  assert(!positive.was_adjusted);
}

int main()
{
  test_keep_alive_retains_only_recent_entries();
  test_keep_alive_ignores_empty_inputs();
  test_default_limits_are_small_bounded_windows();
  test_normalize_registration_cloud_keep_alive_count();
  return 0;
}
