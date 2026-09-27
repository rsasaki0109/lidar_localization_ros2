// Copyright 2026 Sasaki
// SPDX-License-Identifier: BSD-2-Clause
#ifndef LIDAR_LOCALIZATION_SOURCE_WAIT_POLICY_HPP_
#define LIDAR_LOCALIZATION_SOURCE_WAIT_POLICY_HPP_

#include <cstdint>
#include <optional>

namespace lidar_localization
{
// Always try an immediate lookup first. After a timed wait fails, do not
// wait again until the latest source transform changes. An absent source
// is distinct from a valid zero-stamp source. Call reset when frames or
// lifecycle state change; successful exact lookups also reset the policy.
class SourceWaitPolicy
{
public:
  using SourceStamp = std::optional<std::int64_t>;

  bool allowWait(SourceStamp latest_source) const
  {
    return !timed_out_ || latest_source != failed_source_;
  }

  void recordTimeout(SourceStamp latest_source_after_wait)
  {
    failed_source_ = latest_source_after_wait;
    timed_out_ = true;
  }

  void reset()
  {
    timed_out_ = false;
    failed_source_.reset();
  }

private:
  bool timed_out_{false};
  SourceStamp failed_source_;
};
}  // namespace lidar_localization
#endif  // LIDAR_LOCALIZATION_SOURCE_WAIT_POLICY_HPP_
