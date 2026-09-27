// Copyright 2026 Sasaki
// SPDX-License-Identifier: BSD-2-Clause
#include "lidar_localization/source_wait_policy.hpp"
#include <cassert>
#include <iostream>

#ifdef NDEBUG
#error "This behavioral fixture must retain assertions in Release builds"
#endif

int main()
{
  lidar_localization::SourceWaitPolicy policy;
  assert(policy.allowWait(std::nullopt));
  policy.recordTimeout(std::nullopt);
  for (int scan = 0; scan != 40; ++scan) {
    assert(!policy.allowWait(std::nullopt));
  }
  // First TF arrival, including a valid zero stamp, must re-enable waiting.
  assert(policy.allowWait(0));
  policy.recordTimeout(0);
  assert(!policy.allowWait(0));
  assert(policy.allowWait(100));
  policy.recordTimeout(100);
  assert(!policy.allowWait(100));
  assert(policy.allowWait(101));
  // A clock rewind or a changed source can make an older stamp new data.
  assert(policy.allowWait(50));
  // Use the source observed AFTER timeout: progress during an unsuccessful
  // wait must not immediately cause repeated waits on the same data.
  policy.recordTimeout(200);
  assert(!policy.allowWait(200));
  assert(policy.allowWait(201));
  // Exact lookup success, initial pose, frame change and lifecycle reset.
  policy.reset();
  assert(policy.allowWait(200));
  assert(policy.allowWait(std::nullopt));
  std::cout << "Source wait policy scenarios passed; native TF behavior untested\n";
}
