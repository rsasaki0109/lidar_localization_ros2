// Copyright 2026 Sasaki
// SPDX-License-Identifier: BSD-2-Clause
#include "lidar_localization/source_wait_lookup.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>

#ifdef NDEBUG
#error "Native buffer fixture requires active assertions"
#endif

using namespace std::chrono_literals;
using Steady = std::chrono::steady_clock;

builtin_interfaces::msg::Time stamp(int seconds)
{
  builtin_interfaces::msg::Time value;
  value.sec = seconds;
  return value;
}

void insert(tf2_ros::Buffer & buffer, int seconds, const std::string & child = "base")
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = child;
  transform.header.stamp = stamp(seconds);
  transform.transform.translation.x = seconds;
  transform.transform.rotation.w = 1.0;
  assert(buffer.setTransform(transform, "fixture", false));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  // The fixture's insertion thread stands in for TransformListener's worker.
  buffer.setUsingDedicatedThread(true);
  lidar_localization::SourceWaitLookup lookup;
  const auto budget = rclcpp::Duration::from_seconds(0.2);
  auto fail = [&](int seconds, const std::string & child = "base") {
      const auto start = Steady::now();
      bool failed = false;
      try { lookup.lookup(buffer, "odom", child, stamp(seconds), budget); }
      catch (const tf2::TransformException &) { failed = true; }
      assert(failed);
      return std::chrono::duration<double>(Steady::now() - start).count();
    };
  assert(fail(1) >= .18);
  auto start = Steady::now();
  for (int scan = 1; scan <= 40; ++scan) { fail(scan); }
  double repeated = std::chrono::duration<double>(Steady::now() - start).count();
  assert(repeated < .15);

  // Source advance permits waiting again; delayed TF is consumed exactly.
  insert(buffer, 1);
  std::thread delayed([&] { std::this_thread::sleep_for(150ms); insert(buffer, 2); });
  auto resolved = lookup.lookup(buffer, "odom", "base", stamp(2), budget);
  delayed.join();
  assert(resolved.header.stamp == stamp(2));
  assert(resolved.transform.translation.x == 2.0);

  // Progress DURING an unsuccessful wait is remembered, not the starting stamp.
  std::thread incomplete([&] { std::this_thread::sleep_for(50ms); insert(buffer, 3); });
  assert(fail(5) >= .18);
  incomplete.join();
  assert(fail(5) < .15);
  insert(buffer, 5);
  resolved = lookup.lookup(buffer, "odom", "base", stamp(5), budget);
  assert(resolved.transform.translation.x == 5.0);
  assert(fail(6) >= .18);  // success reset the suppression
  assert(fail(6) < .15);

  // Available historical data is checked even when the latest stamp is unchanged.
  resolved = lookup.lookup(buffer, "odom", "base", stamp(4), budget);
  assert(resolved.transform.translation.x == 4.0);  // native interpolation
  assert(fail(6) >= .18);
  lookup.reset();
  assert(fail(6) >= .18);
  assert(fail(6) < .15);
  assert(fail(6, "another_base") >= .18);  // frame change resets
  assert(fail(6, "another_base") < .15);

  // An out-of-order insertion can satisfy a past request without latest progress.
  geometry_msgs::msg::TransformStamped backfill;
  backfill.header.frame_id = "odom";
  backfill.child_frame_id = "base";
  backfill.header.stamp.nanosec = 500000000;
  backfill.transform.translation.x = .5;
  backfill.transform.rotation.w = 1.;
  bool past_failed = false;
  try { lookup.lookup(buffer, "odom", "base", backfill.header.stamp, budget); }
  catch (const tf2::TransformException &) { past_failed = true; }
  assert(past_failed);
  assert(buffer.setTransform(backfill, "fixture", false));
  resolved = lookup.lookup(buffer, "odom", "base", backfill.header.stamp, budget);
  assert(resolved.transform.translation.x == .5);
  assert(fail(6) >= .18);

  // Buffer clear / backward source stamp does not permanently suppress recovery.
  buffer.clear();
  insert(buffer, 1);
  std::thread restored([&] { std::this_thread::sleep_for(50ms); insert(buffer, 2); });
  resolved = lookup.lookup(buffer, "odom", "base", stamp(2), budget);
  restored.join();
  assert(resolved.header.stamp == stamp(2));
  std::cout << "Native tf2 fixture passed; 40 absent-source lookups: " << repeated
            << " s; delayed arrival, partial progress, interpolation, reset, frames, clear passed\n";
  rclcpp::shutdown();
}
