#include "lidar_localization/callback_state_coordinator.hpp"

#include <atomic>
#include <cassert>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

namespace
{

void test_recursive_lifecycle_entry()
{
  lidar_localization::CallbackStateCoordinator coordinator;
  auto lifecycle_lock = coordinator.lockState();
  auto initial_pose_lock = coordinator.lockState();
  coordinator.advanceInitialPoseGeneration();
  assert(coordinator.initialPoseGeneration() == 1);
}

void test_reset_invalidates_alignment_commit()
{
  lidar_localization::CallbackStateCoordinator coordinator;
  int protected_pose = 10;

  auto scan_lock = coordinator.lockState();
  const std::uint64_t seed_generation = coordinator.initialPoseGeneration();
  scan_lock.unlock();  // The expensive registration runs without the state lock.

  std::thread reset_thread([&]() {
    auto reset_lock = coordinator.lockState();
    protected_pose = 20;
    coordinator.advanceInitialPoseGeneration();
  });
  reset_thread.join();

  scan_lock.lock();
  const bool may_commit = coordinator.initialPoseGenerationMatches(seed_generation);
  if (may_commit) {
    protected_pose = 30;
  }
  assert(!may_commit);
  assert(protected_pose == 20);
}

void test_parallel_state_updates_are_serialized()
{
  lidar_localization::CallbackStateCoordinator coordinator;
  int protected_counter = 0;
  constexpr int kThreadCount = 4;
  constexpr int kUpdatesPerThread = 5000;
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);
  for (int thread_index = 0; thread_index < kThreadCount; ++thread_index) {
    threads.emplace_back([&]() {
      for (int update = 0; update < kUpdatesPerThread; ++update) {
        auto state_lock = coordinator.lockState();
        ++protected_counter;
      }
    });
  }
  for (auto & thread : threads) {
    thread.join();
  }
  assert(protected_counter == kThreadCount * kUpdatesPerThread);
}

void test_shutdown_waits_for_registration_without_lock_order_deadlock()
{
  lidar_localization::CallbackStateCoordinator coordinator;
  std::mutex event_mutex;
  std::condition_variable event;
  bool registration_started = false;
  bool allow_registration_to_finish = false;
  bool shutdown_holds_state = false;
  bool backend_destroyed = false;

  std::thread scan_thread([&]() {
    auto state_lock = coordinator.lockState();
    state_lock.unlock();
    {
      auto execution_lock = coordinator.lockRegistrationExecution();
      std::unique_lock<std::mutex> event_lock(event_mutex);
      registration_started = true;
      event.notify_all();
      event.wait(event_lock, [&]() {return allow_registration_to_finish;});
      assert(!backend_destroyed);
    }
    state_lock.lock();
  });

  {
    std::unique_lock<std::mutex> event_lock(event_mutex);
    event.wait(event_lock, [&]() {return registration_started;});
  }
  std::thread shutdown_thread([&]() {
    auto state_lock = coordinator.lockState();
    {
      std::lock_guard<std::mutex> event_lock(event_mutex);
      shutdown_holds_state = true;
    }
    event.notify_all();
    auto execution_lock = coordinator.lockRegistrationExecution();
    backend_destroyed = true;
  });

  {
    std::unique_lock<std::mutex> event_lock(event_mutex);
    event.wait(event_lock, [&]() {return shutdown_holds_state;});
    allow_registration_to_finish = true;
  }
  event.notify_all();
  scan_thread.join();
  shutdown_thread.join();
  assert(backend_destroyed);
}

}  // namespace

int main()
{
  test_recursive_lifecycle_entry();
  test_reset_invalidates_alignment_commit();
  test_parallel_state_updates_are_serialized();
  test_shutdown_waits_for_registration_without_lock_order_deadlock();
  return 0;
}
