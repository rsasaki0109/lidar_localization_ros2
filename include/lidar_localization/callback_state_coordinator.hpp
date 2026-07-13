#ifndef LIDAR_LOCALIZATION__CALLBACK_STATE_COORDINATOR_HPP_
#define LIDAR_LOCALIZATION__CALLBACK_STATE_COORDINATOR_HPP_

#include <atomic>
#include <cstdint>
#include <mutex>

namespace lidar_localization
{

// Coordinates state shared by the default callback group and the dedicated
// /initialpose callback group. The recursive mutex is intentional: lifecycle
// activation applies its configured initial pose through the same callback
// implementation while already holding the state lock.
class CallbackStateCoordinator
{
public:
  using StateLock = std::unique_lock<std::recursive_mutex>;
  using RegistrationExecutionLock = std::unique_lock<std::mutex>;

  StateLock lockState()
  {
    return StateLock(state_mutex_);
  }

  RegistrationExecutionLock lockRegistrationExecution()
  {
    return RegistrationExecutionLock(registration_execution_mutex_);
  }

  std::uint64_t initialPoseGeneration() const
  {
    return initial_pose_generation_.load(std::memory_order_acquire);
  }

  void advanceInitialPoseGeneration()
  {
    initial_pose_generation_.fetch_add(1, std::memory_order_release);
  }

  bool initialPoseGenerationMatches(std::uint64_t generation) const
  {
    return initialPoseGeneration() == generation;
  }

private:
  std::recursive_mutex state_mutex_;
  std::mutex registration_execution_mutex_;
  std::atomic<std::uint64_t> initial_pose_generation_{0};
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION__CALLBACK_STATE_COORDINATOR_HPP_
