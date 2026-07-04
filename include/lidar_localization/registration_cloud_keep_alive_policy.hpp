#ifndef LIDAR_LOCALIZATION_REGISTRATION_CLOUD_KEEP_ALIVE_POLICY_HPP_
#define LIDAR_LOCALIZATION_REGISTRATION_CLOUD_KEEP_ALIVE_POLICY_HPP_

#include <cstddef>
#include <deque>

namespace lidar_localization
{

inline constexpr std::size_t kDefaultRegistrationSourceCloudKeepAliveCount = 8;
inline constexpr std::size_t kDefaultRegistrationTargetCloudKeepAliveCount = 8;

struct RegistrationCloudKeepAliveCount
{
  std::size_t value;
  bool was_adjusted;
};

// A value of 0 disables keep-alive and is only safe when the registration backend
// holds no reference to prior clouds.
inline RegistrationCloudKeepAliveCount normalizeRegistrationCloudKeepAliveCount(
  int requested,
  std::size_t fallback)
{
  if (requested < 0) {
    return {fallback, true};
  }
  return {static_cast<std::size_t>(requested), false};
}

template<typename CloudPtr>
inline void keepRegistrationCloudAlive(
  std::deque<CloudPtr> & recent_clouds,
  const CloudPtr & cloud,
  std::size_t max_count)
{
  if (!cloud || max_count == 0) {
    return;
  }

  recent_clouds.push_back(cloud);
  while (recent_clouds.size() > max_count) {
    recent_clouds.pop_front();
  }
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_REGISTRATION_CLOUD_KEEP_ALIVE_POLICY_HPP_
