#ifndef LIDAR_LOCALIZATION_REGISTRATION_CLOUD_KEEP_ALIVE_POLICY_HPP_
#define LIDAR_LOCALIZATION_REGISTRATION_CLOUD_KEEP_ALIVE_POLICY_HPP_

#include <cstddef>
#include <deque>

namespace lidar_localization
{

inline constexpr std::size_t kDefaultRegistrationSourceCloudKeepAliveCount = 8;
inline constexpr std::size_t kDefaultRegistrationTargetCloudKeepAliveCount = 8;

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
