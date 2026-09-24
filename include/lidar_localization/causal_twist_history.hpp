#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>

namespace lidar_localization
{

struct TimestampedTwist
{
  double stamp_sec{0.0};
  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
};

// Bounded timestamp ordering for direct twist prediction. Selection does not
// integrate the velocity history or impose a stale-sample cutoff.
// Caller owns synchronization; returned pointers are valid until mutation.
class CausalTwistHistory
{
public:
  explicit CausalTwistHistory(std::size_t capacity = 1024) : capacity_(capacity)
  {
    if (capacity == 0) {throw std::invalid_argument("twist history capacity is zero");}
  }

  bool insert(const TimestampedTwist & sample)
  {
    if (!std::isfinite(sample.stamp_sec) || sample.stamp_sec < 0.0 ||
      !sample.linear.allFinite() || !sample.angular.allFinite())
    {
      return false;
    }
    auto it = std::lower_bound(samples_.begin(), samples_.end(), sample.stamp_sec,
      [](const TimestampedTwist & a, double stamp) {return a.stamp_sec < stamp;});
    if (it != samples_.end() && it->stamp_sec == sample.stamp_sec) {
      *it = sample;
    } else {
      samples_.insert(it, sample);
    }
    while (samples_.size() > capacity_) {samples_.pop_front();}
    return true;
  }

  const TimestampedTwist * atOrBefore(double stamp_sec) const
  {
    if (!std::isfinite(stamp_sec)) {return nullptr;}
    auto it = std::upper_bound(samples_.begin(), samples_.end(), stamp_sec,
      [](double stamp, const TimestampedTwist & a) {return stamp < a.stamp_sec;});
    if (it == samples_.begin()) {return nullptr;}
    return &*--it;
  }

  void clear() {samples_.clear();}
  std::size_t size() const {return samples_.size();}

private:
  std::size_t capacity_;
  std::deque<TimestampedTwist> samples_;
};

}  // namespace lidar_localization
