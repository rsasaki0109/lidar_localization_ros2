#pragma once
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <utility>

namespace tf_ready_queue {
// Experiment only. All calls must be serialized by the owner. Readiness must
// be a nonblocking query; the owner rechecks TF when using it. Payload owns the
// immutable cloud + receive-time twist snapshot. This class never runs a worker.
template<class Payload>
class Queue {
public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  using Duration = Clock::duration;
  struct Entry {std::int64_t stamp; Time received; Payload payload;};
  struct Dispatch {Entry entry; bool tf_ready;};
  struct Counts {std::size_t overflow=0, stale=0, invalidated=0, unordered=0;};
  Queue(std::size_t capacity, Duration wait, Duration max_age)
  : capacity_(capacity), wait_(wait), max_age_(max_age) {
    if (!capacity || wait < Duration::zero() || max_age <= wait) {
      throw std::invalid_argument("capacity>0 and 0<=wait<max_age required");
    }
  }
  bool push(std::int64_t stamp, Time now, Payload payload) {
    if (closed_) {return false;}
    if (last_stamp_ && stamp <= *last_stamp_) {++counts_.unordered; return false;}
    last_stamp_=stamp;
    if (entries_.size()==capacity_) {entries_.pop_front(); ++counts_.overflow;}
    entries_.push_back({stamp, now, std::move(payload)});
    return true;
  }
  template<class Ready>
  std::optional<Dispatch> take(Time now, Ready ready) {
    // Never publish stale work even when TF finally arrives. Deadlines are
    // steady-clock based; ROS clock jumps require owner-driven reset().
    while (!entries_.empty() && now-entries_.front().received >= max_age_) {
      entries_.pop_front(); ++counts_.stale;
    }
    if (entries_.empty()) {return std::nullopt;}
    const auto &entry=entries_.front();
    const bool available=ready(entry.stamp);
    if (!available && now-entry.received < wait_) {return std::nullopt;}
    Dispatch result{std::move(entries_.front()), available};
    entries_.pop_front();
    return result;
  }
  void reset() {
    counts_.invalidated+=entries_.size(); entries_.clear(); last_stamp_.reset();
  }
  void close() {reset(); closed_=true;}
  std::size_t size() const {return entries_.size();}
  Counts counts() const {return counts_;}
private:
  std::size_t capacity_;
  Duration wait_, max_age_;
  std::deque<Entry> entries_;
  std::optional<std::int64_t> last_stamp_;
  Counts counts_;
  bool closed_=false;
};
}  // namespace tf_ready_queue
