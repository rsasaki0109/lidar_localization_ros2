// Experiment only: share the two existing pre-registration TF wait allowances.
#pragma once
namespace tf_shared_wait {
constexpr double original_timeout_sec = 0.1;
constexpr double shared_timeout_sec = 2 * original_timeout_sec;
// Synchronous scan callbacks remain on one executor thread. Other callback
// threads retain their original lookup timeout. Durations use the TF buffer clock.
inline thread_local bool active = false;
class Scope {
public:
  explicit Scope(bool enabled) noexcept : previous_(active) {active = enabled;}
  ~Scope() {active = previous_;}
  Scope(const Scope &) = delete;
  Scope & operator=(const Scope &) = delete;
private:
  bool previous_;
};
inline double lookupTimeoutSeconds() noexcept {
  return active ? 0.0 : original_timeout_sec;
}
}  // namespace tf_shared_wait
