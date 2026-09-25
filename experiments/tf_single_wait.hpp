// Experiment only: one blocking odometry lookup attempt per synchronous scan callback.
#pragma once
namespace tf_single_wait {
// A synchronous cloud callback stays on its executor thread even while alignment
// releases the state lock. Other executor threads keep the original timeout.
inline thread_local bool * active_used = nullptr;
class Scope {
public:
  Scope() noexcept : previous_(active_used) {active_used = &used_;}
  ~Scope() {active_used = previous_;}
  Scope(const Scope &) = delete;
  Scope & operator=(const Scope &) = delete;
private:
  bool used_ = false;
  bool * previous_;
};
inline double timeoutSeconds() noexcept {
  if (!active_used) {return 0.1;}
  if (*active_used) {return 0.0;}
  *active_used = true;
  return 0.1;
}
}  // namespace tf_single_wait
