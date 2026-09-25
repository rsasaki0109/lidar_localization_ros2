#pragma once
// Only synchronous experimental dispatches skip blocking odom lookups.
namespace tf_lookup_scope {
inline thread_local bool nonblocking = false;
class Scope {
  bool previous_;
public:
  explicit Scope(bool value) : previous_(nonblocking) {nonblocking=value;}
  ~Scope() {nonblocking=previous_;}
  Scope(const Scope &)=delete;
  Scope &operator=(const Scope &)=delete;
};
inline double timeoutSeconds() {return nonblocking ? 0.0 : 0.1;}
}
