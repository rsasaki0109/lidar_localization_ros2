#pragma once
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>

namespace jeplo_cost_trace {
// Experimental, opt-in, bounded source-time window. No ROS parameters or state changes.
class Scope {
  using Clock = std::chrono::steady_clock;
  struct Window { double first{0}, last{-1}; };
  static const Window & window() {
    static const Window value = [] {
      Window w;
      const char * text = std::getenv("JEPLO_COST_TRACE_WINDOW");
      if (text && std::sscanf(text, "%lf %lf", &w.first, &w.last) != 2) {w.last = -1;}
      return w;
    }();
    return value;
  }
  const char * name_;
  double stamp_;
  bool active_;
  std::array<const char *, 20> labels_{};
  std::array<long long, 20> times_{};
  unsigned size_{0};
public:
  Scope(const char * name, double stamp) : name_(name), stamp_(stamp),
    active_(stamp >= window().first && stamp <= window().last) {mark("enter");}
  void mark(const char * label) {
    if (!active_ || size_ == labels_.size()) {return;}
    labels_[size_] = label;
    times_[size_++] = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
  }
  ~Scope() {
    if (!active_) {return;}
    mark("exit");
    char line[2048];
    int used = std::snprintf(line, sizeof(line), "JEPLO_COST %s %.9f", name_, stamp_);
    for (unsigned i = 0; i < size_ && used > 0 && used < 1900; ++i) {
      used += std::snprintf(line + used, sizeof(line) - used, " %s=%lld", labels_[i], times_[i]);
    }
    if (used > 0 && used < 2047) {line[used++] = '\n';line[used] = 0;std::fputs(line, stderr);}
  }
};
}
