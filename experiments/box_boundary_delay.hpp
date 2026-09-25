#pragma once
// Experiment only: inject one bounded delay outside the node state lock.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <thread>
namespace box_boundary_experiment {
struct Delay {
  double stamp{0.0};
  double seconds{0.0};
  bool matches(double scan) const {
    return seconds > 0.0 && std::isfinite(scan) && std::abs(scan - stamp) <= 0.000002;
  }
};
inline Delay configuration() {
  const char * stamp = std::getenv("JEPLO_BOUNDARY_STAMP");
  const char * seconds = std::getenv("JEPLO_BOUNDARY_DELAY_SEC");
  if (!stamp && !seconds) {return {};}
  if (!stamp || !seconds) {throw std::runtime_error("Both boundary experiment values required");}
  char * end_stamp = nullptr; char * end_seconds = nullptr;
  Delay d{std::strtod(stamp, &end_stamp), std::strtod(seconds, &end_seconds)};
  if (end_stamp == stamp || *end_stamp || end_seconds == seconds || *end_seconds ||
      !std::isfinite(d.stamp) || !std::isfinite(d.seconds) || d.stamp <= 0.0 ||
      d.seconds < 0.0 || d.seconds > 1.0) {
    throw std::runtime_error("Invalid boundary experiment configuration");
  }
  return d;
}
inline void inject(double stamp) {
  static const Delay config = configuration();
  static std::atomic<bool> used{false};
  if (!config.matches(stamp) || used.exchange(true)) {return;}
  std::fprintf(stderr, "BOUNDARY_DELAY stamp=%.9f seconds=%.6f\n", stamp, config.seconds);
  std::this_thread::sleep_for(std::chrono::duration<double>(config.seconds));
}
}
