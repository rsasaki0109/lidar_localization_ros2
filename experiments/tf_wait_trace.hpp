// Diagnostic instrumentation only; not intended for production promotion.
#pragma once
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <builtin_interfaces/msg/time.hpp>
namespace tf_wait_trace {
struct Sink {
  std::FILE * file = nullptr;
  std::mutex mutex;
  Sink() {
    const char * path = std::getenv("JEPLO_TF_WAIT_TRACE");
    if (path && *path) {
      file = std::fopen(path, "wx");
      if (!file) {throw std::runtime_error("Cannot create TF wait trace");}
      std::fprintf(file, "site,stamp_ns,start_ns,end_ns,success\n");
    }
  }
  ~Sink() {if (file) {std::fclose(file);}}
};
inline Sink & sink() {static Sink instance; return instance;}
inline long long now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct Span {
  const char * site;
  long long stamp;
  Sink & output;
  long long begin;
  bool success = false;
  Span(const char * name, const builtin_interfaces::msg::Time & time)
  : site(name), stamp(static_cast<long long>(time.sec) * 1000000000LL + time.nanosec),
    output(sink()), begin(output.file ? now() : 0) {}
  ~Span() {
    if (!output.file) {return;}
    const auto end = now();
    std::lock_guard<std::mutex> lock(output.mutex);
    std::fprintf(output.file, "%s,%lld,%lld,%lld,%d\n", site, stamp, begin, end, success);
  }
};
template<class F>
auto lookup(const char * site, const builtin_interfaces::msg::Time & stamp, F && operation) {
  Span span(site, stamp);
  auto result = std::forward<F>(operation)();
  span.success = true;
  return result;
}
}  // namespace tf_wait_trace
