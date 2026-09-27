#ifndef CONDITIONAL_SCAN_RECOVERY_FEEDBACK_TRACE_HPP_
#define CONDITIONAL_SCAN_RECOVERY_FEEDBACK_TRACE_HPP_

#include <Eigen/Core>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>

namespace conditional_scan_recovery
{
// Explicit row-major, round-trip decimal representation for offline replay.
inline std::string traceMatrix(const Eigen::Matrix4f & matrix)
{
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<float>::max_digits10);
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      if (row || col) {out << ',';}
      out << matrix(row, col);
    }
  }
  return out.str();
}
}  // namespace conditional_scan_recovery
#endif
