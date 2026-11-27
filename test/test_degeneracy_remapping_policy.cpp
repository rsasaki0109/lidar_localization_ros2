#include <cassert>
#include <cmath>
#include <cstdio>

#include "lidar_localization/degeneracy_remapping_policy.hpp"

namespace ll = lidar_localization;

namespace
{

Eigen::Matrix4f at(float x, float y)
{
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m(0, 3) = x;
  m(1, 3) = y;
  return m;
}

ll::RegistrationLocalizabilityMetrics corridor(double weak_ratio, double wx, double wy, double wyaw)
{
  ll::RegistrationLocalizabilityMetrics m;
  m.enabled = true;
  m.valid = true;
  m.weak_ratio = weak_ratio;
  const double n = std::sqrt(wx * wx + wy * wy + wyaw * wyaw);
  m.weakest_eigenvector = {{wx / n, wy / n, 0.0, 0.0, 0.0, wyaw / n}};
  return m;
}

}  // namespace

int main()
{
  ll::DegeneracyRemappingParams params;
  const Eigen::Matrix4f seed = at(10.0f, 2.0f);

  // Off by default: nothing changes.
  Eigen::Matrix4f aligned = at(10.6f, 2.1f);
  assert(!ll::remapDegenerateCorrection(seed, aligned, corridor(0.01, 1, 0, 0), params).applied);
  assert(aligned(0, 3) == 10.6f);

  params.enable = true;
  // A corridor along x: the 0.6 m along it is dropped, the 0.1 m across kept.
  aligned = at(10.6f, 2.1f);
  auto r = ll::remapDegenerateCorrection(seed, aligned, corridor(0.01, 1, 0, 0), params);
  assert(r.applied && std::abs(r.dropped_m - 0.6) < 1e-5);
  assert(std::abs(aligned(0, 3) - 10.0f) < 1e-5f && std::abs(aligned(1, 3) - 2.1f) < 1e-5f);

  // A corridor at 45 degrees: the component along it only.
  aligned = at(11.0f, 3.0f);
  r = ll::remapDegenerateCorrection(seed, aligned, corridor(0.01, 1, 1, 0), params);
  assert(r.applied && std::abs(std::abs(r.dropped_m) - std::sqrt(2.0)) < 1e-5);
  assert(std::abs(aligned(0, 3) - 10.0f) < 1e-5f && std::abs(aligned(1, 3) - 2.0f) < 1e-5f);

  // Well constrained (ratio above the threshold): kept.
  aligned = at(10.6f, 2.1f);
  assert(!ll::remapDegenerateCorrection(seed, aligned, corridor(0.2, 1, 0, 0), params).applied);
  assert(aligned(0, 3) == 10.6f);

  // Weakest in yaw, not a horizontal translation: kept.
  aligned = at(10.6f, 2.1f);
  assert(!ll::remapDegenerateCorrection(seed, aligned, corridor(0.01, 0.1, 0, 1), params).applied);

  // No analysis (another backend): kept.
  ll::RegistrationLocalizabilityMetrics none;
  aligned = at(10.6f, 2.1f);
  assert(!ll::remapDegenerateCorrection(seed, aligned, none, params).applied);

  std::printf("degeneracy remapping policy: ok\n");
  return 0;
}
