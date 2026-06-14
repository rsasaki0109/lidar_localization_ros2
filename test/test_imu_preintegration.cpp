// Numerical regression tests for the on-manifold IMU preintegration core
// (include/lidar_localization/imu_preintegration.hpp, Forster et al. 2017).
//
// The preintegration math is the heart of the IMU estimator that issues #36
// (imu preintegration) and #77 (IMU angular-velocity utilization + estimator)
// ask for, yet only its *guard* policy had a test -- the integration, bias
// Jacobians, residual and covariance propagation were unverified. These tests
// pin them against closed-form answers and a finite-difference check, in the
// repository's plain-assert style. They are pure Eigen (no ROS), so they build
// and run standalone:
//
//   g++ -std=c++17 -I include -I /usr/include/eigen3 \
//       test/test_imu_preintegration.cpp -o /tmp/t && /tmp/t

#include "lidar_localization/imu_preintegration.hpp"

#include <Eigen/Eigenvalues>

#include <cassert>
#include <cmath>
#include <vector>

namespace {

bool close(double a, double b, double tol) { return std::abs(a - b) < tol; }

bool vclose(const Eigen::Vector3d & a, const Eigen::Vector3d & b, double tol)
{
  return (a - b).norm() < tol;
}

// Geodesic distance between two rotations, in radians.
double rot_diff(const Eigen::Matrix3d & A, const Eigen::Matrix3d & B)
{
  return so3::Log(A.transpose() * B).norm();
}

struct ImuTick
{
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;
  double dt;
};

// Integrate a fixed measurement stream under a given bias, returning the
// (jacobian-bearing) preintegration object.
ImuPreintegration integrate_stream(
  const std::vector<ImuTick> & ticks,
  const Eigen::Vector3d & bg, const Eigen::Vector3d & ba)
{
  ImuPreintegration pre;
  pre.reset();
  for (const auto & t : ticks) {
    pre.integrate(t.gyro, t.accel, bg, ba, t.dt);
  }
  return pre;
}

// ---------------------------------------------------------------------------

void test_reset_is_identity()
{
  ImuPreintegration pre;
  pre.integrate({0.3, -0.1, 0.2}, {1.0, 0.0, 9.0}, {0, 0, 0}, {0, 0, 0}, 0.01);
  pre.reset();
  assert(vclose(pre.delta_p, Eigen::Vector3d::Zero(), 1e-12));
  assert(vclose(pre.delta_v, Eigen::Vector3d::Zero(), 1e-12));
  assert(rot_diff(pre.delta_R, Eigen::Matrix3d::Identity()) < 1e-12);
  assert(close(pre.dt_sum, 0.0, 1e-12));
  assert(pre.covariance.norm() < 1e-12);
}

void test_dt_outside_range_is_ignored()
{
  // The integrator rejects non-positive and >0.5 s steps (sensor dropouts).
  ImuPreintegration pre;
  pre.reset();
  pre.integrate({0.3, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0, 0, 0}, {0, 0, 0}, 0.0);
  pre.integrate({0.3, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0, 0, 0}, {0, 0, 0}, -0.01);
  pre.integrate({0.3, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0, 0, 0}, {0, 0, 0}, 0.6);
  assert(close(pre.dt_sum, 0.0, 1e-12));
  assert(rot_diff(pre.delta_R, Eigen::Matrix3d::Identity()) < 1e-12);
}

void test_pure_rotation_matches_analytic()
{
  // Constant angular velocity, zero specific force -> delta_R = Exp(omega * t),
  // and position/velocity stay zero (no acceleration).
  const Eigen::Vector3d omega(0.2, -0.3, 0.15);
  const double dt = 0.005;
  const int n = 200;  // 1.0 s
  ImuPreintegration pre;
  pre.reset();
  for (int i = 0; i < n; ++i) {
    pre.integrate(omega, Eigen::Vector3d::Zero(), {0, 0, 0}, {0, 0, 0}, dt);
  }
  const Eigen::Matrix3d expected_R = so3::Exp(omega * (dt * n));
  assert(rot_diff(pre.delta_R, expected_R) < 1e-9);
  assert(vclose(pre.delta_v, Eigen::Vector3d::Zero(), 1e-12));
  assert(vclose(pre.delta_p, Eigen::Vector3d::Zero(), 1e-12));
  assert(close(pre.dt_sum, dt * n, 1e-9));
}

void test_pure_translation_matches_analytic()
{
  // Zero rotation, constant specific force a -> the discrete scheme is exact for
  // constant acceleration: delta_v = a*t, delta_p = 0.5*a*t^2.
  const Eigen::Vector3d a(0.5, -1.0, 2.0);
  const double dt = 0.01;
  const int n = 100;  // 1.0 s
  ImuPreintegration pre;
  pre.reset();
  for (int i = 0; i < n; ++i) {
    pre.integrate(Eigen::Vector3d::Zero(), a, {0, 0, 0}, {0, 0, 0}, dt);
  }
  const double t = dt * n;
  assert(rot_diff(pre.delta_R, Eigen::Matrix3d::Identity()) < 1e-12);
  assert(vclose(pre.delta_v, a * t, 1e-9));
  assert(vclose(pre.delta_p, 0.5 * a * t * t, 1e-9));
}

void test_residual_zero_for_consistent_trajectory()
{
  // Build a world trajectory consistent with constant world acceleration
  // a_world and identity attitude. The IMU measures specific force
  // accel_raw = a_world - g, so a preintegration of accel_raw plus the gravity
  // terms in computeResidual must reproduce the trajectory exactly (residual 0).
  const Eigen::Vector3d g(0, 0, -9.81);
  const Eigen::Vector3d a_world(0.3, -0.2, 0.1);
  const Eigen::Vector3d accel_raw = a_world - g;
  const double dt = 0.01;
  const int n = 100;
  ImuPreintegration pre;
  pre.params.gravity = g;
  pre.reset();
  for (int i = 0; i < n; ++i) {
    pre.integrate(Eigen::Vector3d::Zero(), accel_raw, {0, 0, 0}, {0, 0, 0}, dt);
  }
  const double t = dt * n;
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d p_i(2.0, -3.0, 1.0);
  const Eigen::Vector3d v_i(1.0, 0.5, -0.2);
  const Eigen::Vector3d v_j = v_i + a_world * t;
  const Eigen::Vector3d p_j = p_i + v_i * t + 0.5 * a_world * t * t;
  const Eigen::Matrix<double, 9, 1> r =
    pre.computeResidual(p_i, v_i, R, p_j, v_j, R);
  assert(r.norm() < 1e-6);
}

void test_bias_jacobian_matches_finite_difference()
{
  // The decisive test: the first-order bias correction (correctByBias, using the
  // accumulated d_*/d_b Jacobians) must match a full re-integration at the
  // perturbed bias, to within the O(delta^2) truncation error. This validates
  // d_dp_d_bg, d_dp_d_ba, d_dv_d_bg, d_dv_d_ba and d_dR_d_bg together.
  std::vector<ImuTick> ticks;
  for (int i = 0; i < 50; ++i) {
    const double s = 0.02 * i;
    ticks.push_back({
      Eigen::Vector3d(0.1 + 0.05 * std::sin(s), -0.2, 0.3 * std::cos(s)),
      Eigen::Vector3d(0.5, -1.0 + 0.2 * s, 9.0),
      0.01});
  }
  const Eigen::Vector3d bg0(0.01, -0.02, 0.015);
  const Eigen::Vector3d ba0(0.05, 0.03, -0.04);
  const Eigen::Vector3d dbg(1e-4, -2e-4, 1.5e-4);
  const Eigen::Vector3d dba(2e-4, 1e-4, -1e-4);

  const ImuPreintegration nominal = integrate_stream(ticks, bg0, ba0);
  const ImuPreintegration truth = integrate_stream(ticks, bg0 + dbg, ba0 + dba);

  ImuPreintegration corrected = nominal;
  corrected.correctByBias(dbg, dba);

  // First-order match: residual error must be far below the perturbation size
  // (which moved delta_p/delta_v by ~1e-3) -- i.e. the Jacobians explain it.
  assert(vclose(corrected.delta_p, truth.delta_p, 1e-6));
  assert(vclose(corrected.delta_v, truth.delta_v, 1e-6));
  assert(rot_diff(corrected.delta_R, truth.delta_R) < 1e-6);

  // Sanity: the perturbation actually moved the state well above the tolerance,
  // so the test is not trivially passing on a no-op.
  assert((nominal.delta_p - truth.delta_p).norm() > 1e-5);
}

void test_covariance_symmetric_and_grows()
{
  ImuPreintegration pre;
  pre.reset();
  auto step = [&]() {
    pre.integrate({0.1, -0.2, 0.05}, {0.4, -0.3, 9.2}, {0, 0, 0}, {0, 0, 0}, 0.01);
  };
  step();
  const double cov_after_1 = pre.covariance.trace();
  for (int i = 0; i < 49; ++i) step();
  const double cov_after_50 = pre.covariance.trace();

  // Symmetric.
  assert((pre.covariance - pre.covariance.transpose()).norm() < 1e-12);
  // Monotone growth of total uncertainty as more noisy steps accumulate.
  assert(cov_after_50 > cov_after_1);
  // Positive semidefinite: smallest eigenvalue is non-negative.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> es(pre.covariance);
  assert(es.eigenvalues().minCoeff() > -1e-12);
}

}  // namespace

int main()
{
  test_reset_is_identity();
  test_dt_outside_range_is_ignored();
  test_pure_rotation_matches_analytic();
  test_pure_translation_matches_analytic();
  test_residual_zero_for_consistent_trajectory();
  test_bias_jacobian_matches_finite_difference();
  test_covariance_symmetric_and_grows();
  return 0;
}
