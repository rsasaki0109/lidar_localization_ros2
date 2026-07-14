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
#include "lidar_localization/imu_gtsam_smoother.hpp"

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

void test_one_step_covariance_matches_continuous_noise_density_units()
{
  ImuPreintegration pre;
  pre.params.accel_noise_density = 0.2;
  pre.params.gyro_noise_density = 0.03;
  const double dt = 0.01;
  pre.integrate(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt);

  const double accel_variance_density = 0.2 * 0.2;
  const double gyro_variance_density = 0.03 * 0.03;
  const double expected_dp_variance =
    0.25 * accel_variance_density * dt * dt * dt;
  const double expected_dv_variance = accel_variance_density * dt;
  const double expected_dtheta_variance = gyro_variance_density * dt;
  for (int axis = 0; axis < 3; ++axis) {
    assert(close(pre.covariance(axis, axis), expected_dp_variance, 1e-15));
    assert(close(pre.covariance(3 + axis, 3 + axis), expected_dv_variance, 1e-15));
    assert(close(pre.covariance(6 + axis, 6 + axis), expected_dtheta_variance, 1e-15));
  }
}

void test_bias_random_walk_increases_prior_variance_analytically()
{
  const double initial_sigma = 0.02;
  const double random_walk = 0.003;
  const double duration = 4.0;
  const double expected =
    initial_sigma * initial_sigma + random_walk * random_walk * duration;

  assert(close(
    imuBiasPriorVariance(initial_sigma, random_walk, duration), expected, 1e-15));
  assert(close(
    imuBiasPriorVariance(initial_sigma, 0.0, duration),
    initial_sigma * initial_sigma, 1e-15));
  assert(close(
    imuBiasPriorVariance(initial_sigma, random_walk, 0.0),
    initial_sigma * initial_sigma, 1e-15));
}

ImuGtsamSmoother estimate_constant_gyro_bias(double gyro_random_walk)
{
  ImuGtsamSmoother smoother;
  smoother.params_.imu_params.gravity.setZero();
  smoother.params_.imu_params.gyro_noise_density = 0.01;
  smoother.params_.bias_prior_sigma_gyro = 0.001;
  smoother.params_.imu_params.gyro_random_walk = gyro_random_walk;
  smoother.initialize(
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0);

  for (int interval = 1; interval <= 5; ++interval) {
    for (int tick = 0; tick < 20; ++tick) {
      smoother.integrateImu(
        Eigen::Vector3d(0.0, 0.0, 0.1), Eigen::Vector3d::Zero(), 0.005);
    }
    assert(smoother.update(
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.01, 0.1 * interval));
  }
  return smoother;
}

void test_bias_random_walk_changes_bias_estimate_in_smoother()
{
  const auto tightly_anchored = estimate_constant_gyro_bias(0.0);
  const auto random_walk_aware = estimate_constant_gyro_bias(0.5);
  const double true_bias = 0.1;

  const double anchored_error =
    std::abs(tightly_anchored.gyroBias().z() - true_bias);
  const double random_walk_error =
    std::abs(random_walk_aware.gyroBias().z() - true_bias);
  assert(random_walk_error < anchored_error);
  assert(random_walk_aware.gyroBias().z() > tightly_anchored.gyroBias().z());
}

void test_smoother_handles_short_and_regular_imu_windows()
{
  ImuGtsamSmoother smoother;
  smoother.params_.min_imu_factor_dt = 0.02;
  smoother.initialize(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  smoother.integrateImu(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.81), 0.004);
  assert(smoother.update(0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.004));
  assert(smoother.poseMatrix().allFinite());

  for (int i = 0; i < 20; ++i) {
    smoother.integrateImu(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.81), 0.005);
  }
  assert(smoother.update(0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.104));
  assert(smoother.poseMatrix().allFinite());
}

void test_dead_reckoning_position_uses_velocity_before_acceleration_update()
{
  ImuGtsamSmoother smoother;
  smoother.params_.imu_params.gravity.setZero();
  const Eigen::Vector3d initial_position(1.0, -2.0, 0.5);
  const Eigen::Vector3d initial_velocity(2.0, -1.0, 0.25);
  const Eigen::Vector3d acceleration(0.5, 1.0, -0.25);
  const double dt = 0.2;
  smoother.initialize(
    initial_position.x(), initial_position.y(), initial_position.z(),
    0.0, 0.0, 0.0,
    initial_velocity.x(), initial_velocity.y(), initial_velocity.z(), 0.0);

  smoother.integrateImu(Eigen::Vector3d::Zero(), acceleration, dt);

  const Eigen::Vector3d expected =
    initial_position + initial_velocity * dt + 0.5 * acceleration * dt * dt;
  const Eigen::Vector3d predicted =
    smoother.predictedPoseMatrix().block<3, 1>(0, 3).cast<double>();
  assert(vclose(predicted, expected, 1e-6));
}

void test_smoother_keeps_window_and_velocity_across_updates()
{
  ImuGtsamSmoother smoother;
  smoother.params_.imu_params.gravity.setZero();
  smoother.initialize(
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0);

  for (int interval = 1; interval <= 2; ++interval) {
    for (int tick = 0; tick < 20; ++tick) {
      smoother.integrateImu(
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.005);
    }
    assert(smoother.update(
      0.1 * interval, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.1, 0.1 * interval));
  }

  assert(smoother.poseCount() == 3);
  assert(std::abs(smoother.velocity().x() - 1.0) < 1e-3);
  assert(std::abs(smoother.velocity().y()) < 1e-6);
  assert(std::abs(smoother.velocity().z()) < 1e-6);
}

void test_pose_only_gap_update_preserves_velocity_and_bias()
{
  ImuGtsamSmoother smoother;
  smoother.params_.imu_params.gravity.setZero();
  const Eigen::Vector3d velocity(1.5, -0.25, 0.1);
  const Eigen::Vector3d gyro_bias(0.01, -0.02, 0.03);
  const Eigen::Vector3d accel_bias(0.1, -0.2, 0.05);
  smoother.initializeState(
    Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(), velocity,
    gyro_bias, accel_bias, 0.0);

  // An uncovered scan gap contributes no IMU factor, but its accepted LiDAR
  // pose must not force a zero-velocity/zero-bias reinitialization.
  smoother.resetPendingIntegration();
  assert(smoother.updatePoseOnly(
    1.5, -0.25, 0.1,
    0.0, 0.0, 0.0, 0.1, 1.0));

  assert(smoother.poseCount() == 2);
  assert(vclose(smoother.velocity(), velocity, 1e-9));
  assert(vclose(smoother.gyroBias(), gyro_bias, 1e-9));
  assert(vclose(smoother.accelBias(), accel_bias, 1e-9));
}

void test_smoother_estimates_nonzero_initial_velocity_from_pose_updates()
{
  ImuGtsamSmoother smoother;
  smoother.params_.imu_params.gravity.setZero();
  smoother.initialize(
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0);

  for (int interval = 1; interval <= 3; ++interval) {
    for (int tick = 0; tick < 20; ++tick) {
      smoother.integrateImu(
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.005);
    }
    assert(smoother.update(
      0.1 * interval, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.1, 0.1 * interval));
  }

  assert(smoother.poseCount() == 4);
  assert(std::abs(smoother.velocity().x() - 1.0) < 1e-2);
  assert(std::abs(smoother.velocity().y()) < 1e-6);
  assert(std::abs(smoother.velocity().z()) < 1e-6);
}

void test_smoother_state_transfer_preserves_velocity_and_bias_for_repropagation()
{
  ImuGtsamSmoother predictor;
  predictor.params_.imu_params.gravity.setZero();
  const Eigen::Vector3d position(1.0, 2.0, 3.0);
  const Eigen::Vector3d velocity(2.0, -1.0, 0.5);
  const Eigen::Vector3d gyro_bias(0.0, 0.0, 0.1);
  const Eigen::Vector3d accel_bias(0.2, 0.0, 0.0);
  predictor.initializeState(
    position, Eigen::Matrix3d::Identity(), velocity,
    gyro_bias, accel_bias, 10.0);

  predictor.integrateImu(gyro_bias, accel_bias, 0.1);

  const Eigen::Vector3d predicted_position =
    predictor.predictedPoseMatrix().block<3, 1>(0, 3).cast<double>();
  assert(vclose(predicted_position, position + velocity * 0.1, 1e-6));
  assert(rot_diff(
    predictor.predictedPoseMatrix().block<3, 3>(0, 0).cast<double>(),
    Eigen::Matrix3d::Identity()) < 1e-6);
  assert(vclose(predictor.gyroBias(), gyro_bias, 1e-12));
  assert(vclose(predictor.accelBias(), accel_bias, 1e-12));
}

void test_reset_pending_integration_prevents_duplicate_repropagation()
{
  ImuGtsamSmoother predictor;
  predictor.params_.imu_params.gravity.setZero();
  predictor.initialize(
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0);

  predictor.integrateImu(
    Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, 0.0), 0.1);
  predictor.resetPendingIntegration();
  predictor.integrateImu(
    Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, 0.0), 0.1);

  const Eigen::Vector3d predicted_position =
    predictor.predictedPoseMatrix().block<3, 1>(0, 3).cast<double>();
  assert(vclose(predicted_position, Eigen::Vector3d(0.105, 0.0, 0.0), 1e-6));
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
  test_one_step_covariance_matches_continuous_noise_density_units();
  test_bias_random_walk_increases_prior_variance_analytically();
  test_bias_random_walk_changes_bias_estimate_in_smoother();
  test_smoother_handles_short_and_regular_imu_windows();
  test_dead_reckoning_position_uses_velocity_before_acceleration_update();
  test_smoother_keeps_window_and_velocity_across_updates();
  test_pose_only_gap_update_preserves_velocity_and_bias();
  test_smoother_estimates_nonzero_initial_velocity_from_pose_updates();
  test_smoother_state_transfer_preserves_velocity_and_bias_for_repropagation();
  test_reset_pending_integration_prevents_duplicate_repropagation();
  return 0;
}
