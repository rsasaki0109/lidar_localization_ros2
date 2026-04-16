#ifndef IMU_PREINTEGRATION_HPP_
#define IMU_PREINTEGRATION_HPP_

#include "lidar_localization/so3_utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>

struct ImuSample
{
  double stamp;
  Eigen::Vector3d gyro;   // angular velocity (rad/s) in body frame
  Eigen::Vector3d accel;  // linear acceleration (m/s^2) in body frame
};

struct ImuPreintegrationParams
{
  double gyro_noise_density = 0.01;     // rad/s/sqrt(Hz)
  double accel_noise_density = 0.1;     // m/s^2/sqrt(Hz)
  double gyro_random_walk = 0.0001;     // rad/s^2/sqrt(Hz)
  double accel_random_walk = 0.001;     // m/s^3/sqrt(Hz)
  Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.81);
};

/// On-manifold IMU preintegration (Forster et al. 2017), pure Eigen
class ImuPreintegration
{
public:
  using Matrix9d = Eigen::Matrix<double, 9, 9>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Matrix93 = Eigen::Matrix<double, 9, 3>;

  ImuPreintegrationParams params;

  // Preintegrated measurements (relative to start frame)
  Eigen::Vector3d delta_p = Eigen::Vector3d::Zero();
  Eigen::Vector3d delta_v = Eigen::Vector3d::Zero();
  Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity();
  double dt_sum = 0.0;

  // Covariance of preintegrated measurements [dp, dv, dtheta] (9x9)
  Matrix9d covariance = Matrix9d::Zero();

  // Jacobians w.r.t. bias for first-order correction
  Eigen::Matrix3d d_dp_d_bg = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d d_dp_d_ba = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d d_dv_d_bg = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d d_dv_d_ba = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d d_dR_d_bg = Eigen::Matrix3d::Zero();

  void reset()
  {
    delta_p.setZero();
    delta_v.setZero();
    delta_R.setIdentity();
    dt_sum = 0.0;
    covariance.setZero();
    d_dp_d_bg.setZero();
    d_dp_d_ba.setZero();
    d_dv_d_bg.setZero();
    d_dv_d_ba.setZero();
    d_dR_d_bg.setZero();
  }

  /// Integrate one IMU measurement (bias-corrected)
  void integrate(
    const Eigen::Vector3d & gyro_raw, const Eigen::Vector3d & accel_raw,
    const Eigen::Vector3d & bg, const Eigen::Vector3d & ba, double dt)
  {
    if (dt <= 0.0 || dt > 0.5) return;

    Eigen::Vector3d omega = gyro_raw - bg;
    Eigen::Vector3d acc = accel_raw - ba;

    // Rotation increment
    Eigen::Vector3d omega_dt = omega * dt;
    Eigen::Matrix3d dR = so3::Exp(omega_dt);
    Eigen::Matrix3d Jr_dt = so3::Jr(omega_dt);

    // Rotated acceleration in preintegration frame
    Eigen::Vector3d acc_rotated = delta_R * acc;

    // Update preintegrated measurements
    delta_p += delta_v * dt + 0.5 * acc_rotated * dt * dt;
    delta_v += acc_rotated * dt;
    delta_R = delta_R * dR;

    // --- Covariance propagation ---
    // State: [dp, dv, dtheta]
    Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
    // dp row
    A.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    A.block<3, 3>(0, 6) = -0.5 * delta_R * so3::skew(acc) * dt * dt;
    // dv row
    A.block<3, 3>(3, 6) = -delta_R * so3::skew(acc) * dt;
    // dtheta row
    A.block<3, 3>(6, 6) = dR.transpose();

    Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();
    // Noise input: [accel_noise, gyro_noise]
    B.block<3, 3>(0, 0) = 0.5 * delta_R * dt * dt;   // dp <- accel noise
    B.block<3, 3>(3, 0) = delta_R * dt;               // dv <- accel noise
    B.block<3, 3>(6, 3) = Jr_dt * dt;                 // dtheta <- gyro noise

    // Continuous noise (per-axis)
    Matrix6d Qc = Matrix6d::Zero();
    double na2 = params.accel_noise_density * params.accel_noise_density;
    double ng2 = params.gyro_noise_density * params.gyro_noise_density;
    Qc.block<3, 3>(0, 0) = na2 * Eigen::Matrix3d::Identity();
    Qc.block<3, 3>(3, 3) = ng2 * Eigen::Matrix3d::Identity();

    covariance = A * covariance * A.transpose() + B * Qc * B.transpose();

    // --- Bias Jacobians ---
    // d_dp_d_ba
    d_dp_d_ba += d_dv_d_ba * dt - 0.5 * delta_R * dt * dt;
    // d_dp_d_bg
    d_dp_d_bg += d_dv_d_bg * dt - 0.5 * delta_R * so3::skew(acc) * d_dR_d_bg * dt * dt;
    // d_dv_d_ba
    d_dv_d_ba += -delta_R * dt;
    // d_dv_d_bg
    d_dv_d_bg += -delta_R * so3::skew(acc) * d_dR_d_bg * dt;
    // d_dR_d_bg (note: update AFTER using it above)
    d_dR_d_bg = dR.transpose() * d_dR_d_bg - Jr_dt * dt;

    dt_sum += dt;
  }

  /// First-order bias correction when biases change
  void correctByBias(const Eigen::Vector3d & delta_bg, const Eigen::Vector3d & delta_ba)
  {
    delta_p += d_dp_d_bg * delta_bg + d_dp_d_ba * delta_ba;
    delta_v += d_dv_d_bg * delta_bg + d_dv_d_ba * delta_ba;
    delta_R = delta_R * so3::Exp(d_dR_d_bg * delta_bg);
  }

  /// Compute residual given two poses, velocities, biases, and gravity
  /// r = [r_dp(3), r_dv(3), r_dtheta(3)]
  Eigen::Matrix<double, 9, 1> computeResidual(
    const Eigen::Vector3d & p_i, const Eigen::Vector3d & v_i, const Eigen::Matrix3d & R_i,
    const Eigen::Vector3d & p_j, const Eigen::Vector3d & v_j, const Eigen::Matrix3d & R_j) const
  {
    const Eigen::Vector3d & g = params.gravity;
    double dt = dt_sum;

    Eigen::Matrix<double, 9, 1> r;
    // Position residual
    r.segment<3>(0) = R_i.transpose() * (p_j - p_i - v_i * dt - 0.5 * g * dt * dt) - delta_p;
    // Velocity residual
    r.segment<3>(3) = R_i.transpose() * (v_j - v_i - g * dt) - delta_v;
    // Rotation residual
    r.segment<3>(6) = so3::Log(delta_R.transpose() * R_i.transpose() * R_j);

    return r;
  }
};

#endif  // IMU_PREINTEGRATION_HPP_
