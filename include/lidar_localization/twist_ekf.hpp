#ifndef TWIST_EKF_HPP_
#define TWIST_EKF_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <optional>

/**
 * Error-State EKF fusing twist (vx, wz) prediction with NDT measurement.
 *
 * State vector (9):
 *   [0-2] position (px, py, pz) in map frame
 *   [3-5] velocity (vx, vy, vz) in map frame
 *   [6]   yaw angle (rad)
 *   [7]   gyro_bias (wz bias, rad/s)
 *   [8]   speed_bias (vx bias, m/s)
 *
 * Prediction uses twist (body-frame vx, wz) at ~100 Hz.
 * Update uses NDT alignment result (px, py, pz, yaw) at ~10 Hz.
 */
class TwistEkf
{
public:
  static constexpr int STATE_DIM = 9;
  static constexpr int MEAS_DIM = 4;  // px, py, pz, yaw

  using StateVec = Eigen::Matrix<double, STATE_DIM, 1>;
  using StateCov = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
  using MeasVec = Eigen::Matrix<double, MEAS_DIM, 1>;
  using MeasCov = Eigen::Matrix<double, MEAS_DIM, MEAS_DIM>;
  using MeasJacobian = Eigen::Matrix<double, MEAS_DIM, STATE_DIM>;

  struct Params
  {
    // Process noise (per second, will be scaled by dt)
    double sigma_pos = 0.01;          // m/sqrt(s)
    double sigma_vel = 0.5;           // m/s/sqrt(s)
    double sigma_yaw = 0.01;          // rad/sqrt(s)
    double sigma_gyro_bias = 0.001;   // rad/s/sqrt(s), random walk
    double sigma_speed_bias = 0.01;   // m/s/sqrt(s), random walk

    // Measurement noise base (scaled by fitness)
    double sigma_ndt_pos = 0.1;       // m
    double sigma_ndt_yaw = 0.02;      // rad

    // Fitness-adaptive measurement noise
    double fitness_nominal = 1.0;     // fitness at which R = base
    double fitness_scale_factor = 2.0; // R *= (fitness/nominal)^scale
    double fitness_reject = 50.0;     // skip update if fitness > this
  };

  TwistEkf() { reset(); }

  void setParams(const Params & p) { params_ = p; }

  void reset()
  {
    x_.setZero();
    P_ = StateCov::Identity() * 100.0;  // large initial uncertainty
    initialized_ = false;
    last_predict_time_ = 0.0;
  }

  /**
   * Initialize state from a known pose (e.g., initial pose or first NDT result).
   */
  void initialize(double px, double py, double pz, double yaw, double stamp_sec)
  {
    x_.setZero();
    x_(0) = px;
    x_(1) = py;
    x_(2) = pz;
    x_(6) = yaw;

    P_ = StateCov::Identity() * 1.0;
    // Position: moderate uncertainty
    P_(0, 0) = P_(1, 1) = P_(2, 2) = 0.5 * 0.5;
    // Velocity: zero with moderate uncertainty
    P_(3, 3) = P_(4, 4) = P_(5, 5) = 0.1 * 0.1;
    // Yaw: small uncertainty (initialized from pose)
    P_(6, 6) = 0.05 * 0.05;
    // Biases: small initial uncertainty
    P_(7, 7) = 0.01 * 0.01;
    P_(8, 8) = 0.05 * 0.05;

    initialized_ = true;
    last_predict_time_ = stamp_sec;
  }

  bool isInitialized() const { return initialized_; }

  /**
   * Predict step using twist measurement (body-frame).
   * @param vx_body  forward velocity (m/s) from twist
   * @param wz       yaw rate (rad/s) from twist
   * @param stamp_sec current timestamp
   */
  void predict(double vx_body, double wz, double stamp_sec)
  {
    if (!initialized_) { return; }

    double dt = stamp_sec - last_predict_time_;
    if (dt <= 0.0 || dt > 1.0) {
      last_predict_time_ = stamp_sec;
      return;
    }
    last_predict_time_ = stamp_sec;

    // Bias-corrected inputs
    double wz_corr = wz - x_(7);
    double vx_corr = vx_body - x_(8);

    double yaw = x_(6);
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    // World-frame velocity from body-frame speed
    double vx_world = vx_corr * cos_yaw;
    double vy_world = vx_corr * sin_yaw;

    // State transition
    x_(0) += vx_world * dt;
    x_(1) += vy_world * dt;
    // x_(2) unchanged (no vertical twist info)
    x_(3) = vx_world;
    x_(4) = vy_world;
    // x_(5) = 0
    x_(6) += wz_corr * dt;
    // Normalize yaw to [-pi, pi]
    x_(6) = normalizeAngle(x_(6));
    // Biases: random walk, no change in prediction

    // Jacobian F = dF/dx
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> F =
      Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();

    // d(px)/d(yaw)
    F(0, 6) = -vx_corr * sin_yaw * dt;
    // d(py)/d(yaw)
    F(1, 6) = vx_corr * cos_yaw * dt;
    // d(px)/d(speed_bias) = -cos_yaw * dt
    F(0, 8) = -cos_yaw * dt;
    // d(py)/d(speed_bias) = -sin_yaw * dt
    F(1, 8) = -sin_yaw * dt;
    // d(vx_world)/d(yaw)
    F(3, 6) = -vx_corr * sin_yaw;
    F(4, 6) = vx_corr * cos_yaw;
    // d(vx_world)/d(speed_bias)
    F(3, 8) = -cos_yaw;
    F(4, 8) = -sin_yaw;
    // d(yaw)/d(gyro_bias) = -dt
    F(6, 7) = -dt;

    // Process noise Q
    StateCov Q = StateCov::Zero();
    Q(0, 0) = params_.sigma_pos * params_.sigma_pos * dt;
    Q(1, 1) = params_.sigma_pos * params_.sigma_pos * dt;
    Q(2, 2) = params_.sigma_pos * params_.sigma_pos * dt;
    Q(3, 3) = params_.sigma_vel * params_.sigma_vel * dt;
    Q(4, 4) = params_.sigma_vel * params_.sigma_vel * dt;
    Q(5, 5) = params_.sigma_vel * params_.sigma_vel * dt;
    Q(6, 6) = params_.sigma_yaw * params_.sigma_yaw * dt;
    Q(7, 7) = params_.sigma_gyro_bias * params_.sigma_gyro_bias * dt;
    Q(8, 8) = params_.sigma_speed_bias * params_.sigma_speed_bias * dt;

    P_ = F * P_ * F.transpose() + Q;
  }

  /**
   * Update step using NDT alignment result.
   * @param px, py, pz, yaw  NDT result in map frame
   * @param fitness_score     NDT fitness score (lower = better)
   * @return true if update was applied, false if rejected
   */
  bool update(double px, double py, double pz, double yaw_meas,
              double fitness_score)
  {
    if (!initialized_) { return false; }

    if (fitness_score > params_.fitness_reject) {
      return false;
    }

    // Adaptive measurement noise based on fitness
    double scale = std::pow(
      std::max(fitness_score, 0.1) / params_.fitness_nominal,
      params_.fitness_scale_factor);
    MeasCov R = MeasCov::Zero();
    R(0, 0) = R(1, 1) = R(2, 2) =
      params_.sigma_ndt_pos * params_.sigma_ndt_pos * scale;
    R(3, 3) = params_.sigma_ndt_yaw * params_.sigma_ndt_yaw * scale;

    // Measurement Jacobian H (linear: z = H*x)
    MeasJacobian H = MeasJacobian::Zero();
    H(0, 0) = 1.0;  // px
    H(1, 1) = 1.0;  // py
    H(2, 2) = 1.0;  // pz
    H(3, 6) = 1.0;  // yaw

    // Innovation
    MeasVec z;
    z << px, py, pz, yaw_meas;

    MeasVec z_pred;
    z_pred << x_(0), x_(1), x_(2), x_(6);

    MeasVec y = z - z_pred;
    y(3) = normalizeAngle(y(3));  // wrap yaw innovation

    // Innovation covariance
    Eigen::Matrix<double, MEAS_DIM, MEAS_DIM> S = H * P_ * H.transpose() + R;

    // Kalman gain
    Eigen::Matrix<double, STATE_DIM, MEAS_DIM> K = P_ * H.transpose() * S.inverse();

    // State update
    x_ += K * y;
    x_(6) = normalizeAngle(x_(6));

    // Covariance update (Joseph form for numerical stability)
    auto I = StateCov::Identity();
    auto IKH = I - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();

    return true;
  }

  // Accessors
  double px() const { return x_(0); }
  double py() const { return x_(1); }
  double pz() const { return x_(2); }
  double vx() const { return x_(3); }
  double vy() const { return x_(4); }
  double vz() const { return x_(5); }
  double yaw() const { return x_(6); }
  double gyroBias() const { return x_(7); }
  double speedBias() const { return x_(8); }

  const StateVec & state() const { return x_; }
  const StateCov & covariance() const { return P_; }

  /**
   * Get pose as 4x4 homogeneous matrix (map frame).
   * Roll and pitch are taken from the latest NDT result (not estimated by EKF).
   */
  Eigen::Matrix4f poseMatrix(double roll, double pitch) const
  {
    Eigen::Affine3f affine = Eigen::Affine3f::Identity();
    affine.translation() = Eigen::Vector3f(
      static_cast<float>(x_(0)),
      static_cast<float>(x_(1)),
      static_cast<float>(x_(2)));
    affine.linear() = (
      Eigen::AngleAxisf(static_cast<float>(x_(6)), Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
    ).toRotationMatrix();
    return affine.matrix();
  }

private:
  static double normalizeAngle(double a)
  {
    while (a > M_PI) { a -= 2.0 * M_PI; }
    while (a < -M_PI) { a += 2.0 * M_PI; }
    return a;
  }

  Params params_;
  StateVec x_;
  StateCov P_;
  bool initialized_{false};
  double last_predict_time_{0.0};
};

#endif  // TWIST_EKF_HPP_
