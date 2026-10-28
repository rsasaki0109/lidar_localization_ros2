#ifndef TWIST_EKF_HPP_
#define TWIST_EKF_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
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
 * Update uses NDT alignment result (px, py, pz, yaw) at ~10 Hz. The result is
 * the pose at the scan stamp, while the state has moved on with the twist
 * received since: the update moves the measurement to the state's time along
 * the EKF's own trajectory (kept in history_).
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
    history_.clear();
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
    history_.clear();
    recordHistory(stamp_sec);
  }

  bool isInitialized() const { return initialized_; }

  /**
   * Predict step using twist measurement (body-frame).
   * @param vx_body  forward velocity (m/s) from twist
   * @param wz       body yaw rate (rad/s) from twist
   * @param stamp_sec current timestamp
   * @param roll, pitch  body attitude (rad), e.g. from the latest NDT result.
   *   On a slope the forward speed splits into horizontal and vertical motion
   *   and the body yaw rate turns the heading at cos(roll) * wz / cos(pitch).
   *   With both zero this is the planar model.
   */
  void predict(double vx_body, double wz, double stamp_sec,
               double roll = 0.0, double pitch = 0.0)
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
    // Body x axis in the map frame is (cos_pitch * cos_yaw, cos_pitch * sin_yaw, -sin_pitch).
    double cos_pitch = std::cos(pitch);
    double sin_pitch = std::sin(pitch);
    // ZYX Euler rate: d(yaw)/dt = cos(roll) * wz / cos(pitch) when wx = wy = 0.
    double yaw_rate_scale =
      std::cos(roll) / std::copysign(std::max(std::abs(cos_pitch), 1e-3), cos_pitch);

    // World-frame velocity from body-frame speed
    double vx_world = vx_corr * cos_pitch * cos_yaw;
    double vy_world = vx_corr * cos_pitch * sin_yaw;
    double vz_world = -vx_corr * sin_pitch;

    // State transition
    x_(0) += vx_world * dt;
    x_(1) += vy_world * dt;
    x_(2) += vz_world * dt;
    x_(3) = vx_world;
    x_(4) = vy_world;
    x_(5) = vz_world;
    x_(6) += yaw_rate_scale * wz_corr * dt;
    // Normalize yaw to [-pi, pi]
    x_(6) = normalizeAngle(x_(6));
    // Biases: random walk, no change in prediction

    // Jacobian F = dF/dx
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> F =
      Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();

    // d(px)/d(yaw)
    F(0, 6) = -vx_corr * cos_pitch * sin_yaw * dt;
    // d(py)/d(yaw)
    F(1, 6) = vx_corr * cos_pitch * cos_yaw * dt;
    // d(p)/d(speed_bias) = -(body x axis) * dt
    F(0, 8) = -cos_pitch * cos_yaw * dt;
    F(1, 8) = -cos_pitch * sin_yaw * dt;
    F(2, 8) = sin_pitch * dt;
    // d(v_world)/d(yaw)
    F(3, 6) = -vx_corr * cos_pitch * sin_yaw;
    F(4, 6) = vx_corr * cos_pitch * cos_yaw;
    // d(v_world)/d(speed_bias)
    F(3, 8) = -cos_pitch * cos_yaw;
    F(4, 8) = -cos_pitch * sin_yaw;
    F(5, 8) = sin_pitch;
    // d(yaw)/d(gyro_bias)
    F(6, 7) = -yaw_rate_scale * dt;

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
    recordHistory(stamp_sec);
  }

  /**
   * Update step using NDT alignment result.
   * @param px, py, pz, yaw  NDT result in map frame
   * @param fitness_score     NDT fitness score (lower = better)
   * @param stamp_sec         scan stamp of the result; without it the result is
   *                          taken as the pose at the latest prediction
   * @return true if update was applied, false if rejected
   */
  bool update(double px, double py, double pz, double yaw_meas,
              double fitness_score, std::optional<double> stamp_sec = std::nullopt)
  {
    if (!initialized_) { return false; }

    if (fitness_score > params_.fitness_reject) {
      return false;
    }

    if (stamp_sec) {
      // Carry the measurement over the motion the EKF predicted since the scan,
      // turned by the heading the measurement gives the scan pose.
      const PoseSample & at_scan = sampleAt(*stamp_sec);
      const double turn = normalizeAngle(yaw_meas - at_scan.yaw);
      const double dx = x_(0) - at_scan.x;
      const double dy = x_(1) - at_scan.y;
      px += std::cos(turn) * dx - std::sin(turn) * dy;
      py += std::sin(turn) * dx + std::cos(turn) * dy;
      pz += x_(2) - at_scan.z;
      yaw_meas = normalizeAngle(yaw_meas + normalizeAngle(x_(6) - at_scan.yaw));
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
    const StateVec correction = K * y;
    x_ += correction;
    x_(6) = normalizeAngle(x_(6));
    // The trajectory the next measurement is carried along includes this correction.
    for (auto & sample : history_) {
      sample.x += correction(0);
      sample.y += correction(1);
      sample.z += correction(2);
      sample.yaw = normalizeAngle(sample.yaw + correction(6));
    }

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
    return buildMatrix(x_(0), x_(1), x_(2), x_(6), roll, pitch);
  }

  /**
   * Pose at a scan stamp: the latest predicted state at or before it (for the
   * NDT seed, and for the pose published with that scan's stamp).
   */
  Eigen::Matrix4f poseMatrixAt(double roll, double pitch, double stamp_sec) const
  {
    if (history_.empty()) {return poseMatrix(roll, pitch);}
    const PoseSample & sample = sampleAt(stamp_sec);
    return buildMatrix(sample.x, sample.y, sample.z, sample.yaw, roll, pitch);
  }

private:
  // The state after a prediction or initialization.
  struct PoseSample
  {
    double stamp;
    double x;
    double y;
    double z;
    double yaw;
  };

  // 10 s at 100 Hz: longer than a scan waits to be aligned.
  static constexpr std::size_t kMaxHistory = 1000;

  void recordHistory(double stamp_sec)
  {
    history_.push_back(PoseSample{stamp_sec, x_(0), x_(1), x_(2), x_(6)});
    while (history_.size() > kMaxHistory) {
      history_.pop_front();
    }
  }

  // The latest sample at or before the stamp, else the oldest one.
  const PoseSample & sampleAt(double stamp_sec) const
  {
    for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
      if (it->stamp <= stamp_sec) {return *it;}
    }
    return history_.front();
  }

  static Eigen::Matrix4f buildMatrix(
    double x, double y, double z, double yaw, double roll, double pitch)
  {
    Eigen::Affine3f affine = Eigen::Affine3f::Identity();
    affine.translation() = Eigen::Vector3f(
      static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    affine.linear() = (
      Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(static_cast<float>(pitch), Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(static_cast<float>(roll), Eigen::Vector3f::UnitX())
    ).toRotationMatrix();
    return affine.matrix();
  }

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
  std::deque<PoseSample> history_;
};

#endif  // TWIST_EKF_HPP_
