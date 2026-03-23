#ifndef IMU_GTSAM_SMOOTHER_HPP_
#define IMU_GTSAM_SMOOTHER_HPP_

#include "lidar_localization/imu_preintegration.hpp"
#include "lidar_localization/so3_utils.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <iostream>

/// Sliding-window Gauss-Newton optimizer with IMU preintegration and 6DOF NDT priors.
/// State per pose: [x, y, z, vx, vy, vz, roll, pitch, yaw] (9 DOF)
/// Shared: [bg_x, bg_y, bg_z, ba_x, ba_y, ba_z] (6 DOF)
class ImuGtsamSmoother
{
public:
  static constexpr int POSE_DIM = 9;
  static constexpr int BIAS_DIM = 6;

  struct Params
  {
    // NDT measurement noise (6DOF)
    double ndt_sigma_x = 0.1;
    double ndt_sigma_y = 0.1;
    double ndt_sigma_z = 0.1;
    double ndt_sigma_roll = 0.05;
    double ndt_sigma_pitch = 0.05;
    double ndt_sigma_yaw = 0.02;
    double fitness_nominal = 1.0;
    double fitness_scale_factor = 2.0;
    double fitness_reject = 50.0;
    double huber_k = 1.345;
    int window_size = 50;

    // Bias prior sigma
    double bias_prior_sigma_gyro = 0.01;
    double bias_prior_sigma_accel = 0.1;

    // IMU preintegration params
    ImuPreintegrationParams imu_params;
  };

  Params params_;

  // IMU biases (shared across window)
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();

  bool isInitialized() const { return initialized_; }

  void initialize(
    double x, double y, double z,
    double roll, double pitch, double yaw,
    double vx, double vy, double vz,
    double stamp_sec)
  {
    PoseEntry entry;
    entry.position = Eigen::Vector3d(x, y, z);
    entry.velocity = Eigen::Vector3d(vx, vy, vz);
    entry.R = so3::eulerToRotation(roll, pitch, yaw);
    entry.has_imu_factor = false;
    entry.has_ndt = false;
    poses_.clear();
    poses_.push_back(entry);
    last_stamp_ = stamp_sec;
    initialized_ = true;
    // Reset IMU accumulation
    current_preint_.reset();
    current_preint_.params = params_.imu_params;
  }

  /// Accumulate one IMU sample (called at IMU rate ~200Hz)
  void integrateImu(
    const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel, double dt)
  {
    if (!initialized_) return;
    current_preint_.integrate(gyro, accel, gyro_bias_, accel_bias_, dt);
    // Dead-reckoning for init_guess
    Eigen::Vector3d acc_world = dr_R_ * (accel - accel_bias_) + params_.imu_params.gravity;
    dr_v_ += acc_world * dt;
    dr_p_ += dr_v_ * dt + 0.5 * acc_world * dt * dt;
    Eigen::Vector3d omega = gyro - gyro_bias_;
    dr_R_ = dr_R_ * so3::Exp(omega * dt);
  }

  /// Update with GICP/NDT measurement at scan time
  bool update(
    double px, double py, double pz,
    double roll, double pitch, double yaw,
    double fitness_score, double stamp_sec)
  {
    if (!initialized_) return false;

    // Reject bad fitness
    if (fitness_score > params_.fitness_reject) return false;

    // Create new pose entry
    PoseEntry entry;
    // Initialize from dead-reckoning
    entry.position = dr_p_;
    entry.velocity = dr_v_;
    entry.R = dr_R_;

    // Store IMU preintegration as between-factor
    entry.preint = current_preint_;
    entry.has_imu_factor = (current_preint_.dt_sum > 0.001);

    // Store NDT measurement
    entry.has_ndt = true;
    entry.ndt_position = Eigen::Vector3d(px, py, pz);
    entry.ndt_R = so3::eulerToRotation(roll, pitch, yaw);
    entry.ndt_fitness = fitness_score;

    poses_.push_back(entry);

    // Trim window
    while (static_cast<int>(poses_.size()) > params_.window_size) {
      poses_.pop_front();
    }

    // Optimize
    optimize();

    // Reset dead-reckoning from latest optimized pose
    const auto & latest = poses_.back();
    dr_p_ = latest.position;
    dr_v_ = latest.velocity;
    dr_R_ = latest.R;

    // Reset preintegration for next interval
    current_preint_.reset();
    current_preint_.params = params_.imu_params;

    last_stamp_ = stamp_sec;
    return true;
  }

  // Accessors
  double px() const { return poses_.back().position.x(); }
  double py() const { return poses_.back().position.y(); }
  double pz() const { return poses_.back().position.z(); }

  /// Get latest optimized pose as 4x4 matrix
  Eigen::Matrix4f poseMatrix() const
  {
    const auto & latest = poses_.back();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = latest.R;
    T.block<3, 1>(0, 3) = latest.position;
    return T.cast<float>();
  }

  /// Get dead-reckoned predicted pose (for GICP/NDT init_guess)
  Eigen::Matrix4f predictedPoseMatrix() const
  {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = dr_R_;
    T.block<3, 1>(0, 3) = dr_p_;
    return T.cast<float>();
  }

private:
  struct PoseEntry
  {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    // IMU between-factor to this pose from previous
    ImuPreintegration preint;
    bool has_imu_factor = false;

    // NDT measurement
    bool has_ndt = false;
    Eigen::Vector3d ndt_position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d ndt_R = Eigen::Matrix3d::Identity();
    double ndt_fitness = 0.0;
  };

  std::deque<PoseEntry> poses_;
  bool initialized_ = false;
  double last_stamp_ = 0.0;

  // Current preintegration being accumulated
  ImuPreintegration current_preint_;

  // Dead-reckoning state for init_guess prediction
  Eigen::Vector3d dr_p_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dr_v_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d dr_R_ = Eigen::Matrix3d::Identity();

  void optimize()
  {
    const int n = static_cast<int>(poses_.size());
    if (n < 2) return;

    const int total_dim = POSE_DIM * n + BIAS_DIM;
    const int bias_offset = POSE_DIM * n;

    for (int iter = 0; iter < 10; ++iter) {
      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(total_dim, total_dim);
      Eigen::VectorXd b = Eigen::VectorXd::Zero(total_dim);

      // --- 1. Anchor prior on first pose ---
      double anchor_w = 1e6;
      for (int k = 0; k < POSE_DIM; ++k) {
        H(k, k) += anchor_w;
      }

      // --- 2. IMU between-factors ---
      for (int i = 1; i < n; ++i) {
        if (!poses_[i].has_imu_factor) continue;

        const auto & pi = poses_[i - 1];
        const auto & pj = poses_[i];
        const auto & preint = pj.preint;

        // Compute residual
        Eigen::Matrix<double, 9, 1> r = preint.computeResidual(
          pi.position, pi.velocity, pi.R,
          pj.position, pj.velocity, pj.R);

        // Information from preintegration covariance
        Eigen::Matrix<double, 9, 9> info =
          preint.covariance.inverse().eval();
        // Symmetrize
        info = 0.5 * (info + info.transpose());

        // Jacobians w.r.t. pose_i [p, v, theta] and pose_j [p, v, theta]
        double dt = preint.dt_sum;
        const Eigen::Vector3d & g = params_.imu_params.gravity;
        Eigen::Matrix3d Ri_T = pi.R.transpose();

        // Full Jacobian blocks (9x9 for pose_i, 9x9 for pose_j)
        Eigen::Matrix<double, 9, 9> Ji = Eigen::Matrix<double, 9, 9>::Zero();
        Eigen::Matrix<double, 9, 9> Jj = Eigen::Matrix<double, 9, 9>::Zero();

        Eigen::Vector3d dp_world = pj.position - pi.position - pi.velocity * dt - 0.5 * g * dt * dt;
        Eigen::Vector3d dv_world = pj.velocity - pi.velocity - g * dt;
        Eigen::Vector3d r_theta = r.segment<3>(6);

        // dr/d(pose_i)
        Ji.block<3, 3>(0, 0) = -Ri_T;                          // dr_p/dp_i
        Ji.block<3, 3>(0, 3) = -Ri_T * dt;                     // dr_p/dv_i
        Ji.block<3, 3>(0, 6) = so3::skew(Ri_T * dp_world);     // dr_p/dtheta_i
        Ji.block<3, 3>(3, 3) = -Ri_T;                          // dr_v/dv_i
        Ji.block<3, 3>(3, 6) = so3::skew(Ri_T * dv_world);     // dr_v/dtheta_i
        Eigen::Matrix3d Jr_inv_r = so3::Jr_inv(r_theta);
        Ji.block<3, 3>(6, 6) = -Jr_inv_r * pj.R.transpose() * pi.R;  // dr_theta/dtheta_i

        // dr/d(pose_j)
        Jj.block<3, 3>(0, 0) = Ri_T;                           // dr_p/dp_j
        Jj.block<3, 3>(3, 3) = Ri_T;                           // dr_v/dv_j
        Jj.block<3, 3>(6, 6) = Jr_inv_r;                       // dr_theta/dtheta_j

        // dr/d(biases) [9x3 for bg, 9x3 for ba]
        Eigen::Matrix<double, 9, 3> Jbg = Eigen::Matrix<double, 9, 3>::Zero();
        Eigen::Matrix<double, 9, 3> Jba = Eigen::Matrix<double, 9, 3>::Zero();
        Jbg.block<3, 3>(0, 0) = -preint.d_dp_d_bg;
        Jbg.block<3, 3>(3, 0) = -preint.d_dv_d_bg;
        Jbg.block<3, 3>(6, 0) = -Jr_inv_r * so3::Exp(r_theta).transpose() *
                                  so3::Jr(preint.d_dR_d_bg * (gyro_bias_ - gyro_bias_)) *
                                  preint.d_dR_d_bg;
        // Simplified: for small bias changes, this ≈ -Jr_inv_r * preint.d_dR_d_bg
        Jbg.block<3, 3>(6, 0) = -Jr_inv_r * preint.d_dR_d_bg;

        Jba.block<3, 3>(0, 0) = -preint.d_dp_d_ba;
        Jba.block<3, 3>(3, 0) = -preint.d_dv_d_ba;

        int idx_i = (i - 1) * POSE_DIM;
        int idx_j = i * POSE_DIM;

        // Accumulate H and b
        // H += J^T * info * J
        H.block<9, 9>(idx_i, idx_i) += Ji.transpose() * info * Ji;
        H.block<9, 9>(idx_i, idx_j) += Ji.transpose() * info * Jj;
        H.block<9, 9>(idx_j, idx_i) += Jj.transpose() * info * Ji;
        H.block<9, 9>(idx_j, idx_j) += Jj.transpose() * info * Jj;

        // Bias blocks
        Eigen::Matrix<double, 9, 6> Jb;
        Jb.block<9, 3>(0, 0) = Jbg;
        Jb.block<9, 3>(0, 3) = Jba;

        H.block(idx_i, bias_offset, 9, 6) += Ji.transpose() * info * Jb;
        H.block(bias_offset, idx_i, 6, 9) += Jb.transpose() * info * Ji;
        H.block(idx_j, bias_offset, 9, 6) += Jj.transpose() * info * Jb;
        H.block(bias_offset, idx_j, 6, 9) += Jb.transpose() * info * Jj;
        H.block<6, 6>(bias_offset, bias_offset) += Jb.transpose() * info * Jb;

        // b += J^T * info * r
        b.segment<9>(idx_i) += Ji.transpose() * info * r;
        b.segment<9>(idx_j) += Jj.transpose() * info * r;
        b.segment<6>(bias_offset) += Jb.transpose() * info * r;
      }

      // --- 3. NDT prior factors (6DOF, Huber-weighted) ---
      for (int i = 0; i < n; ++i) {
        if (!poses_[i].has_ndt) continue;
        const auto & pose = poses_[i];

        // Residual: [dp(3), dtheta(3)]
        Eigen::Matrix<double, 6, 1> r_ndt;
        r_ndt.segment<3>(0) = pose.position - pose.ndt_position;
        r_ndt.segment<3>(3) = so3::Log(pose.ndt_R.transpose() * pose.R);

        // Fitness-adaptive noise
        double scale = std::pow(pose.ndt_fitness / params_.fitness_nominal, params_.fitness_scale_factor);
        scale = std::max(scale, 1.0);
        double sqrt_scale = std::sqrt(scale);

        Eigen::Matrix<double, 6, 1> sigmas;
        sigmas << params_.ndt_sigma_x * sqrt_scale,
                  params_.ndt_sigma_y * sqrt_scale,
                  params_.ndt_sigma_z * sqrt_scale,
                  params_.ndt_sigma_roll * sqrt_scale,
                  params_.ndt_sigma_pitch * sqrt_scale,
                  params_.ndt_sigma_yaw * sqrt_scale;

        Eigen::Matrix<double, 6, 6> info_ndt = Eigen::Matrix<double, 6, 6>::Zero();
        for (int k = 0; k < 6; ++k) {
          info_ndt(k, k) = 1.0 / (sigmas(k) * sigmas(k));
        }

        // Huber weighting
        double mahal = std::sqrt(r_ndt.transpose() * info_ndt * r_ndt);
        double w = 1.0;
        if (mahal > params_.huber_k) {
          w = params_.huber_k / mahal;
        }

        // Jacobian: identity for position (rows 0-2), Jr_inv for rotation (rows 3-5)
        // w.r.t. pose state [p, v, theta]: only affects p(0-2) and theta(6-8)
        Eigen::Matrix<double, 6, 9> J_ndt = Eigen::Matrix<double, 6, 9>::Zero();
        J_ndt.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_ndt.block<3, 3>(3, 6) = so3::Jr_inv(r_ndt.segment<3>(3));

        int idx = i * POSE_DIM;
        Eigen::Matrix<double, 6, 6> w_info = w * info_ndt;
        H.block<9, 9>(idx, idx) += J_ndt.transpose() * w_info * J_ndt;
        b.segment<9>(idx) += J_ndt.transpose() * w_info * r_ndt;
      }

      // --- 4. Bias prior factor ---
      {
        double info_bg = 1.0 / (params_.bias_prior_sigma_gyro * params_.bias_prior_sigma_gyro);
        double info_ba = 1.0 / (params_.bias_prior_sigma_accel * params_.bias_prior_sigma_accel);
        for (int k = 0; k < 3; ++k) {
          H(bias_offset + k, bias_offset + k) += info_bg;
          b(bias_offset + k) += info_bg * gyro_bias_(k);  // prior at current estimate
        }
        for (int k = 0; k < 3; ++k) {
          H(bias_offset + 3 + k, bias_offset + 3 + k) += info_ba;
          b(bias_offset + 3 + k) += info_ba * accel_bias_(k);
        }
      }

      // --- Solve ---
      Eigen::VectorXd delta = H.ldlt().solve(-b);

      // Apply pose updates
      for (int i = 0; i < n; ++i) {
        int idx = i * POSE_DIM;
        poses_[i].position += delta.segment<3>(idx);
        poses_[i].velocity += delta.segment<3>(idx + 3);
        // Rotation update on manifold
        Eigen::Vector3d dtheta = delta.segment<3>(idx + 6);
        poses_[i].R = poses_[i].R * so3::Exp(dtheta);
      }
      // Update biases
      gyro_bias_ += delta.segment<3>(bias_offset);
      accel_bias_ += delta.segment<3>(bias_offset + 3);

      if (delta.squaredNorm() < 1e-10) break;
    }
  }
};

#endif  // IMU_GTSAM_SMOOTHER_HPP_
