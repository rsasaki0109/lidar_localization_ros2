#ifndef IMU_GTSAM_SMOOTHER_HPP_
#define IMU_GTSAM_SMOOTHER_HPP_

#include "lidar_localization/imu_preintegration.hpp"
#include "lidar_localization/so3_utils.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <vector>
#include <iostream>

/// Sliding-window Gauss-Newton optimizer with IMU preintegration and 6DOF NDT priors.
/// State per pose: [x, y, z, vx, vy, vz, roll, pitch, yaw] (9 DOF)
/// Shared: [bg_x, bg_y, bg_z, ba_x, ba_y, ba_z] (6 DOF)
class ImuGtsamSmoother
{
public:
  static constexpr int POSE_DIM = 9;
  static constexpr int BIAS_DIM = 6;

  // --- Configuration ---

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
    double min_imu_factor_dt = 0.02;
    double imu_factor_covariance_regularization = 1e-9;

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

  // --- Initialization and IMU integration ---

  bool isInitialized() const { return initialized_; }

  void initialize(
    double x, double y, double z,
    double roll, double pitch, double yaw,
    double vx, double vy, double vz,
    double stamp_sec)
  {
    initializeState(
      Eigen::Vector3d(x, y, z),
      so3::eulerToRotation(roll, pitch, yaw),
      Eigen::Vector3d(vx, vy, vz),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), stamp_sec);
  }

  void initializeState(
    const Eigen::Vector3d & position,
    const Eigen::Matrix3d & rotation,
    const Eigen::Vector3d & velocity,
    const Eigen::Vector3d & gyro_bias,
    const Eigen::Vector3d & accel_bias,
    double stamp_sec)
  {
    PoseEntry entry;
    entry.position = position;
    entry.velocity = velocity;
    entry.R = rotation;
    entry.has_imu_factor = false;
    entry.has_ndt = false;
    poses_.clear();
    poses_.push_back(entry);
    gyro_bias_ = gyro_bias;
    accel_bias_ = accel_bias;
    dr_p_ = entry.position;
    dr_v_ = entry.velocity;
    dr_R_ = entry.R;
    last_stamp_ = stamp_sec;
    initialized_ = true;
    // Reset IMU accumulation
    current_preint_.reset();
    current_preint_.params = params_.imu_params;
    current_preint_gyro_bias_ = gyro_bias_;
    current_preint_accel_bias_ = accel_bias_;
  }

  /// Discard only the not-yet-corrected IMU interval. Optimized window states,
  /// velocity, and biases remain intact so the same raw tail can be repropagated.
  void resetPendingIntegration()
  {
    if (!initialized_ || poses_.empty()) return;
    const auto & latest = poses_.back();
    dr_p_ = latest.position;
    dr_v_ = latest.velocity;
    dr_R_ = latest.R;
    current_preint_.reset();
    current_preint_.params = params_.imu_params;
    current_preint_gyro_bias_ = gyro_bias_;
    current_preint_accel_bias_ = accel_bias_;
  }

  /// Accumulate one IMU sample (called at IMU rate ~200Hz)
  void integrateImu(
    const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel, double dt)
  {
    if (!initialized_) return;
    current_preint_.integrate(gyro, accel, gyro_bias_, accel_bias_, dt);
    // Dead-reckoning for init_guess
    Eigen::Vector3d acc_world = dr_R_ * (accel - accel_bias_) + params_.imu_params.gravity;
    dr_p_ += dr_v_ * dt + 0.5 * acc_world * dt * dt;
    dr_v_ += acc_world * dt;
    Eigen::Vector3d omega = gyro - gyro_bias_;
    dr_R_ = dr_R_ * so3::Exp(omega * dt);
  }

  // --- Measurement updates ---

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
    entry.preint_gyro_bias = current_preint_gyro_bias_;
    entry.preint_accel_bias = current_preint_accel_bias_;
    entry.has_imu_factor = (current_preint_.dt_sum >= params_.min_imu_factor_dt);
    setImuFactorInformation(entry);

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
    current_preint_gyro_bias_ = gyro_bias_;
    current_preint_accel_bias_ = accel_bias_;

    last_stamp_ = stamp_sec;
    return true;
  }

  /// Advance the correction anchor across an accepted LiDAR observation when
  /// causal IMU coverage is incomplete. No between-factor is invented for the
  /// gap and the existing window is not re-optimized through a disconnected
  /// state; optimized velocity and biases are carried into later
  /// repropagation unchanged.
  bool updatePoseOnly(
    double px, double py, double pz,
    double roll, double pitch, double yaw,
    double fitness_score, double stamp_sec)
  {
    if (!initialized_ || fitness_score > params_.fitness_reject) return false;

    PoseEntry entry;
    entry.position = Eigen::Vector3d(px, py, pz);
    entry.velocity = poses_.back().velocity;
    entry.R = so3::eulerToRotation(roll, pitch, yaw);
    entry.has_imu_factor = false;
    entry.has_ndt = true;
    entry.ndt_position = entry.position;
    entry.ndt_R = entry.R;
    entry.ndt_fitness = fitness_score;
    poses_.push_back(entry);
    while (static_cast<int>(poses_.size()) > params_.window_size) {
      poses_.pop_front();
    }

    dr_p_ = entry.position;
    dr_v_ = entry.velocity;
    dr_R_ = entry.R;
    current_preint_.reset();
    current_preint_.params = params_.imu_params;
    current_preint_gyro_bias_ = gyro_bias_;
    current_preint_accel_bias_ = accel_bias_;
    last_stamp_ = stamp_sec;
    return true;
  }

  // --- Accessors ---

  double px() const { return poses_.back().position.x(); }
  double py() const { return poses_.back().position.y(); }
  double pz() const { return poses_.back().position.z(); }
  Eigen::Vector3d velocity() const { return poses_.back().velocity; }
  Eigen::Vector3d position() const { return poses_.back().position; }
  Eigen::Matrix3d rotation() const { return poses_.back().R; }
  Eigen::Vector3d gyroBias() const { return gyro_bias_; }
  // Gauss-Newton iterations of the last optimization (0 before the first one).
  int lastIterationCount() const { return last_iteration_count_; }
  Eigen::Vector3d accelBias() const { return accel_bias_; }
  std::size_t poseCount() const { return poses_.size(); }

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
  // --- Window state ---

  struct PoseEntry
  {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    // IMU between-factor to this pose from previous
    ImuPreintegration preint;
    Eigen::Vector3d preint_gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d preint_accel_bias = Eigen::Vector3d::Zero();
    bool has_imu_factor = false;
    // The factor's information, see setImuFactorInformation().
    Eigen::Matrix<double, 9, 9> imu_info = Eigen::Matrix<double, 9, 9>::Zero();
    bool imu_info_valid = false;

    // NDT measurement
    bool has_ndt = false;
    Eigen::Vector3d ndt_position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d ndt_R = Eigen::Matrix3d::Identity();
    double ndt_fitness = 0.0;
  };

  std::deque<PoseEntry> poses_;
  bool initialized_ = false;
  int last_iteration_count_ = 0;
  double last_stamp_ = 0.0;

  // Current preintegration being accumulated
  ImuPreintegration current_preint_;
  Eigen::Vector3d current_preint_gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_preint_accel_bias_ = Eigen::Vector3d::Zero();

  // Dead-reckoning state for init_guess prediction
  Eigen::Vector3d dr_p_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dr_v_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d dr_R_ = Eigen::Matrix3d::Identity();

  // --- Sliding-window Gauss-Newton optimization ---

  // The information of an IMU factor (the inverse of its regularized preintegration
  // covariance), or nothing when that is not finite. It does not change while the
  // factor is in the window, so it is computed once, when the pose is added.
  void setImuFactorInformation(PoseEntry & entry) const
  {
    entry.imu_info_valid = false;
    if (!entry.has_imu_factor) return;
    Eigen::Matrix<double, 9, 9> covariance = entry.preint.covariance;
    covariance.diagonal().array() += params_.imu_factor_covariance_regularization;
    if (!covariance.allFinite()) return;
    Eigen::Matrix<double, 9, 9> info =
      covariance.ldlt().solve(Eigen::Matrix<double, 9, 9>::Identity());
    if (!info.allFinite()) return;
    entry.imu_info = 0.5 * (info + info.transpose());
    entry.imu_info_valid = true;
  }

public:
  using Matrix9d = Eigen::Matrix<double, 9, 9>;
  using Matrix96 = Eigen::Matrix<double, 9, 6>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Vector9d = Eigen::Matrix<double, 9, 1>;

  // The normal equations of one iteration. The poses form a chain, so H is block
  // tridiagonal over them, plus the rows and columns of the shared biases:
  //   pose_diag[i] = H(i, i), pose_next[i] = H(i - 1, i) (i >= 1),
  //   pose_bias[i] = H(i, bias), bias_diag = H(bias, bias); pose_b, bias_b likewise b.
  struct NormalEquations
  {
    std::vector<Matrix9d> pose_diag;
    std::vector<Matrix9d> pose_next;
    std::vector<Matrix96> pose_bias;
    std::vector<Vector9d> pose_b;
    Matrix6d bias_diag;
    Eigen::Matrix<double, 6, 1> bias_b;

    void reset(int n)
    {
      pose_diag.assign(n, Matrix9d::Zero());
      pose_next.assign(n, Matrix9d::Zero());
      pose_bias.assign(n, Matrix96::Zero());
      pose_b.assign(n, Vector9d::Zero());
      bias_diag.setZero();
      bias_b.setZero();
    }

    // H as one dense matrix, for the pivoting fallback.
    Eigen::MatrixXd denseH() const
    {
      const int n = static_cast<int>(pose_diag.size());
      const int bias_offset = POSE_DIM * n;
      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(bias_offset + BIAS_DIM, bias_offset + BIAS_DIM);
      for (int i = 0; i < n; ++i) {
        H.block<9, 9>(POSE_DIM * i, POSE_DIM * i) = pose_diag[i];
        if (i > 0) {
          H.block<9, 9>(POSE_DIM * (i - 1), POSE_DIM * i) = pose_next[i];
          H.block<9, 9>(POSE_DIM * i, POSE_DIM * (i - 1)) = pose_next[i].transpose();
        }
        H.block<9, 6>(POSE_DIM * i, bias_offset) = pose_bias[i];
        H.block<6, 9>(bias_offset, POSE_DIM * i) = pose_bias[i].transpose();
      }
      H.block<6, 6>(bias_offset, bias_offset) = bias_diag;
      return H;
    }

    Eigen::VectorXd denseB() const
    {
      const int n = static_cast<int>(pose_diag.size());
      Eigen::VectorXd b(POSE_DIM * n + BIAS_DIM);
      for (int i = 0; i < n; ++i) {
        b.segment<9>(POSE_DIM * i) = pose_b[i];
      }
      b.tail<6>() = bias_b;
      return b;
    }

    // Solves H delta = -b in time linear in the window: block elimination along the
    // chain, carrying the bias columns along, then the 6 x 6 Schur complement of the
    // biases. The poses' blocks are positive definite (every pose has the anchor or an
    // IMU factor, and a velocity prior). Returns false if a block cannot be factored.
    bool solve(Eigen::VectorXd & delta) const
    {
      const int n = static_cast<int>(pose_diag.size());
      using Matrix97 = Eigen::Matrix<double, 9, 7>;
      std::vector<Eigen::LDLT<Matrix9d>> pivots(n);
      std::vector<Matrix97> rhs(n);  // [H(i, bias) | -b_i], eliminated forward
      for (int i = 0; i < n; ++i) {
        rhs[i].leftCols<6>() = pose_bias[i];
        rhs[i].col(6) = -pose_b[i];
        Matrix9d block = pose_diag[i];
        if (i > 0) {
          const Matrix9d coupling = pivots[i - 1].solve(pose_next[i]);
          block.noalias() -= pose_next[i].transpose().lazyProduct(coupling);
          rhs[i].noalias() -= coupling.transpose().lazyProduct(rhs[i - 1]);
        }
        pivots[i].compute(block);
        if (pivots[i].info() != Eigen::Success) return false;
      }
      // Back substitution: x[i] = H_pp^-1 [H(pose, bias) | -b], per pose.
      for (int i = n - 1; i >= 0; --i) {
        if (i < n - 1) {
          rhs[i].noalias() -= pose_next[i + 1].lazyProduct(rhs[i + 1]);
        }
        rhs[i] = pivots[i].solve(rhs[i]);
      }
      Matrix6d schur = bias_diag;
      Eigen::Matrix<double, 6, 1> schur_b = -bias_b;
      for (int i = 0; i < n; ++i) {
        schur.noalias() -= pose_bias[i].transpose().lazyProduct(rhs[i].leftCols<6>());
        schur_b.noalias() -= pose_bias[i].transpose().lazyProduct(rhs[i].col(6));
      }
      const Eigen::Matrix<double, 6, 1> bias_delta = schur.ldlt().solve(schur_b);
      delta.resize(POSE_DIM * n + BIAS_DIM);
      for (int i = 0; i < n; ++i) {
        delta.segment<9>(POSE_DIM * i) =
          rhs[i].col(6) - rhs[i].leftCols<6>().lazyProduct(bias_delta);
      }
      delta.tail<6>() = bias_delta;
      return delta.allFinite();
    }
  };

private:
  void optimize()
  {
    const int n = static_cast<int>(poses_.size());
    if (n < 2) return;

    const int bias_offset = POSE_DIM * n;

    // H is block tridiagonal over the window plus the bias rows and columns, so it is
    // assembled by blocks and solved in time linear in the window. The dense LDLT of
    // the full (9n + 6)^2 matrix took ~87 ms per update at the 50-pose window (10
    // iterations), so the node skipped scans; a general sparse LDLT took 2.2 ms.
    NormalEquations equations;

    // The anchor holds the first pose where this update found it. It needs its residual:
    // with only the weight on the diagonal, the other factors pulled the first pose by
    // about 3e-5 per iteration, the step never fell below the 1e-10 limit, and every
    // update ran all ten iterations.
    const Eigen::Vector3d anchor_position = poses_[0].position;
    const Eigen::Matrix3d anchor_rotation = poses_[0].R;
    last_iteration_count_ = 0;
    for (int iter = 0; iter < 10; ++iter) {
      equations.reset(n);

      // --- 1. Anchor prior on first pose ---
      double anchor_w = 1e6;
      for (int k = 0; k < 3; ++k) {
        equations.pose_diag[0](k, k) += anchor_w;
        equations.pose_diag[0](6 + k, 6 + k) += anchor_w;
      }
      equations.pose_b[0].segment<3>(0) += anchor_w * (poses_[0].position - anchor_position);
      equations.pose_b[0].segment<3>(6) +=
        anchor_w * so3::Log(anchor_rotation.transpose() * poses_[0].R);

      // Velocity is only weakly observed when an interval has too little IMU
      // data to form a stable factor. Keep those dimensions numerically tame.
      double velocity_prior_w = 1e-6;
      for (int i = 0; i < n; ++i) {
        for (int k = 3; k < 6; ++k) {
          equations.pose_diag[i](k, k) += velocity_prior_w;
        }
      }

      // --- 2. IMU between-factors ---
      for (int i = 1; i < n; ++i) {
        if (!poses_[i].has_imu_factor || !poses_[i].imu_info_valid) continue;

        const auto & pi = poses_[i - 1];
        const auto & pj = poses_[i];
        const auto & stored_preint = pj.preint;
        ImuPreintegration corrected_preint = stored_preint;
        corrected_preint.correctByBias(
          gyro_bias_ - pj.preint_gyro_bias,
          accel_bias_ - pj.preint_accel_bias);

        // Compute residual
        Eigen::Matrix<double, 9, 1> r = corrected_preint.computeResidual(
          pi.position, pi.velocity, pi.R,
          pj.position, pj.velocity, pj.R);

        // Information from preintegration covariance
        const Matrix9d & info = pj.imu_info;

        // Jacobians w.r.t. pose_i [p, v, theta] and pose_j [p, v, theta]
        double dt = stored_preint.dt_sum;
        const Eigen::Vector3d & g = params_.imu_params.gravity;
        Eigen::Matrix3d Ri_T = pi.R.transpose();

        // Full Jacobian blocks (9x9 for pose_i, 9x9 for pose_j)
        Matrix9d Ji = Matrix9d::Zero();
        Matrix9d Jj = Matrix9d::Zero();

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

        // dr/d(biases): [bg | ba]
        Eigen::Matrix<double, 9, 6> Jb = Eigen::Matrix<double, 9, 6>::Zero();
        Jb.block<3, 3>(0, 0) = -stored_preint.d_dp_d_bg;
        Jb.block<3, 3>(3, 0) = -stored_preint.d_dv_d_bg;
        Jb.block<3, 3>(6, 0) = -Jr_inv_r * stored_preint.d_dR_d_bg;
        Jb.block<3, 3>(0, 3) = -stored_preint.d_dp_d_ba;
        Jb.block<3, 3>(3, 3) = -stored_preint.d_dv_d_ba;

        // H += J^T * info * J, b += J^T * info * r. The products are small and fixed in
        // size: coefficient-wise products are faster than Eigen's blocked ones there.
        const Matrix9d JiT_info = Ji.transpose().lazyProduct(info);
        const Matrix9d JjT_info = Jj.transpose().lazyProduct(info);
        const Eigen::Matrix<double, 6, 9> JbT_info = Jb.transpose().lazyProduct(info);
        equations.pose_diag[i - 1].noalias() += JiT_info.lazyProduct(Ji);
        equations.pose_next[i].noalias() += JiT_info.lazyProduct(Jj);
        equations.pose_diag[i].noalias() += JjT_info.lazyProduct(Jj);
        equations.pose_bias[i - 1].noalias() += JiT_info.lazyProduct(Jb);
        equations.pose_bias[i].noalias() += JjT_info.lazyProduct(Jb);
        equations.bias_diag.noalias() += JbT_info.lazyProduct(Jb);
        equations.pose_b[i - 1].noalias() += JiT_info.lazyProduct(r);
        equations.pose_b[i].noalias() += JjT_info.lazyProduct(r);
        equations.bias_b.noalias() += JbT_info.lazyProduct(r);
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

        const Eigen::Matrix<double, 9, 6> JT_w_info =
          J_ndt.transpose().lazyProduct(w * info_ndt);
        equations.pose_diag[i].noalias() += JT_w_info.lazyProduct(J_ndt);
        equations.pose_b[i].noalias() += JT_w_info.lazyProduct(r_ndt);
      }

      // --- 4. Bias prior factor ---
      {
        double window_duration_sec = 0.0;
        for (int i = 1; i < n; ++i) {
          if (poses_[i].has_imu_factor) {
            window_duration_sec += poses_[i].preint.dt_sum;
          }
        }
        const double gyro_bias_variance = imuBiasPriorVariance(
          params_.bias_prior_sigma_gyro,
          params_.imu_params.gyro_random_walk,
          window_duration_sec);
        const double accel_bias_variance = imuBiasPriorVariance(
          params_.bias_prior_sigma_accel,
          params_.imu_params.accel_random_walk,
          window_duration_sec);
        double info_bg = 1.0 / gyro_bias_variance;
        double info_ba = 1.0 / accel_bias_variance;
        for (int k = 0; k < 3; ++k) {
          equations.bias_diag(k, k) += info_bg;
          equations.bias_b(k) += info_bg * gyro_bias_(k);
        }
        for (int k = 0; k < 3; ++k) {
          equations.bias_diag(3 + k, 3 + k) += info_ba;
          equations.bias_b(3 + k) += info_ba * accel_bias_(k);
        }
      }

      // --- Solve ---
      Eigen::VectorXd delta;
      // The dense LDLT pivots; keep it for a matrix the block solve cannot factor.
      if (!equations.solve(delta)) {
        delta = equations.denseH().ldlt().solve(-equations.denseB());
      }

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

      last_iteration_count_ = iter + 1;
      if (delta.squaredNorm() < 1e-10) break;
    }
  }
};

#endif  // IMU_GTSAM_SMOOTHER_HPP_
