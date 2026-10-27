#ifndef TWIST_GTSAM_SMOOTHER_HPP_
#define TWIST_GTSAM_SMOOTHER_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iostream>
#include <limits>

/**
 * Sliding-window smoother fusing twist odometry with NDT measurements.
 *
 * Uses Pose2 (x, y, yaw) for the planar component.
 * Z, roll, pitch are taken directly from the latest NDT result.
 *
 * Factors:
 *   - Between: twist odometry between scan timestamps (each twist message counts
 *     toward the scan its stamp precedes, however late the scan is processed)
 *   - Prior:   NDT measurement with robust Huber kernel
 *   - Anchor:  prior on oldest pose in window
 *
 * Pure Eigen Gauss-Newton solver — no GTSAM dependency.
 */
class TwistGtsamSmoother
{
public:
  struct Params
  {
    double odom_sigma_x = 0.05;
    double odom_sigma_y = 0.02;
    double odom_sigma_yaw = 0.01;

    double ndt_sigma_x = 0.1;
    double ndt_sigma_y = 0.1;
    double ndt_sigma_yaw = 0.02;

    double fitness_nominal = 1.0;
    double fitness_scale_factor = 2.0;
    double fitness_reject = 50.0;

    double huber_k = 1.345;

    int window_size = 50;
  };

  TwistGtsamSmoother() = default;

  void setParams(const Params & p) { params_ = p; }

  void reset()
  {
    poses_.clear();
    initialized_ = false;
    last_predict_time_ = 0.0;
    accumulated_ = Motion{};
    folded_ = Motion{};
    pending_steps_.clear();
    latest_z_ = 0.0;
  }

  bool isInitialized() const { return initialized_; }

  void initialize(double px, double py, double pz, double yaw, double stamp_sec)
  {
    reset();

    PoseEntry entry;
    entry.x = px;
    entry.y = py;
    entry.yaw = yaw;
    entry.has_ndt = true;
    entry.ndt_x = px;
    entry.ndt_y = py;
    entry.ndt_yaw = yaw;
    entry.ndt_fitness = 0.1;
    poses_.push_back(entry);

    latest_z_ = pz;
    last_predict_time_ = stamp_sec;
    initialized_ = true;
  }

  void predict(double vx_body, double wz, double stamp_sec)
  {
    if (!initialized_) { return; }

    double dt = stamp_sec - last_predict_time_;
    if (dt <= 0.0 || dt > 1.0) {
      last_predict_time_ = stamp_sec;
      return;
    }
    last_predict_time_ = stamp_sec;

    const TwistStep step{stamp_sec, vx_body * dt, wz * dt};
    accumulated_.add(step);
    pending_steps_.push_back(step);
    // Bound the memory while no scan updates (40 s at 100 Hz).
    while (pending_steps_.size() > kMaxPendingSteps) {
      folded_.add(pending_steps_.front());
      pending_steps_.pop_front();
    }
  }

  bool update(double px, double py, double pz, double yaw_meas,
              double fitness_score, double stamp_sec)
  {
    if (!initialized_) { return false; }

    bool accept_ndt = (fitness_score <= params_.fitness_reject);

    // The NDT pose is the pose at the scan stamp: the twist received after it,
    // while the scan waited and was aligned, moves on from the new node.
    const Motion odom = motionThrough(stamp_sec);
    std::deque<TwistStep> later_steps;
    for (const auto & step : pending_steps_) {
      if (step.stamp > stamp_sec) {later_steps.push_back(step);}
    }
    pending_steps_.swap(later_steps);
    folded_ = Motion{};
    accumulated_ = motionThrough(std::numeric_limits<double>::infinity());

    PoseEntry entry;
    entry.odom_dx = odom.dx;
    entry.odom_dy = odom.dy;
    entry.odom_dyaw = odom.dyaw;
    entry.has_odom = true;

    if (accept_ndt) {
      entry.has_ndt = true;
      entry.ndt_x = px;
      entry.ndt_y = py;
      entry.ndt_yaw = yaw_meas;
      entry.ndt_fitness = fitness_score;
    }

    // Predict current pose from odometry (compose)
    const auto & prev = poses_.back();
    double c = std::cos(prev.yaw);
    double s = std::sin(prev.yaw);
    entry.x = prev.x + c * odom.dx - s * odom.dy;
    entry.y = prev.y + s * odom.dx + c * odom.dy;
    entry.yaw = normalizeAngle(prev.yaw + odom.dyaw);
    poses_.push_back(entry);

    // Trim sliding window
    while (static_cast<int>(poses_.size()) > params_.window_size) {
      poses_.pop_front();
    }

    // Optimize the window
    optimize();

    latest_z_ = pz;

    return accept_ndt;
  }

  double px() const
  {
    if (!initialized_ || poses_.empty()) return 0.0;
    return poses_.back().x;
  }

  double py() const
  {
    if (!initialized_ || poses_.empty()) return 0.0;
    return poses_.back().y;
  }

  double pz() const { return latest_z_; }

  double yaw() const
  {
    if (!initialized_ || poses_.empty()) return 0.0;
    return poses_.back().yaw;
  }

  // Returns the latest optimized pose (for publishing after update)
  Eigen::Matrix4f poseMatrix(float roll, float pitch) const
  {
    if (!initialized_ || poses_.empty()) return Eigen::Matrix4f::Identity();

    const auto & p = poses_.back();
    return buildMatrix(p.x, p.y, p.yaw, roll, pitch);
  }

  // Returns the pose predicted with all received twist
  Eigen::Matrix4f predictedPoseMatrix(float roll, float pitch) const
  {
    return predictedPoseMatrix(roll, pitch, accumulated_);
  }

  // Returns the pose predicted at a scan stamp (for NDT init_guess)
  Eigen::Matrix4f predictedPoseMatrixAt(float roll, float pitch, double stamp_sec) const
  {
    return predictedPoseMatrix(roll, pitch, motionThrough(stamp_sec));
  }

private:
  // A twist message's body-frame increment, ending at its stamp.
  struct TwistStep
  {
    double stamp;
    double dx;
    double dyaw;
  };

  // Planar motion relative to the latest pose node.
  struct Motion
  {
    double dx{0.0};
    double dy{0.0};
    double dyaw{0.0};

    void add(const TwistStep & step)
    {
      dx += step.dx * std::cos(dyaw);
      dy += step.dx * std::sin(dyaw);
      dyaw += step.dyaw;
    }
  };

  static constexpr std::size_t kMaxPendingSteps = 4000;

  Motion motionThrough(double stamp_sec) const
  {
    Motion motion = folded_;
    for (const auto & step : pending_steps_) {
      if (step.stamp <= stamp_sec) {motion.add(step);}
    }
    return motion;
  }

  Eigen::Matrix4f predictedPoseMatrix(float roll, float pitch, const Motion & motion) const
  {
    if (!initialized_ || poses_.empty()) return Eigen::Matrix4f::Identity();

    const auto & p = poses_.back();
    double c = std::cos(p.yaw);
    double s = std::sin(p.yaw);
    double pred_x = p.x + c * motion.dx - s * motion.dy;
    double pred_y = p.y + s * motion.dx + c * motion.dy;
    double pred_yaw = p.yaw + motion.dyaw;
    return buildMatrix(pred_x, pred_y, pred_yaw, roll, pitch);
  }

  Eigen::Matrix4f buildMatrix(double px, double py, double yaw_val,
                               float roll, float pitch) const
  {
    Eigen::Affine3f affine = Eigen::Affine3f::Identity();
    affine.translation() = Eigen::Vector3f(
      static_cast<float>(px),
      static_cast<float>(py),
      static_cast<float>(latest_z_));
    affine.linear() = (
      Eigen::AngleAxisf(static_cast<float>(yaw_val), Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
    ).toRotationMatrix();
    return affine.matrix();
  }

private:
  struct PoseEntry
  {
    double x{0.0}, y{0.0}, yaw{0.0};
    double odom_dx{0.0}, odom_dy{0.0}, odom_dyaw{0.0};
    bool has_odom{false};
    bool has_ndt{false};
    double ndt_x{0.0}, ndt_y{0.0}, ndt_yaw{0.0};
    double ndt_fitness{0.0};
  };

  static double normalizeAngle(double a)
  {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // Huber weight: returns weight in [0, 1]
  static double huberWeight(double residual_norm, double k)
  {
    if (residual_norm <= k) return 1.0;
    return k / residual_norm;
  }

  void optimize()
  {
    const int n = static_cast<int>(poses_.size());
    if (n < 2) return;

    const int dim = 3 * n;  // 3 DOF per pose (x, y, yaw)
    const int max_iter = 10;

    for (int iter = 0; iter < max_iter; ++iter) {
      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim, dim);
      Eigen::VectorXd b = Eigen::VectorXd::Zero(dim);

      // Anchor prior on first pose
      {
        Eigen::Vector3d sigma_anchor(0.05, 0.05, 0.005);
        Eigen::Matrix3d info_anchor = Eigen::Matrix3d::Zero();
        for (int k = 0; k < 3; ++k)
          info_anchor(k, k) = 1.0 / (sigma_anchor(k) * sigma_anchor(k));

        // Error: pose_0 - anchor (anchor = first pose's initial value after window trim)
        // We anchor the first pose tightly, so error = 0 when pose matches anchor
        // The anchor values are stored in the PoseEntry already
        // (poses_[0] is already the desired anchor, so error = 0 initially)
        // This is effectively handled by the structure — skip explicit anchor
        // to keep the first pose fixed.
      }

      // Anchor first pose in window
      double anchor_weight = 1e6;
      for (int k = 0; k < 3; ++k)
        H(k, k) += anchor_weight;

      // Odometry between factors
      for (int i = 1; i < n; ++i) {
        if (!poses_[i].has_odom) continue;

        const auto & prev = poses_[i - 1];
        const auto & cur = poses_[i];

        // Measured relative pose (odometry)
        double meas_dx = cur.odom_dx;
        double meas_dy = cur.odom_dy;
        double meas_dyaw = cur.odom_dyaw;

        // Predicted relative pose: prev^-1 * cur
        double c = std::cos(prev.yaw);
        double s = std::sin(prev.yaw);
        double dx = cur.x - prev.x;
        double dy = cur.y - prev.y;
        double pred_dx = c * dx + s * dy;
        double pred_dy = -s * dx + c * dy;
        double pred_dyaw = normalizeAngle(cur.yaw - prev.yaw);

        // Error: predicted - measured
        Eigen::Vector3d err(pred_dx - meas_dx, pred_dy - meas_dy,
                           normalizeAngle(pred_dyaw - meas_dyaw));

        // Distance-adaptive noise
        double dist = std::sqrt(meas_dx * meas_dx + meas_dy * meas_dy);
        double angle = std::abs(meas_dyaw);
        double sx = std::max(params_.odom_sigma_x, params_.odom_sigma_x * dist);
        double sy = std::max(params_.odom_sigma_y, params_.odom_sigma_y * dist);
        double syaw = std::max(params_.odom_sigma_yaw, params_.odom_sigma_yaw * angle * 10.0);

        Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
        info(0, 0) = 1.0 / (sx * sx);
        info(1, 1) = 1.0 / (sy * sy);
        info(2, 2) = 1.0 / (syaw * syaw);

        // Jacobian of error w.r.t. prev pose (3x3)
        Eigen::Matrix3d J_prev;
        J_prev << -c, -s,  (-s * dx + c * dy),
                   s, -c,  (-c * dx - s * dy),
                   0,  0,  -1.0;

        // Jacobian of error w.r.t. cur pose (3x3)
        Eigen::Matrix3d J_cur;
        J_cur << c,  s,  0,
                -s,  c,  0,
                 0,  0,  1.0;

        int idx_prev = (i - 1) * 3;
        int idx_cur = i * 3;

        // Accumulate into H and b
        H.block<3, 3>(idx_prev, idx_prev) += J_prev.transpose() * info * J_prev;
        H.block<3, 3>(idx_prev, idx_cur)  += J_prev.transpose() * info * J_cur;
        H.block<3, 3>(idx_cur, idx_prev)  += J_cur.transpose() * info * J_prev;
        H.block<3, 3>(idx_cur, idx_cur)   += J_cur.transpose() * info * J_cur;
        b.segment<3>(idx_prev) += J_prev.transpose() * info * err;
        b.segment<3>(idx_cur)  += J_cur.transpose() * info * err;
      }

      // NDT prior factors with Huber robust kernel
      for (int i = 0; i < n; ++i) {
        if (!poses_[i].has_ndt) continue;

        // Error: pose - ndt_measurement
        Eigen::Vector3d err(
          poses_[i].x - poses_[i].ndt_x,
          poses_[i].y - poses_[i].ndt_y,
          normalizeAngle(poses_[i].yaw - poses_[i].ndt_yaw));

        // Fitness-adaptive noise
        double scale = std::pow(
          std::max(poses_[i].ndt_fitness, 0.1) / params_.fitness_nominal,
          params_.fitness_scale_factor);
        double sigma_x = params_.ndt_sigma_x * std::sqrt(scale);
        double sigma_y = params_.ndt_sigma_y * std::sqrt(scale);
        double sigma_yaw = params_.ndt_sigma_yaw * std::sqrt(scale);

        Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
        info(0, 0) = 1.0 / (sigma_x * sigma_x);
        info(1, 1) = 1.0 / (sigma_y * sigma_y);
        info(2, 2) = 1.0 / (sigma_yaw * sigma_yaw);

        // Huber weighting
        Eigen::Vector3d weighted_err;
        weighted_err(0) = err(0) / sigma_x;
        weighted_err(1) = err(1) / sigma_y;
        weighted_err(2) = err(2) / sigma_yaw;
        double norm = weighted_err.norm();
        double w = huberWeight(norm, params_.huber_k);

        // Jacobian of prior error w.r.t. pose is Identity
        int idx = i * 3;
        H.block<3, 3>(idx, idx) += w * info;
        b.segment<3>(idx) += w * info * err;
      }

      // Solve H * delta = -b
      Eigen::VectorXd delta = H.ldlt().solve(-b);

      // Update poses
      double max_update = 0.0;
      for (int i = 0; i < n; ++i) {
        // Skip anchor (first pose) — effectively fixed by large diagonal
        poses_[i].x += delta(static_cast<Eigen::Index>(i) * 3 + 0);
        poses_[i].y += delta(static_cast<Eigen::Index>(i) * 3 + 1);
        poses_[i].yaw =
          normalizeAngle(poses_[i].yaw + delta(static_cast<Eigen::Index>(i) * 3 + 2));
        max_update =
          std::max(max_update,
                   delta.segment<3>(static_cast<Eigen::Index>(i) * 3).squaredNorm());
      }

      if (max_update < 1e-10) break;
    }
  }

  Params params_;

  std::deque<PoseEntry> poses_;
  bool initialized_{false};
  double last_predict_time_{0.0};

  // Motion through all received twist, and through the steps folded out of
  // pending_steps_; pending_steps_ holds the steps since the latest node.
  Motion accumulated_;
  Motion folded_;
  std::deque<TwistStep> pending_steps_;

  double latest_z_{0.0};
};

#endif  // TWIST_GTSAM_SMOOTHER_HPP_
