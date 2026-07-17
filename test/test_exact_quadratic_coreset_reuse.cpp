// M1 validation for deferred-sampling reuse: a coreset extracted at one
// linearization pose must stay a good approximation of the full quadratic
// when re-linearized at nearby poses (ICRA 2025 reuse thresholds: extract
// after the update step falls below 0.25 m / 0.25 deg, re-extract beyond
// 1.0 m / 1.0 deg).

#include "lidar_localization/exact_quadratic_coreset.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

namespace ll = lidar_localization;

namespace
{

struct Problem
{
  std::vector<Eigen::Vector3d> source;
  std::vector<Eigen::Vector3d> target;
  std::vector<Eigen::Matrix3d> sqrt_information;
};

// Structured scene: three orthogonal planes plus scattered clutter, GICP-like
// per-point anisotropic weights (strong along each plane normal).
Problem makeProblem(int points_per_plane, std::mt19937 & rng)
{
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uniform(-10.0, 10.0);
  Problem problem;
  for (int axis = 0; axis < 3; ++axis) {
    for (int i = 0; i < points_per_plane; ++i) {
      Eigen::Vector3d point(uniform(rng), uniform(rng), uniform(rng));
      point(axis) = (axis == 2) ? 0.0 : 10.0 * (axis == 0 ? 1.0 : -1.0);
      Eigen::Vector3d noise = 0.01 * Eigen::Vector3d(gauss(rng), gauss(rng), gauss(rng));
      problem.target.push_back(point);
      problem.source.push_back(point + noise);
      Eigen::Matrix3d sqrt_info = Eigen::Matrix3d::Identity();
      sqrt_info(axis, axis) = 10.0;  // plane-normal direction is well constrained
      problem.sqrt_information.push_back(sqrt_info);
    }
  }
  return problem;
}

Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d out;
  out << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return out;
}

// Linearize r_i = R p_i + t - q_i at pose (R, t) with left perturbation.
void linearize(
  const Problem & problem, const Eigen::Matrix3d & rotation, const Eigen::Vector3d & translation,
  Eigen::MatrixXd & jacobian, Eigen::VectorXd & residual)
{
  const int n = static_cast<int>(problem.source.size());
  jacobian.resize(3 * n, 6);
  residual.resize(3 * n);
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3d transformed = rotation * problem.source[i] + translation;
    const Eigen::Vector3d r = problem.sqrt_information[i] * (transformed - problem.target[i]);
    Eigen::Matrix<double, 3, 6> j;
    j.leftCols<3>() = -skew(transformed);
    j.rightCols<3>() = Eigen::Matrix3d::Identity();
    jacobian.middleRows(3 * i, 3) = problem.sqrt_information[i] * j;
    residual.segment(3 * i, 3) = r;
  }
}

Eigen::VectorXd gaussNewtonStep(const Eigen::MatrixXd & j, const Eigen::VectorXd & r)
{
  const Eigen::MatrixXd h = j.transpose() * j;
  const Eigen::VectorXd b = j.transpose() * r;
  return h.ldlt().solve(-b);
}

Eigen::VectorXd coresetGaussNewtonStep(
  const Eigen::MatrixXd & j, const Eigen::VectorXd & r, const ll::QuadraticCoreset & coreset)
{
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(6, 6);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(6);
  for (std::size_t k = 0; k < coreset.block_indices.size(); ++k) {
    const int i = coreset.block_indices[k];
    const double w = coreset.weights[k];
    const Eigen::MatrixXd jb = j.middleRows(3 * i, 3);
    const Eigen::VectorXd rb = r.segment(3 * i, 3);
    h += w * jb.transpose() * jb;
    b += w * jb.transpose() * rb;
  }
  return h.ldlt().solve(-b);
}

void test_reuse_error_stays_small_within_deferred_sampling_thresholds()
{
  std::mt19937 rng(17);
  const Problem problem = makeProblem(1500, rng);  // 4500 residual blocks

  // Sampling pose: slightly off identity, as after a first optimizer step.
  const Eigen::Matrix3d rotation0 =
    Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d translation0(0.02, -0.01, 0.005);

  Eigen::MatrixXd jacobian;
  Eigen::VectorXd residual;
  linearize(problem, rotation0, translation0, jacobian, residual);
  const auto coreset = ll::extractQuadraticCoreset(jacobian, residual, 3);

  // Exactness at the sampling pose.
  const Eigen::VectorXd full0 = gaussNewtonStep(jacobian, residual);
  const Eigen::VectorXd sub0 = coresetGaussNewtonStep(jacobian, residual, coreset);
  assert((full0 - sub0).norm() < 1e-8 * (full0.norm() + 1e-12));

  std::printf("displacement -> relative GN-step deviation (coreset size %zu)\n",
    coreset.block_indices.size());
  double deviation_at_reuse_threshold = 0.0;
  for (const double displacement : {0.05, 0.1, 0.25, 0.5, 1.0}) {
    const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(0.25 * M_PI / 180.0 * displacement / 0.25, Eigen::Vector3d::UnitZ())
      .toRotationMatrix() * rotation0;
    const Eigen::Vector3d translation =
      translation0 + displacement * Eigen::Vector3d(1.0, 0.5, 0.1).normalized();
    linearize(problem, rotation, translation, jacobian, residual);
    const Eigen::VectorXd full = gaussNewtonStep(jacobian, residual);
    const Eigen::VectorXd sub = coresetGaussNewtonStep(jacobian, residual, coreset);
    const double relative = (full - sub).norm() / (full.norm() + 1e-12);
    std::printf("  %.2f m: %.4f (|full|=%.4f)\n", displacement, relative, full.norm());
    if (std::abs(displacement - 0.25) < 1e-9) {
      deviation_at_reuse_threshold = relative;
    }
  }
  // Within the paper's deferred-sampling threshold the reused coreset must
  // still point the optimizer the right way.
  assert(deviation_at_reuse_threshold < 0.10);
}

}  // namespace

int main()
{
  test_reuse_error_stays_small_within_deferred_sampling_thresholds();
  std::printf("test_exact_quadratic_coreset_reuse: all tests passed\n");
  return 0;
}
