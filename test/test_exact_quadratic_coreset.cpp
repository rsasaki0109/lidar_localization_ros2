#include "lidar_localization/exact_quadratic_coreset.hpp"

#include <cassert>
#include <chrono>
#include <cstdio>
#include <random>

namespace ll = lidar_localization;

namespace
{

struct QuadraticCoefficients
{
  Eigen::MatrixXd hessian;
  Eigen::VectorXd gradient;
  double constant{0.0};
};

QuadraticCoefficients fullCoefficients(
  const Eigen::MatrixXd & jacobian, const Eigen::VectorXd & residual)
{
  QuadraticCoefficients out;
  out.hessian = jacobian.transpose() * jacobian;
  out.gradient = jacobian.transpose() * residual;
  out.constant = residual.squaredNorm();
  return out;
}

QuadraticCoefficients coresetCoefficients(
  const Eigen::MatrixXd & jacobian,
  const Eigen::VectorXd & residual,
  int rows_per_block,
  const ll::QuadraticCoreset & coreset)
{
  const int dof = static_cast<int>(jacobian.cols());
  QuadraticCoefficients out;
  out.hessian = Eigen::MatrixXd::Zero(dof, dof);
  out.gradient = Eigen::VectorXd::Zero(dof);
  for (std::size_t i = 0; i < coreset.block_indices.size(); ++i) {
    const int block = coreset.block_indices[i];
    const double weight = coreset.weights[i];
    const Eigen::MatrixXd j = jacobian.middleRows(block * rows_per_block, rows_per_block);
    const Eigen::VectorXd e = residual.segment(block * rows_per_block, rows_per_block);
    out.hessian += weight * j.transpose() * j;
    out.gradient += weight * j.transpose() * e;
    out.constant += weight * e.squaredNorm();
  }
  return out;
}

void expectExact(
  const QuadraticCoefficients & full, const QuadraticCoefficients & sampled)
{
  const double scale = full.hessian.norm();
  assert((full.hessian - sampled.hessian).norm() < 1e-8 * scale);
  assert((full.gradient - sampled.gradient).norm() < 1e-8 * scale);
  assert(std::abs(full.constant - sampled.constant) < 1e-8 * scale);
}

Eigen::MatrixXd randomJacobian(int rows, int dof, std::mt19937 & rng)
{
  std::normal_distribution<double> gauss(0.0, 1.0);
  Eigen::MatrixXd out(rows, dof);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < dof; ++c) {
      out(r, c) = gauss(rng);
    }
  }
  return out;
}

Eigen::VectorXd randomResidual(int rows, std::mt19937 & rng)
{
  std::normal_distribution<double> gauss(0.0, 1.0);
  Eigen::VectorXd out(rows);
  for (int r = 0; r < rows; ++r) {
    out(r) = gauss(rng);
  }
  return out;
}

void test_minimum_size_coreset_reproduces_quadratic_exactly()
{
  std::mt19937 rng(42);
  const int blocks = 500;
  const int rows_per_block = 3;
  const int dof = 6;
  const Eigen::MatrixXd jacobian = randomJacobian(blocks * rows_per_block, dof, rng);
  const Eigen::VectorXd residual = randomResidual(blocks * rows_per_block, rng);

  const auto coreset = ll::extractQuadraticCoreset(jacobian, residual, rows_per_block);

  const int sym_dim = (dof + 1) * (dof + 2) / 2;
  assert(static_cast<int>(coreset.block_indices.size()) <= sym_dim + 1);
  for (const double weight : coreset.weights) {
    assert(weight > 0.0);
  }
  expectExact(
    fullCoefficients(jacobian, residual),
    coresetCoefficients(jacobian, residual, rows_per_block, coreset));
}

void test_target_size_keeps_more_blocks_and_stays_exact()
{
  std::mt19937 rng(7);
  const int blocks = 1000;
  const int rows_per_block = 3;
  const Eigen::MatrixXd jacobian = randomJacobian(blocks * rows_per_block, 6, rng);
  const Eigen::VectorXd residual = randomResidual(blocks * rows_per_block, rng);

  const auto coreset =
    ll::extractQuadraticCoreset(jacobian, residual, rows_per_block, 64, 100);

  assert(static_cast<int>(coreset.block_indices.size()) <= 100 + 64);
  assert(static_cast<int>(coreset.block_indices.size()) >= 29);
  expectExact(
    fullCoefficients(jacobian, residual),
    coresetCoefficients(jacobian, residual, rows_per_block, coreset));
}

void test_small_input_is_returned_unchanged()
{
  std::mt19937 rng(3);
  const int blocks = 10;
  const int rows_per_block = 3;
  const Eigen::MatrixXd jacobian = randomJacobian(blocks * rows_per_block, 6, rng);
  const Eigen::VectorXd residual = randomResidual(blocks * rows_per_block, rng);

  const auto coreset = ll::extractQuadraticCoreset(jacobian, residual, rows_per_block);

  assert(static_cast<int>(coreset.block_indices.size()) == blocks);
  for (const double weight : coreset.weights) {
    assert(std::abs(weight - 1.0) < 1e-12);
  }
}

void test_single_row_blocks_are_supported()
{
  std::mt19937 rng(11);
  const int blocks = 400;
  const Eigen::MatrixXd jacobian = randomJacobian(blocks, 6, rng);
  const Eigen::VectorXd residual = randomResidual(blocks, rng);

  const auto coreset = ll::extractQuadraticCoreset(jacobian, residual, 1);

  assert(static_cast<int>(coreset.block_indices.size()) <= 29);
  expectExact(
    fullCoefficients(jacobian, residual),
    coresetCoefficients(jacobian, residual, 1, coreset));
}

void test_duplicated_blocks_stay_exact()
{
  std::mt19937 rng(5);
  const int unique_blocks = 40;
  const int repeats = 10;
  const int rows_per_block = 3;
  const Eigen::MatrixXd base = randomJacobian(unique_blocks * rows_per_block, 6, rng);
  const Eigen::VectorXd base_residual = randomResidual(unique_blocks * rows_per_block, rng);
  Eigen::MatrixXd jacobian(unique_blocks * repeats * rows_per_block, 6);
  Eigen::VectorXd residual(unique_blocks * repeats * rows_per_block);
  for (int r = 0; r < repeats; ++r) {
    jacobian.middleRows(r * base.rows(), base.rows()) = base;
    residual.segment(r * base_residual.size(), base_residual.size()) = base_residual;
  }

  const auto coreset = ll::extractQuadraticCoreset(jacobian, residual, rows_per_block);

  assert(static_cast<int>(coreset.block_indices.size()) <= 29);
  expectExact(
    fullCoefficients(jacobian, residual),
    coresetCoefficients(jacobian, residual, rows_per_block, coreset));
}

void test_relative_pose_coreset_reproduces_binary_blocks_exactly()
{
  std::mt19937 rng(19);
  const int blocks = 500;
  const int rows_per_block = 3;
  const Eigen::MatrixXd source_jacobian =
    randomJacobian(blocks * rows_per_block, 6, rng);
  const Eigen::VectorXd residual =
    randomResidual(blocks * rows_per_block, rng);

  // A binary registration residual depends only on the relative pose. At one
  // linearization point its target-pose Jacobian is the source-pose Jacobian
  // multiplied by one common adjoint matrix. Thus a six-DoF coreset must also
  // preserve the complete 12-DoF binary quadratic.
  const Eigen::Matrix<double, 6, 6> common_adjoint =
    randomJacobian(6, 6, rng);
  Eigen::MatrixXd binary_jacobian(blocks * rows_per_block, 12);
  binary_jacobian.leftCols(6) = source_jacobian * common_adjoint;
  binary_jacobian.rightCols(6) = source_jacobian;

  const auto coreset = ll::extractQuadraticCoreset(
    source_jacobian, residual, rows_per_block);
  assert(static_cast<int>(coreset.block_indices.size()) <= 29);
  expectExact(
    fullCoefficients(binary_jacobian, residual),
    coresetCoefficients(binary_jacobian, residual, rows_per_block, coreset));
}

void report_runtime_for_benchmark_scale()
{
  std::mt19937 rng(1);
  const int blocks = 6500;
  const int rows_per_block = 3;
  const Eigen::MatrixXd jacobian = randomJacobian(blocks * rows_per_block, 6, rng);
  const Eigen::VectorXd residual = randomResidual(blocks * rows_per_block, rng);

  const auto start = std::chrono::steady_clock::now();
  const auto coreset = ll::extractQuadraticCoreset(jacobian, residual, rows_per_block);
  const auto elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();

  expectExact(
    fullCoefficients(jacobian, residual),
    coresetCoefficients(jacobian, residual, rows_per_block, coreset));
  std::printf(
    "coreset: %d -> %zu blocks in %.2f ms\n",
    blocks, coreset.block_indices.size(), elapsed);
}

}  // namespace

int main()
{
  test_minimum_size_coreset_reproduces_quadratic_exactly();
  test_target_size_keeps_more_blocks_and_stays_exact();
  test_small_input_is_returned_unchanged();
  test_single_row_blocks_are_supported();
  test_duplicated_blocks_stay_exact();
  test_relative_pose_coreset_reproduces_binary_blocks_exactly();
  report_runtime_for_benchmark_scale();
  std::printf("test_exact_quadratic_coreset: all tests passed\n");
  return 0;
}
