// Exact point cloud downsampling via fast Caratheodory coreset extraction.
//
// Given N residual blocks with stacked Jacobian J (N*M x dof) and residual
// e (N*M), extracts a weighted subset S of blocks such that at the
// linearization point
//   sum_{i in S} w_i A_i^T A_i == sum_{i=1..N} A_i^T A_i,  A_i = [J_i | e_i],
// i.e. the quadratic error function coefficients H = J^T W J, b = J^T W e,
// c = e^T W e are reproduced exactly with |S| <= (dof+1)(dof+2)/2 + 1 blocks.
//
// Clean-room implementation from:
//  - Koide et al., "Exact Point Cloud Downsampling for Fast and Accurate
//    Global Trajectory Optimization", IROS 2023.
//  - Koide et al., "Tightly Coupled Range Inertial Odometry and Mapping with
//    Exact Point Cloud Downsampling", ICRA 2025 (arXiv:2505.01017).
//  - Maalouf, Jubran, Feldman, "Fast and Accurate Least-Mean-Squares
//    Solvers", NeurIPS 2019 (fast Caratheodory clustering scheme).

#ifndef LIDAR_LOCALIZATION__EXACT_QUADRATIC_CORESET_HPP_
#define LIDAR_LOCALIZATION__EXACT_QUADRATIC_CORESET_HPP_

#include <Eigen/Dense>

#include <algorithm>
#include <limits>
#include <numeric>
#include <vector>

namespace lidar_localization
{

struct QuadraticCoreset
{
  // Indices of the retained residual blocks in the original block order.
  std::vector<int> block_indices;
  // Positive weight to apply to every row of the retained block.
  std::vector<double> weights;
};

namespace detail
{

// One in-place Caratheodory pruning pass: given points (columns) and positive
// weights whose weighted sum is s = sum_i w_i p_i, reduce the number of
// positively weighted points to at most dim + 1 while keeping s unchanged.
// Returns the indices (into the input columns) that keep a positive weight.
inline std::vector<int> caratheodoryPrune(
  const Eigen::MatrixXd & points, Eigen::VectorXd & weights)
{
  const int dim = static_cast<int>(points.rows());
  std::vector<int> active(points.cols());
  std::iota(active.begin(), active.end(), 0);

  while (static_cast<int>(active.size()) > dim + 1) {
    const int m = static_cast<int>(active.size());
    // Null combination: sum_i v_i p_i = 0 with sum_i v_i = 0. Build the
    // difference matrix B = [p_1 - p_0, ..., p_{m-1} - p_0] and take any
    // kernel vector; v_0 absorbs the negated sum.
    Eigen::MatrixXd differences(dim, m - 1);
    for (int i = 1; i < m; ++i) {
      differences.col(i - 1) = points.col(active[i]) - points.col(active[0]);
    }
    const Eigen::FullPivLU<Eigen::MatrixXd> lu(differences);
    const Eigen::MatrixXd kernel = lu.kernel();
    if (lu.dimensionOfKernel() == 0) {
      break;  // Numerically full rank: cannot prune further.
    }
    Eigen::VectorXd v(m);
    v.tail(m - 1) = kernel.col(0);
    v(0) = -v.tail(m - 1).sum();

    double alpha = std::numeric_limits<double>::infinity();
    int drop = -1;
    for (int i = 0; i < m; ++i) {
      if (v(i) > 1e-12) {
        const double ratio = weights(active[i]) / v(i);
        if (ratio < alpha) {
          alpha = ratio;
          drop = i;
        }
      }
    }
    if (drop < 0) {
      // Kernel vector has no positive entry; flip it (its negation is also a
      // null combination) and retry on the next loop iteration.
      for (int i = 0; i < m; ++i) {
        v(i) = -v(i);
      }
      for (int i = 0; i < m; ++i) {
        if (v(i) > 1e-12) {
          const double ratio = weights(active[i]) / v(i);
          if (ratio < alpha) {
            alpha = ratio;
            drop = i;
          }
        }
      }
      if (drop < 0) {
        break;  // Degenerate; keep the current (already valid) subset.
      }
    }
    for (int i = 0; i < m; ++i) {
      weights(active[i]) -= alpha * v(i);
    }
    weights(active[drop]) = 0.0;  // Exact removal of the limiting point.
    std::vector<int> next;
    next.reserve(active.size() - 1);
    for (int i = 0; i < m; ++i) {
      if (i != drop && weights(active[i]) > 0.0) {
        next.push_back(active[i]);
      }
    }
    active.swap(next);
  }
  return active;
}

// Upper-triangular vectorization of A^T A for one block row range.
inline Eigen::VectorXd symmetricOuterProductVector(const Eigen::MatrixXd & block)
{
  const int s = static_cast<int>(block.cols());
  const Eigen::MatrixXd gram = block.transpose() * block;
  Eigen::VectorXd out(s * (s + 1) / 2);
  int k = 0;
  for (int r = 0; r < s; ++r) {
    for (int c = r; c < s; ++c) {
      out(k++) = gram(r, c);
    }
  }
  return out;
}

}  // namespace detail

// jacobian: (N*rows_per_block) x dof, residual: (N*rows_per_block).
// cluster_count: clusters per fast-Caratheodory round (must exceed the
// symmetric dimension (dof+1)(dof+2)/2 + 1 to make progress; e.g. 64 for
// dof = 6). target_size <= 0 selects the minimum exact size.
inline QuadraticCoreset extractQuadraticCoreset(
  const Eigen::MatrixXd & jacobian,
  const Eigen::VectorXd & residual,
  int rows_per_block,
  int cluster_count = 64,
  int target_size = 0)
{
  const int dof = static_cast<int>(jacobian.cols());
  const int block_count = static_cast<int>(jacobian.rows()) / rows_per_block;
  const int s = dof + 1;
  const int sym_dim = s * (s + 1) / 2;
  const int minimum_size = sym_dim + 1;
  const int target = std::max(target_size, minimum_size);

  // Per-block vectorized Gram contributions u_i = vec(A_i^T A_i).
  Eigen::MatrixXd contributions(sym_dim, block_count);
  Eigen::MatrixXd block(rows_per_block, s);
  for (int i = 0; i < block_count; ++i) {
    block.leftCols(dof) = jacobian.middleRows(i * rows_per_block, rows_per_block);
    block.col(dof) = residual.segment(i * rows_per_block, rows_per_block);
    contributions.col(i) = detail::symmetricOuterProductVector(block);
  }

  std::vector<int> indices(block_count);
  std::iota(indices.begin(), indices.end(), 0);
  std::vector<double> weights(block_count, 1.0);

  const int effective_clusters = std::max(cluster_count, minimum_size + 1);
  while (static_cast<int>(indices.size()) > target) {
    const int m = static_cast<int>(indices.size());
    if (m <= effective_clusters) {
      // Direct pass on the remaining points.
      Eigen::MatrixXd points(sym_dim, m);
      Eigen::VectorXd point_weights(m);
      for (int i = 0; i < m; ++i) {
        points.col(i) = contributions.col(indices[i]);
        point_weights(i) = weights[indices[i]];
      }
      // Fold weights into the points so the pruning invariant is the plain
      // sum sum_i w'_i q_i with w'_i starting at 1.
      for (int i = 0; i < m; ++i) {
        points.col(i) *= point_weights(i);
      }
      Eigen::VectorXd unit = Eigen::VectorXd::Ones(m);
      const std::vector<int> kept = detail::caratheodoryPrune(points, unit);
      std::vector<int> next_indices;
      std::vector<double> next_weights;
      next_indices.reserve(kept.size());
      next_weights.reserve(kept.size());
      for (const int i : kept) {
        next_indices.push_back(indices[i]);
        next_weights.push_back(weights[indices[i]] * unit(i));
      }
      for (std::size_t i = 0; i < next_indices.size(); ++i) {
        weights[next_indices[i]] = next_weights[i];
      }
      indices.swap(next_indices);
      break;  // A direct pass always reaches the minimum size.
    }

    // Fast round: contiguous clusters, prune their weighted sums.
    const int clusters = effective_clusters;
    const int chunk = (m + clusters - 1) / clusters;
    const int cluster_total = (m + chunk - 1) / chunk;
    Eigen::MatrixXd cluster_sums = Eigen::MatrixXd::Zero(sym_dim, cluster_total);
    for (int i = 0; i < m; ++i) {
      cluster_sums.col(i / chunk) += weights[indices[i]] * contributions.col(indices[i]);
    }
    Eigen::VectorXd cluster_weights = Eigen::VectorXd::Ones(cluster_total);
    const std::vector<int> kept_clusters =
      detail::caratheodoryPrune(cluster_sums, cluster_weights);

    std::vector<int> next_indices;
    next_indices.reserve(kept_clusters.size() * chunk);
    for (const int c : kept_clusters) {
      const int begin = c * chunk;
      const int end = std::min(begin + chunk, m);
      for (int i = begin; i < end; ++i) {
        weights[indices[i]] *= cluster_weights(c);
        next_indices.push_back(indices[i]);
      }
    }
    if (next_indices.size() >= indices.size()) {
      break;  // No progress (degenerate data); the current subset is valid.
    }
    indices.swap(next_indices);
  }

  QuadraticCoreset result;
  result.block_indices.reserve(indices.size());
  result.weights.reserve(indices.size());
  for (const int index : indices) {
    result.block_indices.push_back(index);
    result.weights.push_back(weights[index]);
  }
  return result;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION__EXACT_QUADRATIC_CORESET_HPP_
