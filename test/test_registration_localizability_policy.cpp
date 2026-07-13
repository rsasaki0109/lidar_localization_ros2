#include <cassert>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "lidar_localization/registration_localizability_policy.hpp"

namespace ll = lidar_localization;

int main()
{
  {
    const auto disabled = ll::analyzeRegistrationHessian(
      Eigen::Matrix<double, 6, 6>::Identity(), 10, false);
    assert(!disabled.enabled);
    assert(!disabled.valid);
    assert(disabled.status == "disabled");
  }
  {
    Eigen::Matrix<double, 6, 6> score_hessian =
      Eigen::Matrix<double, 6, 6>::Zero();
    score_hessian.diagonal() << -100.0, -200.0, -300.0, -400.0, -500.0, -600.0;
    const auto metrics = ll::analyzeRegistrationHessian(score_hessian, 100);
    assert(metrics.enabled);
    assert(metrics.valid);
    assert(metrics.status == "ok");
    assert(metrics.correspondence_count == 100);
    assert(metrics.nonpositive_eigenvalue_count == 0);
    assert(std::abs(metrics.eigenvalues[0] - 1.0) < 1e-12);
    assert(std::abs(metrics.eigenvalues[5] - 6.0) < 1e-12);
    assert(std::abs(metrics.weak_ratio - 1.0 / 6.0) < 1e-12);
    assert(std::abs(metrics.absolute_condition_number - 6.0) < 1e-12);
    assert(std::abs(std::abs(metrics.weakest_eigenvector[0]) - 1.0) < 1e-12);
  }
  {
    Eigen::Matrix<double, 6, 6> indefinite =
      -Eigen::Matrix<double, 6, 6>::Identity();
    indefinite(2, 2) = 2.0;
    const auto metrics = ll::analyzeRegistrationHessian(indefinite, 1);
    assert(metrics.valid);
    assert(metrics.nonpositive_eigenvalue_count == 1);
  }
  {
    const auto empty = ll::analyzeRegistrationHessian(
      Eigen::Matrix<double, 6, 6>::Identity(), 0);
    assert(!empty.valid);
    assert(empty.status == "no_correspondences");
  }
  {
    Eigen::Matrix<double, 6, 6> invalid = Eigen::Matrix<double, 6, 6>::Identity();
    invalid(0, 0) = std::numeric_limits<double>::quiet_NaN();
    const auto metrics = ll::analyzeRegistrationHessian(invalid, 1);
    assert(!metrics.valid);
    assert(metrics.status == "non_finite_hessian");
  }
  return 0;
}
