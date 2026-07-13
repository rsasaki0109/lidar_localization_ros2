#ifndef LIDAR_LOCALIZATION_DIAGNOSTIC_NDT_OMP_HPP_
#define LIDAR_LOCALIZATION_DIAGNOSTIC_NDT_OMP_HPP_

#include <cstddef>
#include <limits>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pclomp/ndt_omp.h>

namespace lidar_localization
{

template<typename PointSource, typename PointTarget>
class DiagnosticNdtOmp
  : public pclomp::NormalDistributionsTransform<PointSource, PointTarget>
{
public:
  using Base = pclomp::NormalDistributionsTransform<PointSource, PointTarget>;
  using PointCloudSource = typename Base::PointCloudSource;

  bool evaluateFinalScoreHessian(
    Eigen::Matrix<double, 6, 6> & hessian,
    std::size_t & correspondence_count)
  {
    hessian.setConstant(std::numeric_limits<double>::quiet_NaN());
    correspondence_count = 0;
    if (!this->input_ || this->input_->empty() || !this->target_ || this->target_->empty()) {
      return false;
    }

    PointCloudSource transformed;
    pcl::transformPointCloud(*this->input_, transformed, this->final_transformation_);
    const Eigen::Affine3f transform(this->final_transformation_);
    const Eigen::Vector3f translation = transform.translation();
    const Eigen::Vector3f rotation = transform.rotation().eulerAngles(0, 1, 2);
    Eigen::Matrix<double, 6, 1> parameters;
    parameters << translation.x(), translation.y(), translation.z(),
      rotation.x(), rotation.y(), rotation.z();
    Eigen::Matrix<double, 6, 1> gradient;
    this->computeDerivatives(gradient, hessian, transformed, parameters, true);
    const int count = this->last_correspondence_count_.load();
    correspondence_count = count > 0 ? static_cast<std::size_t>(count) : 0;
    return hessian.allFinite() && correspondence_count > 0;
  }
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_DIAGNOSTIC_NDT_OMP_HPP_
