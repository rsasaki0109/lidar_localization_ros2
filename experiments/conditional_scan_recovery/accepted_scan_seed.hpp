#ifndef CONDITIONAL_SCAN_RECOVERY_ACCEPTED_SCAN_SEED_HPP_
#define CONDITIONAL_SCAN_RECOVERY_ACCEPTED_SCAN_SEED_HPP_

#include <cmath>
#include <cstdint>
#include <pcl/common/point_tests.h>
#include <pclomp/gicp_omp.h>

namespace conditional_scan_recovery
{
// Experimental seed provider, not an accepted pose source. Caller serializes
// access and rechecks initial-pose generation after expensive work completes.
class AcceptedScanSeed
{
public:
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;
  struct Result
  {
    bool valid{false};
    const char * reason{"no_reference"};
    Eigen::Matrix4f seed{Eigen::Matrix4f::Identity()};
  };

  explicit AcceptedScanSeed(double max_gap_sec) : max_gap_sec_(max_gap_sec) {}

  void reset()
  {
    reference_.reset();
    covariance_.reset();
  }

  // O(1): retain only an immutable, already-prepared accepted scan. No GICP
  // or covariance computation belongs on the healthy acceptance path.
  bool observeAccepted(
    const Cloud::ConstPtr & cloud, const Eigen::Matrix4f & pose,
    double stamp, std::uint64_t generation)
  {
    reset();
    if (!cloud || cloud->size() < 20 || cloud->header.frame_id.empty() ||
      !pose.allFinite() || !std::isfinite(stamp))
    {
      return false;
    }
    reference_ = cloud;
    reference_pose_ = pose;
    reference_stamp_ = stamp;
    generation_ = generation;
    return true;
  }

  Result estimate(
    const Cloud::ConstPtr & current, double stamp, std::uint64_t generation,
    const Eigen::Matrix3f * world_rotation_hint = nullptr)
  {
    Result result;
    if (!reference_) {return result;}
    if (generation != generation_) {result.reason = "generation_mismatch"; return result;}
    if (!std::isfinite(stamp) || stamp <= reference_stamp_ ||
      !std::isfinite(max_gap_sec_) || max_gap_sec_ <= 0.0 ||
      stamp - reference_stamp_ > max_gap_sec_)
    {
      result.reason = "time_unavailable";
      return result;
    }
    if (!current || current->size() < 20 ||
      current->header.frame_id != reference_->header.frame_id)
    {
      result.reason = "cloud_unavailable";
      return result;
    }
    for (const auto & p : *current) {
      if (!pcl::isFinite(p)) {result.reason = "nonfinite_cloud"; return result;}
    }
    for (const auto & p : *reference_) {
      if (!pcl::isFinite(p)) {result.reason = "nonfinite_cloud"; return result;}
    }
    Eigen::Matrix4f initial = Eigen::Matrix4f::Identity();
    if (world_rotation_hint) {
      const auto & rotation = *world_rotation_hint;
      if (!rotation.allFinite() || std::abs(rotation.determinant() - 1.0f) > 1e-3f ||
        !(rotation.transpose() * rotation).isApprox(Eigen::Matrix3f::Identity(), 1e-3f))
      {
        result.reason = "invalid_rotation_hint";
        return result;
      }
      initial.block<3, 3>(0, 0) =
        reference_pose_.block<3, 3>(0, 0).transpose() * rotation;
    }
    // The hint contains orientation only. Biased primary translation cannot
    // enter this initial guess, and the map gate must still validate the result.
    Probe registration;
    registration.setCorrespondenceRandomness(20);
    registration.setMaxCorrespondenceDistance(0.5);
    registration.setTransformationEpsilon(5e-4);
    registration.setMaximumIterations(30);
    registration.setInputTarget(reference_);
    if (covariance_) {registration.setTargetCovariances(covariance_);}
    registration.setInputSource(current);
    Cloud aligned;
    registration.align(aligned, initial);
    covariance_ = registration.targetCovariance();
    const auto delta = registration.getFinalTransformation();
    if (!registration.hasConverged() || !delta.allFinite()) {
      result.reason = "not_converged";
      return result;
    }
    result.seed = reference_pose_ * delta;
    result.valid = result.seed.allFinite();
    result.reason = result.valid ? "ready" : "nonfinite_seed";
    // Never replace reference_ with an unaccepted current scan.
    return result;
  }

private:
  using Registration = pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>;
  class Probe : public Registration
  {
  public:
    MatricesVectorPtr targetCovariance() const {return target_covariances_;}
  };
  double max_gap_sec_;
  Cloud::ConstPtr reference_;
  Registration::MatricesVectorPtr covariance_;
  Eigen::Matrix4f reference_pose_{Eigen::Matrix4f::Identity()};
  double reference_stamp_{0.0};
  std::uint64_t generation_{0};
};
}  // namespace conditional_scan_recovery
#endif
