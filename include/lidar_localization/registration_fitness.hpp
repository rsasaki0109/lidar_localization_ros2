#ifndef LIDAR_LOCALIZATION_REGISTRATION_FITNESS_HPP_
#define LIDAR_LOCALIZATION_REGISTRATION_FITNESS_HPP_

#include <pcl/common/transforms.h>
#include <pcl/registration/registration.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace lidar_localization
{

// The fitness score and, when inlier_distance_m > 0, the inlier ratio: the share of the
// source points, moved by the final transformation, whose nearest target point lies
// within inlier_distance_m. One nearest-neighbour search per point serves both, spread
// over THREADS. The fitness is summed in point order, as in getFitnessScore().
//
// The fitness is a mean squared distance, so points that see structure missing from the
// map, a few metres from any map point, dominate it. On Koide outdoor_hard_01a at
// 111.3 s, the alignment from the ground truth scores 3.7, and one 8.3 m and 34 deg off
// scores 2.3. The inlier ratio counts those points once each: 0.92 against 0.40.
//
// A scan of fewer than min_inlier_points points gets an inlier ratio of 0: a nearly
// blind scan may lie near the map by chance.
struct RegistrationFitness
{
  double score{std::numeric_limits<double>::max()};
  double inlier_ratio{std::numeric_limits<double>::quiet_NaN()};
};

template<typename PointSource, typename PointTarget, typename Scalar>
RegistrationFitness registrationFitness(
  pcl::Registration<PointSource, PointTarget, Scalar> & registration, int threads,
  double inlier_distance_m, std::size_t min_inlier_points)
{
  RegistrationFitness result;
  const auto source = registration.getInputSource();
  const auto tree = registration.getSearchMethodTarget();
  // Without a target tree there is nothing to count against, so no inlier ratio.
  if (!source || !tree || !tree->getInputCloud()) {
    result.score = registration.getFitnessScore();
    return result;
  }
  pcl::PointCloud<PointSource> transformed;
  pcl::transformPointCloud(*source, transformed, registration.getFinalTransformation());
  const int count = static_cast<int>(transformed.size());
  std::vector<float> distances(transformed.size(), std::numeric_limits<float>::quiet_NaN());
#pragma omp parallel for num_threads(std::max(1, threads)) schedule(guided, 32)
  for (int i = 0; i < count; ++i) {
    pcl::Indices index(1);
    std::vector<float> distance(1);
    if (tree->nearestKSearch(transformed[i], 1, index, distance) > 0) {
      distances[i] = distance[0];
    }
  }
  const float inlier_limit = static_cast<float>(inlier_distance_m * inlier_distance_m);
  double fitness = 0.0;
  int matched = 0;
  int inliers = 0;
  for (const float distance : distances) {
    // A point without a neighbour is skipped; PCL would read an unset distance there.
    if (!std::isnan(distance)) {
      fitness += distance;
      ++matched;
      if (distance <= inlier_limit) {
        ++inliers;
      }
    }
  }
  result.score = matched > 0 ? fitness / matched : std::numeric_limits<double>::max();
  if (inlier_distance_m > 0.0) {
    result.inlier_ratio = transformed.size() < min_inlier_points ? 0.0 :
      static_cast<double>(inliers) / count;
  }
  return result;
}

// pcl::Registration::getFitnessScore() with the nearest-neighbour searches spread over
// THREADS. It finds, for every source point moved by the final transformation, the
// squared distance to its nearest target point, then sums them in point order as PCL
// does, so the score is bit-identical. PCL's loop is serial: on Boreas, about 4 600
// points against a 100 m crop took 6.5-7 ms there and take 2 ms here on 4-8 threads.
// It falls back to PCL's own for one thread or without a source or a target tree.
template<typename PointSource, typename PointTarget, typename Scalar>
double parallelFitnessScore(
  pcl::Registration<PointSource, PointTarget, Scalar> & registration, int threads)
{
  const auto source = registration.getInputSource();
  const auto tree = registration.getSearchMethodTarget();
  if (threads <= 1 || !source || !tree || !tree->getInputCloud()) {
    return registration.getFitnessScore();
  }
  return registrationFitness(registration, threads, -1.0, 0).score;
}

// The inlier ratio on the fitness score's scale, for the measurement gate and everything
// else that reads the score: 0 when every point is an inlier, score_threshold at
// min_inlier_ratio, and larger below it.
inline double inlierRatioGateScore(
  double inlier_ratio, double min_inlier_ratio, double score_threshold)
{
  if (!std::isfinite(inlier_ratio)) {
    return std::numeric_limits<double>::infinity();
  }
  return score_threshold * (1.0 - inlier_ratio) / (1.0 - min_inlier_ratio);
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_REGISTRATION_FITNESS_HPP_
