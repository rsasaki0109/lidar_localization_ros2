#ifndef LIDAR_LOCALIZATION_REGISTRATION_FITNESS_HPP_
#define LIDAR_LOCALIZATION_REGISTRATION_FITNESS_HPP_

#include <pcl/common/transforms.h>
#include <pcl/registration/registration.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace lidar_localization
{

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
  pcl::PointCloud<PointSource> transformed;
  pcl::transformPointCloud(*source, transformed, registration.getFinalTransformation());
  const int count = static_cast<int>(transformed.size());
  std::vector<float> distances(transformed.size(), std::numeric_limits<float>::quiet_NaN());
#pragma omp parallel for num_threads(threads) schedule(guided, 32)
  for (int i = 0; i < count; ++i) {
    pcl::Indices index(1);
    std::vector<float> distance(1);
    if (tree->nearestKSearch(transformed[i], 1, index, distance) > 0) {
      distances[i] = distance[0];
    }
  }
  double fitness = 0.0;
  int matched = 0;
  for (const float distance : distances) {
    // A point without a neighbour is skipped; PCL would read an unset distance there.
    if (!std::isnan(distance)) {
      fitness += distance;
      ++matched;
    }
  }
  return matched > 0 ? fitness / matched : std::numeric_limits<double>::max();
}

// The share of the source points, moved by the final transformation, whose nearest
// target point lies within inlier_distance_m. Unlike the fitness score, points that see
// structure missing from the map do not drag it down beyond their count.
template<typename PointSource, typename PointTarget, typename Scalar>
double registrationInlierRatio(
  pcl::Registration<PointSource, PointTarget, Scalar> & registration, int threads,
  double inlier_distance_m)
{
  const auto source = registration.getInputSource();
  const auto tree = registration.getSearchMethodTarget();
  if (!source || source->empty() || !tree || !tree->getInputCloud()) {
    return 0.0;
  }
  pcl::PointCloud<PointSource> transformed;
  pcl::transformPointCloud(*source, transformed, registration.getFinalTransformation());
  const int count = static_cast<int>(transformed.size());
  const float limit = static_cast<float>(inlier_distance_m * inlier_distance_m);
  int inliers = 0;
#pragma omp parallel for num_threads(std::max(1, threads)) schedule(guided, 32) reduction(+ : inliers)
  for (int i = 0; i < count; ++i) {
    pcl::Indices index(1);
    std::vector<float> distance(1);
    if (tree->nearestKSearch(transformed[i], 1, index, distance) > 0 && distance[0] <= limit) {
      ++inliers;
    }
  }
  return static_cast<double>(inliers) / count;
}

// The inlier ratio on the fitness score's scale, for the measurement gate: 0 when every
// point is an inlier, score_threshold at min_inlier_ratio, larger below it.
inline double inlierRatioGateScore(
  double inlier_ratio, double min_inlier_ratio, double score_threshold)
{
  return score_threshold * (1.0 - inlier_ratio) / (1.0 - min_inlier_ratio);
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_REGISTRATION_FITNESS_HPP_
