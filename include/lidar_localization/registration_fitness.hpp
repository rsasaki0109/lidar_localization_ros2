#ifndef LIDAR_LOCALIZATION_REGISTRATION_FITNESS_HPP_
#define LIDAR_LOCALIZATION_REGISTRATION_FITNESS_HPP_

#include <pcl/common/transforms.h>
#include <pcl/registration/registration.h>

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

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_REGISTRATION_FITNESS_HPP_
