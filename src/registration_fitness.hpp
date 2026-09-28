// Copyright 2026 Sasaki
// SPDX-License-Identifier: BSD-2-Clause
#ifndef LIDAR_LOCALIZATION_REGISTRATION_FITNESS_HPP_
#define LIDAR_LOCALIZATION_REGISTRATION_FITNESS_HPP_

#include <pcl/registration/registration.h>
#include <pcl/common/transforms.h>
#include <limits>
#include <vector>

namespace lidar_localization
{
// Preserve PCL's default fitness metric, including its original summation order.
// Used only for the native NDT backend with its existing thread budget.
inline double orderedParallelFitness(
  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI> & registration, int threads)
{
  if (threads <= 1) {return registration.getFitnessScore();}
#ifndef _OPENMP
  return registration.getFitnessScore();
#else
  pcl::PointCloud<pcl::PointXYZI> transformed;
  pcl::transformPointCloud(
    *registration.getInputSource(), transformed, registration.getFinalTransformation());
  const auto tree = registration.getSearchMethodTarget();
  std::vector<float> nearest(transformed.size());
#pragma omp parallel num_threads(threads)
  {
    pcl::Indices indices(1);
    std::vector<float> distances(1);
#pragma omp for schedule(static)
    for (std::size_t i = 0; i < transformed.size(); ++i) {
      tree->nearestKSearch(transformed[i], 1, indices, distances);
      nearest[i] = distances[0];
    }
  }
  double sum = 0.0;
  int count = 0;
  for (float distance : nearest) {
    if (distance <= std::numeric_limits<double>::max()) {
      sum += distance;
      ++count;
    }
  }
  return count > 0 ? sum / count : std::numeric_limits<double>::max();
#endif
}
}  // namespace lidar_localization
#endif  // LIDAR_LOCALIZATION_REGISTRATION_FITNESS_HPP_
