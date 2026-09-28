// Copyright 2026 Sasaki
// SPDX-License-Identifier: BSD-2-Clause
#include "../src/registration_fitness.hpp"
#include <cassert>

using Cloud = pcl::PointCloud<pcl::PointXYZI>;
class Fixture : public pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>
{
public:
  void emptySource() {input_ = Cloud::Ptr(new Cloud);}
protected:
  void computeTransformation(Cloud & output, const Eigen::Matrix4f & guess) override
  {
    final_transformation_ = guess;
    pcl::transformPointCloud(output, output, guess);
    converged_ = true;
  }
};

int main()
{
  Cloud::Ptr target(new Cloud);
  for (int i = -5; i <= 5; ++i) {
    for (int j = -5; j <= 5; ++j) {
      pcl::PointXYZI point{};
      point.x = 0.3f * i; point.y = 0.2f * j; point.z = 0.1f * (i + j);
      target->push_back(point);
      target->push_back(point);  // Duplicates exercise equal-distance neighbors.
    }
  }
  for (int size : {1, 2, 17, 128}) {
    for (float shift : {0.0f, 0.13f, 100.0f}) {
      Fixture registration;
      registration.setInputTarget(target);
      Cloud::Ptr source(new Cloud);
      for (int i = 0; i < size; ++i) {source->push_back((*target)[i]);}
      registration.setInputSource(source);
      Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
      guess(0, 3) = shift; guess(2, 3) = -shift;
      Cloud output;
      registration.align(output, guess);
      const double expected = registration.getFitnessScore();
      for (int threads : {0, 1, 2, 4}) {
        assert(lidar_localization::orderedParallelFitness(registration, threads) == expected);
      }
      registration.emptySource();
      assert(lidar_localization::orderedParallelFitness(registration, 4) ==
        registration.getFitnessScore());
    }
  }
}
