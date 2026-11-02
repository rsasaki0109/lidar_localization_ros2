// Time ImuGtsamSmoother::update on a synthetic drive: 10 m/s along x, 20 IMU samples
// (200 Hz) per 10 Hz scan, NDT poses with 2 cm noise, 120 scans. The 50-pose window
// fills at scan 50. Prints the median and max update time per stretch and the final
// pose, so two builds of the header can be compared for speed and for the result.
#include "lidar_localization/imu_gtsam_smoother.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <random>
#include <vector>

int main()
{
  ImuGtsamSmoother smoother;
  smoother.initialize(0, 0, 0, 0, 0, 0, 10.0, 0, 0, 0.0);
  std::mt19937 rng(1);
  std::normal_distribution<double> noise(0.0, 0.02);
  std::vector<double> ms;
  double t = 0.0;
  for (int scan = 1; scan <= 120; ++scan) {
    for (int k = 0; k < 20; ++k) {
      smoother.integrateImu(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 9.81), 0.005);
    }
    t += 0.1;
    const auto start = std::chrono::steady_clock::now();
    smoother.update(10.0 * t + noise(rng), noise(rng), noise(rng), 0, 0, 0, 0.5, t);
    const auto end = std::chrono::steady_clock::now();
    ms.push_back(std::chrono::duration<double, std::milli>(end - start).count());
  }
  const auto stats = [&ms](int first, int last) {
    std::vector<double> v(ms.begin() + first, ms.begin() + last);
    std::sort(v.begin(), v.end());
    std::printf(
      "scans %3d-%3d: median %.1f ms, max %.1f ms\n", first + 1, last, v[v.size() / 2],
      v.back());
  };
  stats(0, 10);
  stats(10, 50);
  stats(50, 120);
  const auto pose = smoother.poseMatrix();
  std::printf("final pose x %.9f y %.9f z %.9f\n", pose(0, 3), pose(1, 3), pose(2, 3));
  return 0;
}
