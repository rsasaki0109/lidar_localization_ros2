#include "lidar_localization/twist_gtsam_smoother.hpp"

#include <cassert>
#include <cmath>

namespace
{

constexpr double kDt = 0.1;

bool near(double a, double b, double tol = 1e-6)
{
  return std::abs(a - b) < tol;
}

double deg(double d)
{
  return d * M_PI / 180.0;
}

// Accumulates steps * kDt seconds of constant twist after `start`.
void drive(
  TwistGtsamSmoother & smoother, double start, double vx, double wz, int steps,
  double roll, double pitch)
{
  for (int i = 1; i <= steps; ++i) {
    smoother.predict(vx, wz, start + i * kDt, roll, pitch);
  }
}

void test_level_prediction_is_planar()
{
  TwistGtsamSmoother smoother;
  smoother.initialize(0.0, 0.0, 1.5, 0.3, 0.0);
  drive(smoother, 0.0, 1.0, 0.0, 10, 0.0, 0.0);
  const Eigen::Matrix4f predicted = smoother.predictedPoseMatrix(0.0f, 0.0f);
  assert(near(predicted(0, 3), std::cos(0.3)));
  assert(near(predicted(1, 3), std::sin(0.3)));
  assert(predicted(2, 3) == 1.5f);
}

void test_climbing_moves_the_predicted_seed_up()
{
  // Negative pitch tilts the nose up.
  const double pitch = deg(-10.0);
  TwistGtsamSmoother smoother;
  smoother.initialize(0.0, 0.0, 1.0, 0.0, 0.0);
  drive(smoother, 0.0, 1.0, 0.0, 20, 0.0, pitch);
  const Eigen::Matrix4f predicted =
    smoother.predictedPoseMatrix(0.0f, static_cast<float>(pitch));
  assert(near(predicted(0, 3), 2.0 * std::cos(pitch)));
  assert(near(predicted(1, 3), 0.0));
  assert(near(predicted(2, 3), 1.0 + 2.0 * std::sin(-pitch)));
}

void test_update_takes_z_from_ndt_and_restarts_the_slope()
{
  const double pitch = deg(-10.0);
  TwistGtsamSmoother smoother;
  smoother.initialize(0.0, 0.0, 1.0, 0.0, 0.0);
  drive(smoother, 0.0, 1.0, 0.0, 20, 0.0, pitch);
  // NDT agrees with the slope-aware odometry.
  const double x = 2.0 * std::cos(pitch);
  const double z = 1.0 + 2.0 * std::sin(-pitch);
  assert(smoother.update(x, 0.0, z, 0.0, 1.0, 2.0));
  assert(near(smoother.px(), x));
  assert(near(smoother.pz(), z));
  // Right after the update there is no accumulated motion left.
  const Eigen::Matrix4f predicted =
    smoother.predictedPoseMatrix(0.0f, static_cast<float>(pitch));
  assert(near(predicted(0, 3), x, 1e-5));
  assert(near(predicted(2, 3), z, 1e-5));
}

void test_roll_and_pitch_scale_the_heading_rate()
{
  TwistGtsamSmoother rolled;
  rolled.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  drive(rolled, 0.0, 0.0, 0.1, 10, deg(20.0), 0.0);
  const Eigen::Matrix4f rolled_pose = rolled.predictedPoseMatrix(0.0f, 0.0f);
  assert(near(std::atan2(rolled_pose(1, 0), rolled_pose(0, 0)), 0.1 * std::cos(deg(20.0))));

  TwistGtsamSmoother pitched;
  pitched.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  drive(pitched, 0.0, 0.0, 0.1, 10, 0.0, deg(30.0));
  const Eigen::Matrix4f pitched_pose = pitched.predictedPoseMatrix(0.0f, 0.0f);
  assert(near(std::atan2(pitched_pose(1, 0), pitched_pose(0, 0)), 0.1 / std::cos(deg(30.0))));
}

}  // namespace

int main()
{
  test_level_prediction_is_planar();
  test_climbing_moves_the_predicted_seed_up();
  test_update_takes_z_from_ndt_and_restarts_the_slope();
  test_roll_and_pitch_scale_the_heading_rate();
  return 0;
}
