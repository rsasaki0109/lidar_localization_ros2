#include "lidar_localization/twist_ekf.hpp"

#include <cassert>
#include <cmath>

namespace
{

constexpr double kDt = 0.1;

bool near(double a, double b, double tol = 1e-9)
{
  return std::abs(a - b) < tol;
}

double deg(double d)
{
  return d * M_PI / 180.0;
}

// Drives for steps * kDt seconds at a constant twist and attitude.
void drive(TwistEkf & ekf, double vx, double wz, int steps, double roll, double pitch)
{
  for (int i = 1; i <= steps; ++i) {
    ekf.predict(vx, wz, i * kDt, roll, pitch);
  }
}

void test_level_straight_line_is_planar()
{
  TwistEkf ekf;
  ekf.initialize(0.0, 0.0, 0.0, 0.5, 0.0);
  drive(ekf, 1.0, 0.0, 10, 0.0, 0.0);
  assert(near(ekf.px(), std::cos(0.5)));
  assert(near(ekf.py(), std::sin(0.5)));
  assert(ekf.pz() == 0.0);
  assert(ekf.vz() == 0.0);
  assert(near(ekf.yaw(), 0.5));
}

void test_climbing_a_slope_moves_up_and_less_forward()
{
  // Negative pitch tilts the nose up.
  const double pitch = deg(-10.0);
  TwistEkf ekf;
  ekf.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  drive(ekf, 1.0, 0.0, 20, 0.0, pitch);
  assert(near(ekf.px(), 2.0 * std::cos(pitch)));
  assert(near(ekf.py(), 0.0));
  assert(near(ekf.pz(), 2.0 * std::sin(-pitch)));
  assert(near(ekf.vz(), std::sin(-pitch)));
}

void test_roll_and_pitch_scale_the_heading_rate()
{
  TwistEkf rolled;
  rolled.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  drive(rolled, 0.0, 0.1, 10, deg(20.0), 0.0);
  assert(near(rolled.yaw(), 0.1 * std::cos(deg(20.0))));

  TwistEkf pitched;
  pitched.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  drive(pitched, 0.0, 0.1, 10, 0.0, deg(30.0));
  assert(near(pitched.yaw(), 0.1 / std::cos(deg(30.0))));
}

void test_covariance_stays_symmetric_on_a_slope()
{
  TwistEkf ekf;
  ekf.initialize(0.0, 0.0, 0.0, 0.7, 0.0);
  drive(ekf, 1.5, 0.3, 50, deg(5.0), deg(-12.0));
  const auto & P = ekf.covariance();
  assert((P - P.transpose()).cwiseAbs().maxCoeff() < 1e-12);
  // A speed bias error now also moves z.
  assert(std::abs(P(2, 8)) > 0.0);
}

}  // namespace

int main()
{
  test_level_straight_line_is_planar();
  test_climbing_a_slope_moves_up_and_less_forward();
  test_roll_and_pitch_scale_the_heading_rate();
  test_covariance_stays_symmetric_on_a_slope();
  return 0;
}
