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

// Feeds 100 Hz level twist stamped (first, last]. Scans below are stamped
// between twist stamps.
void drive_100hz(TwistEkf & ekf, double vx, double wz, double first, double last)
{
  for (int step = static_cast<int>(std::lround(first / 0.01)) + 1;
    step <= static_cast<int>(std::lround(last / 0.01)); ++step)
  {
    ekf.predict(vx, wz, step * 0.01);
  }
}

double yaw_of(const Eigen::Matrix4f & pose)
{
  return std::atan2(pose(1, 0), pose(0, 0));
}

// A scan stamped 1.005 s is aligned while the twist runs on to 1.5 s: a result
// that agrees with the twist leaves the 5 m driven since in the state.
void test_late_update_keeps_the_motion_after_the_scan()
{
  TwistEkf ekf;
  ekf.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  drive_100hz(ekf, 10.0, 0.0, 0.0, 1.5);
  assert(near(ekf.poseMatrixAt(0.0, 0.0, 1.005)(0, 3), 10.0, 1e-4));
  assert(ekf.update(10.0, 0.0, 0.0, 0.0, 0.5, 1.005));
  assert(near(ekf.px(), 15.0, 1e-6));
  assert(near(ekf.poseMatrixAt(0.0, 0.0, 1.005)(0, 3), 10.0, 1e-4));

  // Without a stamp the same result is taken as the current pose.
  TwistEkf unstamped;
  unstamped.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  drive_100hz(unstamped, 10.0, 0.0, 0.0, 1.5);
  assert(unstamped.update(10.0, 0.0, 0.0, 0.0, 0.5));
  assert(unstamped.px() < 11.0);
}

// A result 1 m to the side of the scan pose moves the scan pose and the current
// state by the same correction.
void test_offset_update_corrects_the_scan_pose_and_the_state_alike()
{
  TwistEkf ekf;
  ekf.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  drive_100hz(ekf, 10.0, 0.0, 0.0, 1.5);
  assert(ekf.update(10.0, 1.0, 0.0, 0.0, 0.5, 1.005));
  assert(ekf.py() > 0.9 && ekf.py() < 1.0);
  assert(near(ekf.poseMatrixAt(0.0, 0.0, 1.005)(1, 3), ekf.py(), 1e-4));
}

// While turning, a result that agrees with the twist leaves the state where it
// would be without the update.
void test_consistent_update_while_turning_does_not_move_the_state()
{
  TwistEkf updated;
  TwistEkf reference;
  updated.initialize(1.0, 2.0, 0.0, 0.3, 0.0);
  reference.initialize(1.0, 2.0, 0.0, 0.3, 0.0);
  drive_100hz(updated, 8.0, 0.4, 0.0, 1.6);
  drive_100hz(reference, 8.0, 0.4, 0.0, 1.6);
  const Eigen::Matrix4f at_scan = updated.poseMatrixAt(0.0, 0.0, 1.005);
  assert(updated.update(at_scan(0, 3), at_scan(1, 3), at_scan(2, 3), yaw_of(at_scan), 0.5, 1.005));
  assert(near(updated.px(), reference.px(), 1e-4));
  assert(near(updated.py(), reference.py(), 1e-4));
  assert(near(updated.yaw(), reference.yaw(), 1e-5));
}

}  // namespace

int main()
{
  test_level_straight_line_is_planar();
  test_climbing_a_slope_moves_up_and_less_forward();
  test_roll_and_pitch_scale_the_heading_rate();
  test_covariance_stays_symmetric_on_a_slope();
  test_late_update_keeps_the_motion_after_the_scan();
  test_offset_update_corrects_the_scan_pose_and_the_state_alike();
  test_consistent_update_while_turning_does_not_move_the_state();
  return 0;
}
