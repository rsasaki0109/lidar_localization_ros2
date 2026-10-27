#include "lidar_localization/twist_gtsam_smoother.hpp"

#include <cassert>
#include <cmath>

namespace
{

constexpr double kDt = 0.01;

bool near(double a, double b, double tol = 1e-6)
{
  return std::abs(a - b) < tol;
}

double x_of(const Eigen::Matrix4f & pose) {return pose(0, 3);}
double y_of(const Eigen::Matrix4f & pose) {return pose(1, 3);}
double yaw_of(const Eigen::Matrix4f & pose) {return std::atan2(pose(1, 0), pose(0, 0));}

// Feeds 100 Hz twist stamped (first, last] at a constant velocity. Scans below
// are stamped between twist stamps, so no step sits on a scan stamp.
void drive(TwistGtsamSmoother & smoother, double vx, double wz, double first, double last)
{
  for (int step = static_cast<int>(std::lround(first / kDt)) + 1;
    step <= static_cast<int>(std::lround(last / kDt)); ++step)
  {
    smoother.predict(vx, wz, step * kDt);
  }
}

// A scan stamped 1.005 s is aligned while the twist runs on to 1.5 s: the NDT
// pose is the pose at the scan, and the 5 m driven since must stay in the
// prediction.
void test_late_update_keeps_the_motion_after_the_scan()
{
  TwistGtsamSmoother smoother;
  smoother.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  drive(smoother, 10.0, 0.0, 0.0, 1.5);
  assert(near(x_of(smoother.predictedPoseMatrix(0, 0)), 15.0, 1e-3));
  assert(near(x_of(smoother.predictedPoseMatrixAt(0, 0, 1.005)), 10.0, 1e-3));

  assert(smoother.update(10.0, 0.0, 0.0, 0.0, 0.5, 1.005));
  assert(near(smoother.px(), 10.0, 1e-3));
  assert(near(x_of(smoother.predictedPoseMatrix(0, 0)), 15.0, 1e-3));
  assert(near(x_of(smoother.predictedPoseMatrixAt(0, 0, 1.205)), 12.0, 1e-3));
}

// While turning, an update that agrees with the twist leaves the prediction
// where it would be without the update: the steps after the scan are carried
// over to the new node in its own frame.
void test_consistent_update_does_not_move_the_prediction()
{
  TwistGtsamSmoother updated;
  TwistGtsamSmoother reference;
  updated.initialize(1.0, 2.0, 0.0, 0.3, 0.0);
  reference.initialize(1.0, 2.0, 0.0, 0.3, 0.0);
  drive(updated, 8.0, 0.4, 0.0, 1.6);
  drive(reference, 8.0, 0.4, 0.0, 1.6);

  const Eigen::Matrix4f at_scan = updated.predictedPoseMatrixAt(0, 0, 1.005);
  assert(updated.update(x_of(at_scan), y_of(at_scan), 0.0, yaw_of(at_scan), 0.5, 1.005));
  assert(near(updated.px(), x_of(at_scan), 1e-4));
  assert(near(updated.py(), y_of(at_scan), 1e-4));

  const Eigen::Matrix4f expected = reference.predictedPoseMatrix(0, 0);
  const Eigen::Matrix4f predicted = updated.predictedPoseMatrix(0, 0);
  assert(near(x_of(predicted), x_of(expected), 1e-4));
  assert(near(y_of(predicted), y_of(expected), 1e-4));
  assert(near(yaw_of(predicted), yaw_of(expected), 1e-5));
}

// Each scan is aligned 0.3 s after its stamp: the twist received meanwhile
// counts toward the next scan, so the nodes follow the scan stamps.
void test_scan_updates_in_sequence_track_a_constant_velocity()
{
  TwistGtsamSmoother smoother;
  smoother.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  double fed = 0.0;
  for (int scan = 1; scan <= 20; ++scan) {
    const double scan_stamp = 0.1 * scan + 0.005;
    const double driven = 0.1 * scan;
    drive(smoother, 10.0, 0.0, fed, 0.1 * scan + 0.3);
    fed = 0.1 * scan + 0.3;
    assert(near(x_of(smoother.predictedPoseMatrixAt(0, 0, scan_stamp)), 10.0 * driven, 1e-3));
    assert(smoother.update(10.0 * driven, 0.0, 0.0, 0.0, 0.5, scan_stamp));
  }
  assert(near(smoother.px(), 20.0, 1e-3));
  assert(near(x_of(smoother.predictedPoseMatrix(0, 0)), 23.0, 1e-3));
}

// Without scan updates the pending twist is folded, not dropped.
void test_long_prediction_without_updates_keeps_all_motion()
{
  TwistGtsamSmoother smoother;
  smoother.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  drive(smoother, 2.0, 0.0, 0.0, 60.0);
  assert(near(x_of(smoother.predictedPoseMatrix(0, 0)), 120.0, 1e-2));
  assert(near(x_of(smoother.predictedPoseMatrixAt(0, 0, 60.0)), 120.0, 1e-2));
}

}  // namespace

int main()
{
  test_late_update_keeps_the_motion_after_the_scan();
  test_consistent_update_does_not_move_the_prediction();
  test_scan_updates_in_sequence_track_a_constant_velocity();
  test_long_prediction_without_updates_keeps_all_motion();
  return 0;
}
