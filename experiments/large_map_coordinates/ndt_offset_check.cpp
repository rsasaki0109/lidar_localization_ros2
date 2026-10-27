// Does NDT_OMP accuracy depend on how far the map is from the origin?
// The same synthetic map and scans are registered with the map shifted by
// MGRS- and UTM-sized offsets. Map points, the initial guess and the result
// are float32, as in the localizer. Build and run: see README.md.
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pclomp/ndt_omp.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

static void add_plane(std::vector<Eigen::Vector3d> & pts, const Eigen::Vector3d & o,
  const Eigen::Vector3d & u, const Eigen::Vector3d & v, double lu, double lv, double step)
{
  for (double a = 0; a <= lu; a += step) {
    for (double b = 0; b <= lv; b += step) {pts.push_back(o + a * u + b * v);}
  }
}

static std::vector<Eigen::Vector3d> make_world(std::mt19937 & rng)
{
  std::vector<Eigen::Vector3d> pts;
  const Eigen::Vector3d X = Eigen::Vector3d::UnitX(), Y = Eigen::Vector3d::UnitY(),
    Z = Eigen::Vector3d::UnitZ();
  add_plane(pts, {-50, -50, 0}, X, Y, 100, 100, 0.25);  // ground
  std::uniform_real_distribution<double> h(6, 18), w(6, 15);
  // Building facades along both streets, with varying depth and height.
  for (double x = -50; x < 50; x += 10) {
    const double hh = h(rng), dd = std::uniform_real_distribution<double>(0, 3)(rng);
    add_plane(pts, {x, 12 + dd, 0}, X, Z, 9, hh, 0.2);
    add_plane(pts, {x, -12 - dd, 0}, X, Z, 9, hh, 0.2);
  }
  for (double y = -50; y < 50; y += 12) {
    if (std::abs(y) < 14) {continue;}
    add_plane(pts, {30, y, 0}, Y, Z, 10, h(rng), 0.2);
    add_plane(pts, {-30, y, 0}, Y, Z, 10, h(rng), 0.2);
  }
  // Poles and parked cars.
  std::uniform_real_distribution<double> px(-45, 45), py(-10, 10);
  for (int i = 0; i < 40; ++i) {
    const Eigen::Vector3d c(px(rng), py(rng) > 0 ? 10.5 : -10.5, 0);
    for (double z = 0; z < 5; z += 0.1) {
      for (int k = 0; k < 8; ++k) {
        const double t = k * M_PI / 4;
        pts.push_back(c + Eigen::Vector3d(0.12 * std::cos(t), 0.12 * std::sin(t), z));
      }
    }
  }
  for (int i = 0; i < 25; ++i) {
    const Eigen::Vector3d c(px(rng), py(rng) * 0.8, 0);
    const double l = 4.2, b = 1.8, hh = 1.5;
    add_plane(pts, c, X, Z, l, hh, 0.1);
    add_plane(pts, c + b * Y, X, Z, l, hh, 0.1);
    add_plane(pts, c, Y, Z, b, hh, 0.1);
    add_plane(pts, c + l * X, Y, Z, b, hh, 0.1);
    add_plane(pts, c + hh * Z, X, Y, l, b, 0.1);
  }
  // Blocks behind the facades and short cross walls, so the along-street
  // axis is as well constrained as the others.
  std::uniform_real_distribution<double> bx(-45, 45), by(-45, 45);
  for (int i = 0; i < 60; ++i) {
    const Eigen::Vector3d c(bx(rng), by(rng), 0);
    if (std::abs(c.y()) < 11) {continue;}
    const double l = w(rng) * 0.5, b = w(rng) * 0.4, hh = h(rng) * 0.5;
    add_plane(pts, c, X, Z, l, hh, 0.2);
    add_plane(pts, c + b * Y, X, Z, l, hh, 0.2);
    add_plane(pts, c, Y, Z, b, hh, 0.2);
    add_plane(pts, c + l * X, Y, Z, b, hh, 0.2);
  }
  for (double x = -40; x <= 40; x += 20) {
    add_plane(pts, {x, -11, 0}, Y, Z, 1.5, 3.0, 0.1);
    add_plane(pts, {x + 7, 9.5, 0}, Y, Z, 1.5, 3.0, 0.1);
  }
  return pts;
}

int main()
{
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  std::mt19937 rng(7);
  const auto world = make_world(rng);
  const std::vector<Eigen::Vector3d> offsets = {
    {0, 0, 0}, {1e4, 1e4, 0}, {99950, 99950, 50}, {3.5e5, 3.9e6, 0}};
  std::normal_distribution<double> noise(0.0, 0.02);
  std::uniform_real_distribution<double> ux(-20, 20), uy(-6, 6), uyaw(-M_PI, M_PI),
    upert(-1.0, 1.0);

  struct Trial {Eigen::Isometry3d gt; Eigen::Isometry3d guess_local; Cloud::Ptr scan;};
  std::vector<Trial> trials;
  for (int t = 0; t < 12; ++t) {
    Eigen::Isometry3d gt = Eigen::Isometry3d::Identity();
    gt.translate(Eigen::Vector3d(ux(rng), uy(rng), 1.8));
    gt.rotate(Eigen::AngleAxisd(uyaw(rng), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.02 * upert(rng), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(0.02 * upert(rng), Eigen::Vector3d::UnitY()));
    Cloud::Ptr raw(new Cloud);
    const Eigen::Isometry3d inv = gt.inverse();
    for (const auto & p : world) {
      const Eigen::Vector3d s = inv * p;
      const double r = s.norm();
      if (r < 1.0 || r > 40.0) {continue;}
      pcl::PointXYZI q;
      q.x = s.x() + noise(rng); q.y = s.y() + noise(rng); q.z = s.z() + noise(rng);
      q.intensity = 0;
      raw->push_back(q);
    }
    Cloud::Ptr scan(new Cloud);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setLeafSize(0.5f, 0.5f, 0.5f);
    vg.setInputCloud(raw);
    vg.filter(*scan);
    Eigen::Isometry3d guess = gt;
    guess.pretranslate(Eigen::Vector3d(0.4 * upert(rng), 0.4 * upert(rng), 0.1 * upert(rng)));
    guess.rotate(Eigen::AngleAxisd(0.03 * upert(rng), Eigen::Vector3d::UnitZ()));
    trials.push_back({gt, guess, scan});
  }

  std::vector<Eigen::Vector3d> baseline;
  std::printf("%-26s %10s %10s %10s %10s %12s %12s %6s\n", "offset [m]", "med_t[mm]",
    "max_t[mm]", "med_yaw[mdeg]", "max_yaw", "med_vs0[mm]", "max_vs0[mm]", "conv");
  for (const auto & o : offsets) {
    Cloud::Ptr map(new Cloud);
    for (const auto & p : world) {
      pcl::PointXYZI q;
      q.x = static_cast<float>(p.x() + o.x());
      q.y = static_cast<float>(p.y() + o.y());
      q.z = static_cast<float>(p.z() + o.z());
      q.intensity = 0;
      map->push_back(q);
    }
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt.setResolution(1.0);
    ndt.setStepSize(0.1);
    ndt.setTransformationEpsilon(0.01);
    ndt.setMaximumIterations(35);
    ndt.setNumThreads(4);
    ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt.setInputTarget(map);
    std::vector<double> et, ey, ed;
    int converged = 0;
    size_t trial_index = 0;
    for (const auto & tr : trials) {
      Eigen::Isometry3d guess = tr.guess_local;
      guess.pretranslate(o);
      ndt.setInputSource(tr.scan);
      Cloud out;
      ndt.align(out, guess.matrix().cast<float>());
      converged += ndt.hasConverged();
      const Eigen::Matrix4d est = ndt.getFinalTransformation().cast<double>();
      Eigen::Isometry3d truth = tr.gt;
      truth.pretranslate(o);
      // Compare in double after removing the offset, so the error measure
      // itself does not lose precision. vs0 is the change against the same
      // trial at offset 0: what the offset alone does to the result.
      const Eigen::Vector3d dt = (est.block<3, 1>(0, 3) - o) - (truth.translation() - o);
      const Eigen::Matrix3d dR = truth.rotation().transpose() * est.block<3, 3>(0, 0);
      const Eigen::Vector3d local_est = est.block<3, 1>(0, 3) - o;
      if (o.isZero()) {baseline.push_back(local_est);}
      ed.push_back((local_est - baseline[trial_index++]).norm() * 1e3);
      et.push_back(dt.norm() * 1e3);
      ey.push_back(std::abs(std::atan2(dR(1, 0), dR(0, 0))) * 180.0 / M_PI * 1e3);
    }
    auto med = [](std::vector<double> v) {
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
      };
    std::printf("(%9.0f, %9.0f, %4.0f)  %10.1f %10.1f %10.1f %10.1f %12.2f %12.2f %3d/%zu\n",
      o.x(), o.y(), o.z(), med(et), *std::max_element(et.begin(), et.end()), med(ey),
      *std::max_element(ey.begin(), ey.end()), med(ed), *std::max_element(ed.begin(), ed.end()),
      converged, trials.size());
  }
  return 0;
}
