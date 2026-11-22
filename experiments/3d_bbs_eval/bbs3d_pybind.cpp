// Evaluation-only pybind11 binding of 3D-BBS (CPU), https://github.com/KOKIAOKI/3d_bbs
// (MIT License, Copyright (c) 2023 KokiAoki). Not part of the package build: see
// experiments/3d_bbs_eval/README.md.
//
// Two searches over the same hierarchical voxel maps:
//   * localize(): the library's own cpu::BBS3D::localize(), the single best pose.
//   * localize_topk(): the same branch-and-bound with the library's VoxelMaps and
//     DiscreteTransformation, kept running after the first leaf so that it returns
//     the k best leaves that are not suppressed by an earlier, better one. The
//     angular split, initial set and voxel scoring below follow cpu_bbs3d/bbs3d.cpp
//     (private there), adapted under the MIT licence above. With k = 1 it returns
//     the library's answer (up to ties).

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <queue>
#include <stdexcept>
#include <vector>

#include <cpu_bbs3d/bbs3d.hpp>
#include <cpu_bbs3d/voxelmaps.hpp>

namespace py = pybind11;
using Trans = DiscreteTransformation<double>;

namespace {

std::vector<Eigen::Vector3d> to_points(
  const py::array_t<double, py::array::c_style | py::array::forcecast>& array)
{
  if (array.ndim() != 2 || array.shape(1) != 3) {
    throw std::invalid_argument("points must be an (N, 3) array");
  }
  auto view = array.unchecked<2>();
  std::vector<Eigen::Vector3d> points(view.shape(0));
  for (py::ssize_t i = 0; i < view.shape(0); ++i) {
    points[i] = Eigen::Vector3d(view(i, 0), view(i, 1), view(i, 2));
  }
  return points;
}

struct AngularLevel
{
  Eigen::Vector3i num_division;
  Eigen::Vector3d rpy_res;
  Eigen::Vector3d min_rpy;
};

class Searcher
{
public:
  Searcher(
    const py::array_t<double, py::array::c_style | py::array::forcecast>& map_points,
    double min_level_res, int max_level, int num_threads)
  : min_level_res_(min_level_res), max_level_(max_level), num_threads_(num_threads)
  {
    const auto points = to_points(map_points);
    const auto start = std::chrono::steady_clock::now();
    bbs_.set_num_threads(num_threads);
    bbs_.set_tar_points(points, min_level_res, max_level);
    bbs_.set_trans_search_range(points);
    build_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    // A second copy of the hierarchy for localize_topk (the library keeps its own
    // private); built the same way.
    voxelmaps_ = std::make_unique<cpu::VoxelMaps>();
    voxelmaps_->set_min_res(min_level_res);
    voxelmaps_->set_max_level(max_level);
    voxelmaps_->create_voxelmaps(points, kVRate);
    const auto range = bbs_.get_trans_search_range();
    set_trans_range(range.first, range.second);
  }

  void set_trans_range(const Eigen::Vector3d & min_xyz, const Eigen::Vector3d & max_xyz)
  {
    bbs_.set_trans_search_range(min_xyz, max_xyz);
    min_xyz_ = min_xyz;
    max_xyz_ = max_xyz;
  }

  void set_angular_range(const Eigen::Vector3d & min_rpy, const Eigen::Vector3d & max_rpy)
  {
    bbs_.set_angular_search_range(min_rpy, max_rpy);
    min_rpy_ = min_rpy;
    max_rpy_ = max_rpy;
  }

  double build_ms() const { return build_ms_; }

  std::size_t voxel_count() const
  {
    std::size_t total = 0;
    for (const auto & buckets : voxelmaps_->multi_buckets_) {
      total += buckets.size();
    }
    return total;
  }

  py::dict localize(
    const py::array_t<double, py::array::c_style | py::array::forcecast>& scan,
    double score_threshold_percentage, int timeout_ms)
  {
    bbs_.set_src_points(to_points(scan));
    bbs_.set_score_threshold_percentage(score_threshold_percentage);
    if (timeout_ms > 0) {
      bbs_.enable_timeout();
      bbs_.set_timeout_duration_in_msec(timeout_ms);
    } else {
      bbs_.disable_timeout();
    }
    bbs_.localize();
    py::dict out;
    out["localized"] = bbs_.has_localized();
    out["timed_out"] = bbs_.has_timed_out();
    out["elapsed_ms"] = bbs_.get_elapsed_time();
    out["score"] = bbs_.get_best_score();
    out["score_percentage"] = bbs_.get_best_score_percentage();
    out["pose"] = Eigen::Matrix4d(bbs_.get_global_pose());
    return out;
  }

  // k best leaves; a leaf within nms_radius_m (3D) and nms_yaw_rad of an accepted
  // one is dropped. Leaves come off the best-first queue in score order. Once the
  // best leaf is found, nodes below relative_floor x its score are pruned (without
  // it, separating k peaks expands every near-best leaf and takes minutes on a
  // large map). On a timeout the leaves accepted so far are returned.
  py::dict localize_topk(
    const py::array_t<double, py::array::c_style | py::array::forcecast>& scan,
    int k, double nms_radius_m, double nms_yaw_rad, double min_score_percentage,
    int timeout_ms, double relative_floor)
  {
    const auto points = to_points(scan);
    const auto start = std::chrono::steady_clock::now();
    const auto limit = start + std::chrono::milliseconds(timeout_ms > 0 ? timeout_ms : 1 << 30);
    std::vector<AngularLevel> levels(max_level_ + 1);
    angular_levels(points, levels);

    int threshold = static_cast<int>(std::floor(points.size() * min_score_percentage));
    const double top_res = voxelmaps_->voxelmaps_res_[max_level_];
    std::vector<Trans> init;
    const auto & top = levels[max_level_];
    for (int tx = static_cast<int>(std::floor(min_xyz_.x() / top_res));
      tx <= static_cast<int>(std::ceil(max_xyz_.x() / top_res)); ++tx)
    {
      for (int ty = static_cast<int>(std::floor(min_xyz_.y() / top_res));
        ty <= static_cast<int>(std::ceil(max_xyz_.y() / top_res)); ++ty)
      {
        for (int tz = static_cast<int>(std::floor(min_xyz_.z() / top_res));
          tz <= static_cast<int>(std::ceil(max_xyz_.z() / top_res)); ++tz)
        {
          for (int r = 0; r < top.num_division.x(); ++r) {
            for (int p = 0; p < top.num_division.y(); ++p) {
              for (int y = 0; y < top.num_division.z(); ++y) {
                init.emplace_back(0, max_level_, tx, ty, tz, r, p, y);
              }
            }
          }
        }
      }
    }
    score_all(init, max_level_, levels, points);

    std::priority_queue<Trans> queue;
    for (const auto & t : init) {
      if (t.score >= threshold) {
        queue.push(t);
      }
    }
    std::vector<Trans> accepted;
    std::vector<Eigen::Matrix4d> poses;
    std::size_t expanded = 0;
    std::size_t max_queue = queue.size();
    bool timed_out = false;
    while (!queue.empty() && static_cast<int>(accepted.size()) < k) {
      if (std::chrono::steady_clock::now() > limit) {
        timed_out = true;
        break;
      }
      Trans trans = queue.top();
      queue.pop();
      if (trans.score < threshold) {
        continue;
      }
      if (trans.is_leaf()) {
        const Eigen::Matrix4d pose =
          trans.create_matrix(min_level_res_, levels[0].rpy_res, levels[0].min_rpy);
        bool suppressed = false;
        for (const auto & other : poses) {
          const double distance = (pose.block<3, 1>(0, 3) - other.block<3, 1>(0, 3)).norm();
          const double yaw_a = std::atan2(pose(1, 0), pose(0, 0));
          const double yaw_b = std::atan2(other(1, 0), other(0, 0));
          const double yaw_diff = std::abs(std::remainder(yaw_a - yaw_b, 2.0 * M_PI));
          if (distance < nms_radius_m && yaw_diff < nms_yaw_rad) {
            suppressed = true;
            break;
          }
        }
        if (!suppressed) {
          if (accepted.empty()) {
            threshold = std::max(
              threshold, static_cast<int>(std::ceil(relative_floor * trans.score)));
          }
          accepted.push_back(trans);
          poses.push_back(pose);
        }
        continue;
      }
      ++expanded;
      const int child_level = trans.level - 1;
      auto children = trans.branch(child_level, kVRate, levels[child_level].num_division);
      score_all(children, child_level, levels, points);
      for (const auto & child : children) {
        if (child.score >= threshold) {
          queue.push(child);
        }
      }
      max_queue = std::max(max_queue, queue.size());
    }
    py::dict out;
    py::list candidates;
    for (std::size_t i = 0; i < accepted.size(); ++i) {
      py::dict c;
      c["pose"] = poses[i];
      c["score"] = accepted[i].score;
      c["score_percentage"] = points.empty() ? 0.0 :
        static_cast<double>(accepted[i].score) / points.size();
      candidates.append(c);
    }
    out["candidates"] = candidates;
    out["timed_out"] = timed_out;
    out["expanded"] = expanded;
    out["max_queue"] = max_queue;
    out["initial"] = init.size();
    out["elapsed_ms"] = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    return out;
  }

private:
  static constexpr int kVRate = 2;

  void angular_levels(const std::vector<Eigen::Vector3d> & points, std::vector<AngularLevel> & levels) const
  {
    double max_norm = 0.0;
    for (const auto & p : points) {
      max_norm = std::max(max_norm, p.norm());
    }
    for (int i = max_level_; i >= 0; --i) {
      const double res = voxelmaps_->voxelmaps_res_[i];
      const double cosine = 1.0 - (res * res / (max_norm * max_norm)) * 0.5;
      double ori_res = std::acos(std::max(cosine, -1.0));
      ori_res = std::floor(ori_res * 10000) / 10000;
      Eigen::Vector3d span = max_rpy_ - min_rpy_;
      Eigen::Vector3d piece;
      for (int a = 0; a < 3; ++a) {
        const double tmp = ori_res <= std::abs(span[a]) ? ori_res : 0.0;
        if (i == max_level_) {
          piece[a] = span[a];
        } else {
          piece[a] = levels[i + 1].rpy_res[a] != 0.0 ? levels[i + 1].rpy_res[a] : span[a];
        }
        levels[i].num_division[a] = tmp == 0.0 ? 1 : static_cast<int>(std::ceil(piece[a] / tmp));
        levels[i].rpy_res[a] = levels[i].num_division[a] == 1 ? 0.0 : piece[a] / levels[i].num_division[a];
        levels[i].min_rpy[a] = levels[i].rpy_res[a] == 0.0 ? 0.0 : min_rpy_[a];
      }
    }
  }

  void score_all(
    std::vector<Trans> & set, int level, const std::vector<AngularLevel> & levels,
    const std::vector<Eigen::Vector3d> & points) const
  {
    const auto & buckets = voxelmaps_->multi_buckets_[level];
    const int num_buckets = static_cast<int>(buckets.size());
    const int scan_count = voxelmaps_->get_max_bucket_scan_count();
    const double res = voxelmaps_->voxelmaps_res_[level];
    const double inv_res = 1.0 / res;
    const auto & rpy_res = levels[level].rpy_res;
    const auto & min_rpy = levels[level].min_rpy;
#pragma omp parallel for num_threads(num_threads_)
    for (int s = 0; s < static_cast<int>(set.size()); ++s) {
      Trans & trans = set[s];
      const Eigen::Transform<double, 3, Eigen::Affine> transform(
        trans.create_matrix(res, rpy_res, min_rpy));
      int score = 0;
      for (const auto & point : points) {
        const Eigen::Vector3i coord = ((transform * point).array() * inv_res).floor().cast<int>();
        const std::uint32_t hash =
          (coord[0] * 73856093) ^ (coord[1] * 19349669) ^ (coord[2] * 83492791);
        for (int j = 0; j < scan_count; ++j) {
          const auto & bucket = buckets[(hash + j) % num_buckets];
          if (bucket.w() == 0) {
            break;
          }
          if (bucket.x() == coord.x() && bucket.y() == coord.y() && bucket.z() == coord.z()) {
            ++score;
            break;
          }
        }
      }
      trans.score = score;
    }
  }

  cpu::BBS3D bbs_;
  std::unique_ptr<cpu::VoxelMaps> voxelmaps_;
  double min_level_res_;
  int max_level_;
  int num_threads_;
  double build_ms_ = 0.0;
  Eigen::Vector3d min_xyz_, max_xyz_;
  Eigen::Vector3d min_rpy_{-0.02, -0.02, 0.0};
  Eigen::Vector3d max_rpy_{0.02, 0.02, 2.0 * M_PI};
};

}  // namespace

PYBIND11_MODULE(bbs3d_py, m)
{
  m.doc() = "Evaluation binding of 3D-BBS (CPU) for experiments/3d_bbs_eval";
  py::class_<Searcher>(m, "Searcher")
  .def(py::init<const py::array_t<double, py::array::c_style | py::array::forcecast>&, double, int, int>(),
    py::arg("map_points"), py::arg("min_level_res"), py::arg("max_level"), py::arg("num_threads") = 4)
  .def("set_trans_range", &Searcher::set_trans_range)
  .def("set_angular_range", &Searcher::set_angular_range)
  .def("build_ms", &Searcher::build_ms)
  .def("voxel_count", &Searcher::voxel_count)
  .def("localize", &Searcher::localize, py::arg("scan"),
    py::arg("score_threshold_percentage") = 0.0, py::arg("timeout_ms") = 0)
  .def("localize_topk", &Searcher::localize_topk, py::arg("scan"), py::arg("k") = 8,
    py::arg("nms_radius_m") = 1.0, py::arg("nms_yaw_rad") = 0.35,
    py::arg("min_score_percentage") = 0.0, py::arg("timeout_ms") = 0,
    py::arg("relative_floor") = 0.0);
}
