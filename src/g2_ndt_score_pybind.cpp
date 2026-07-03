// Optional pybind11 module exposing per-candidate NDT registration scoring for
// the G2 global-localization service. When importable, GlobalLocalizationEngine
// can re-rank BBS candidates by NDT fitness instead of occupancy score alone.

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <limits>
#include <string>

#include "lidar_localization/g2_ndt_candidate_score.hpp"

namespace py = pybind11;
namespace g2 = lidar_localization::g2_ndt;

namespace {

struct ScoreResultPy
{
  double fitness;
  bool converged;
  std::size_t target_point_count;
  std::size_t source_point_count;
  double refined_x;
  double refined_y;
  double refined_z;
  double refined_yaw;
};

class MapNdtScorerPy
{
public:
  MapNdtScorerPy(
    const std::string & map_path,
    double ndt_resolution,
    double ndt_step_size,
    double transform_epsilon,
    int max_iterations,
    int num_threads,
    double scan_voxel_leaf_size,
    double target_voxel_leaf_size,
    double local_map_radius,
    std::size_t min_target_points)
  : scorer_(map_path, g2::G2NdtScoreParams{
      ndt_resolution,
      ndt_step_size,
      transform_epsilon,
      max_iterations,
      num_threads,
      scan_voxel_leaf_size,
      target_voxel_leaf_size,
      local_map_radius,
      min_target_points})
  {
  }

  ScoreResultPy score_candidate(
    py::array_t<double, py::array::c_style | py::array::forcecast> scan_xyz,
    double x,
    double y,
    double z,
    double yaw) const
  {
    const auto info = scan_xyz.request();
    if (info.ndim != 2 || info.shape[1] != 3) {
      throw std::invalid_argument("scan_xyz must have shape (N, 3)");
    }
    const auto * ptr = static_cast<const double *>(info.ptr);
    const std::size_t count = static_cast<std::size_t>(info.shape[0]);
    g2::G2NdtScoreResult result;
    {
      py::gil_scoped_release release;
      result = scorer_.score_xyz(ptr, count, x, y, z, yaw);
    }
    return ScoreResultPy{
      result.fitness,
      result.converged,
      result.target_point_count,
      result.source_point_count,
      result.refined_x,
      result.refined_y,
      result.refined_z,
      result.refined_yaw,
    };
  }

  std::vector<ScoreResultPy> score_candidates(
    py::array_t<double, py::array::c_style | py::array::forcecast> scan_xyz,
    py::list poses) const
  {
    std::vector<ScoreResultPy> results;
    results.reserve(poses.size());
    for (py::handle item : poses) {
      py::tuple pose = py::cast<py::tuple>(item);
      if (pose.size() != 4) {
        throw std::invalid_argument("each pose must be (x, y, z, yaw)");
      }
      results.push_back(score_candidate(
        scan_xyz,
        py::cast<double>(pose[0]),
        py::cast<double>(pose[1]),
        py::cast<double>(pose[2]),
        py::cast<double>(pose[3])));
    }
    return results;
  }

private:
  g2::G2NdtCandidateScorer scorer_;
};

}  // namespace

PYBIND11_MODULE(g2_ndt_score, m)
{
  m.doc() = "Per-candidate NDT registration scoring for G2 global localization";

  py::class_<ScoreResultPy>(m, "ScoreResult")
    .def_readonly("fitness", &ScoreResultPy::fitness)
    .def_readonly("converged", &ScoreResultPy::converged)
    .def_readonly("target_point_count", &ScoreResultPy::target_point_count)
    .def_readonly("source_point_count", &ScoreResultPy::source_point_count)
    .def_readonly("refined_x", &ScoreResultPy::refined_x)
    .def_readonly("refined_y", &ScoreResultPy::refined_y)
    .def_readonly("refined_z", &ScoreResultPy::refined_z)
    .def_readonly("refined_yaw", &ScoreResultPy::refined_yaw);

  py::class_<MapNdtScorerPy>(m, "MapNdtScorer")
    .def(
      py::init<
        const std::string &,
        double,
        double,
        double,
        int,
        int,
        double,
        double,
        double,
        std::size_t>(),
      py::arg("map_path"),
      py::arg("ndt_resolution") = 1.0,
      py::arg("ndt_step_size") = 0.1,
      py::arg("transform_epsilon") = 0.01,
      py::arg("max_iterations") = 30,
      py::arg("num_threads") = 1,
      py::arg("scan_voxel_leaf_size") = 1.0,
      py::arg("target_voxel_leaf_size") = 1.0,
      py::arg("local_map_radius") = 150.0,
      py::arg("min_target_points") = 100)
    .def("score_candidate", &MapNdtScorerPy::score_candidate)
    .def("score_candidates", &MapNdtScorerPy::score_candidates);
}
