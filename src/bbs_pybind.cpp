// pybind11 binding that exposes the bit-exact C++ branch-and-bound search
// (include/lidar_localization/bbs_branch_and_bound.hpp) to Python as the module
// `bbs_cpp`, so the runtime global-localization node can opt into the fast C++
// backend while keeping the pure-Python path as the default fallback.
//
// The module is OPTIONAL: it is only built when pybind11 is found at configure
// time (CMakeLists.txt gates it behind find_package(pybind11 QUIET)), and the
// engine imports it lazily and falls back to Python on ImportError. So nothing
// here is on the critical path of a default build.
//
// The returned BbsGridCandidate mirrors the dataclass field-for-field
// (scripts/make_bbs_relocalization_attempts.py:BbsGridCandidate) so
// GlobalLocalizationEngine.query() can consume either backend unchanged.
//
// Correctness is established offline: the C++ core is proven bit-exact against
// the Python reference by test/test_bbs_branch_and_bound.cpp (golden fixtures),
// and test/test_bbs_cpp_backend_parity.py re-checks this binding's numpy<->struct
// plumbing wherever the compiled module is importable.

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <array>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "lidar_localization/bbs_branch_and_bound.hpp"

namespace py = pybind11;
namespace bbs = lidar_localization::bbs;

namespace {

// occupancy: 2D boolean/uint8 grid (row-major). scan_xy_m: (N, 2) float64 in the
// sensor frame, metres. The remaining args mirror the Python signature exactly.
std::vector<bbs::BbsGridCandidate> branch_and_bound_candidates_py(
  py::array_t<uint8_t, py::array::c_style | py::array::forcecast> occupancy,
  py::array_t<double, py::array::c_style | py::array::forcecast> scan_xy_m,
  double resolution_m,
  double angular_resolution_rad,
  int pyramid_depth,
  int max_candidates,
  int nms_radius_cells)
{
  const auto occ_info = occupancy.request();
  if (occ_info.ndim != 2) {
    throw std::invalid_argument("occupancy must be a 2D array");
  }
  const int height = static_cast<int>(occ_info.shape[0]);
  const int width = static_cast<int>(occ_info.shape[1]);
  bbs::Grid grid(height, width);
  const auto * occ_ptr = static_cast<const uint8_t *>(occ_info.ptr);
  const size_t cells = static_cast<size_t>(height) * static_cast<size_t>(width);
  for (size_t i = 0; i < cells; ++i) {
    grid.data[i] = occ_ptr[i] ? 1 : 0;
  }

  const auto scan_info = scan_xy_m.request();
  if (scan_info.ndim != 2 || scan_info.shape[1] != 2) {
    throw std::invalid_argument("scan_xy_m must have shape (N, 2)");
  }
  const size_t n_points = static_cast<size_t>(scan_info.shape[0]);
  std::vector<std::array<double, 2>> scan(n_points);
  const auto * scan_ptr = static_cast<const double *>(scan_info.ptr);
  for (size_t i = 0; i < n_points; ++i) {
    scan[i] = {scan_ptr[2 * i], scan_ptr[2 * i + 1]};
  }

  // Release the GIL for the search itself; no Python objects are touched here.
  std::vector<bbs::BbsGridCandidate> result;
  {
    py::gil_scoped_release release;
    result = bbs::branch_and_bound_candidates(
      grid, scan, resolution_m, angular_resolution_rad, pyramid_depth,
      max_candidates, nms_radius_cells);
  }
  return result;
}

}  // namespace

PYBIND11_MODULE(bbs_cpp, m)
{
  m.doc() =
    "C++ branch-and-bound (BBS_2D) global-localization search, a bit-exact "
    "port of branch_and_bound_candidates in make_bbs_relocalization_attempts.py.";

  py::class_<bbs::BbsGridCandidate>(m, "BbsGridCandidate")
    .def_readonly("tx_cell", &bbs::BbsGridCandidate::tx_cell)
    .def_readonly("ty_cell", &bbs::BbsGridCandidate::ty_cell)
    .def_readonly("yaw_index", &bbs::BbsGridCandidate::yaw_index)
    .def_readonly("yaw_rad", &bbs::BbsGridCandidate::yaw_rad)
    .def_readonly("score", &bbs::BbsGridCandidate::score)
    .def_readonly("hit_count", &bbs::BbsGridCandidate::hit_count)
    .def_readonly("point_count", &bbs::BbsGridCandidate::point_count)
    .def("__repr__", [](const bbs::BbsGridCandidate & c) {
      return "<bbs_cpp.BbsGridCandidate tx=" + std::to_string(c.tx_cell) +
             " ty=" + std::to_string(c.ty_cell) +
             " yaw_index=" + std::to_string(c.yaw_index) +
             " hit=" + std::to_string(c.hit_count) + ">";
    });

  m.def(
    "branch_and_bound_candidates", &branch_and_bound_candidates_py,
    py::arg("occupancy"), py::arg("scan_xy_m"), py::arg("resolution_m"),
    py::arg("angular_resolution_rad"), py::arg("pyramid_depth"),
    py::arg("max_candidates"), py::arg("nms_radius_cells") = 0,
    "Run the BBS_2D search and return ranked BbsGridCandidate objects "
    "identical to the Python reference.");
}
