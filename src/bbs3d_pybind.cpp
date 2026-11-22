// Python binding of the optional 3D global search (G2 engine "bbs3d").
//
// The search (include/lidar_localization/bbs3d_topk_search.hpp) runs on the voxel
// maps of 3D-BBS (third_party/bbs3d, MIT License, Copyright (c) 2023 KokiAoki).
// global_localization_query.py imports this module only when
// global_search_engine=bbs3d, builds one MapSearcher per map load and asks it for the
// k best poses of each levelled scan; the G2 NDT scorer then refines and ranks them.

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_localization/bbs3d_topk_search.hpp"

namespace py = pybind11;
namespace bbs3d = lidar_localization::bbs3d;

namespace
{

using DoubleArray = py::array_t<double, py::array::c_style | py::array::forcecast>;

bbs3d::Points to_points(const DoubleArray & array)
{
  if (array.ndim() != 2 || array.shape(1) != 3) {
    throw std::invalid_argument("points must be an (N, 3) array");
  }
  auto view = array.unchecked<2>();
  bbs3d::Points points;
  points.reserve(static_cast<std::size_t>(view.shape(0)));
  for (py::ssize_t i = 0; i < view.shape(0); ++i) {
    const Eigen::Vector3d point(view(i, 0), view(i, 1), view(i, 2));
    if (point.allFinite()) {
      points.push_back(point);
    }
  }
  return points;
}

bbs3d::Points load_map_points(const std::string & path)
{
  std::string lower = path;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  auto ends_with = [&lower](const std::string & suffix) {
    return lower.size() >= suffix.size() &&
           lower.compare(lower.size() - suffix.size(), suffix.size(), suffix) == 0;
  };
  bbs3d::Points points;
  {
    pcl::PCLPointCloud2 raw;
    int status = -1;
    if (ends_with(".pcd")) {
      status = pcl::io::loadPCDFile(path, raw);
    } else if (ends_with(".ply")) {
      status = pcl::io::loadPLYFile(path, raw);
    } else {
      throw std::runtime_error("bbs3d: unsupported map file (need .pcd or .ply): " + path);
    }
    if (status != 0) {
      throw std::runtime_error("bbs3d: failed to load map: " + path);
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(raw, cloud);
    points.reserve(cloud.size());
    for (const auto & p : cloud.points) {
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
        points.emplace_back(p.x, p.y, p.z);
      }
    }
  }
  return points;
}

class MapSearcher
{
public:
  // map_leaf_m thins the map (voxel centroids) before the voxel hierarchy is built;
  // the thinned points are not kept.
  MapSearcher(
    const std::string & map_path, double map_leaf_m, double min_level_res_m, int max_level)
  {
    py::gil_scoped_release release;
    auto points = load_map_points(map_path);
    loaded_points_ = points.size();
    build(std::move(points), map_leaf_m, min_level_res_m, max_level);
  }

  MapSearcher(
    const DoubleArray & map_points, double map_leaf_m, double min_level_res_m, int max_level)
  {
    auto points = to_points(map_points);
    loaded_points_ = points.size();
    py::gil_scoped_release release;
    build(std::move(points), map_leaf_m, min_level_res_m, max_level);
  }

  py::dict info() const
  {
    py::dict out;
    out["loaded_points"] = loaded_points_;
    out["map_points"] = map_points_;
    out["build_ms"] = search_->build_ms();
    out["buckets"] = search_->bucket_count();
    out["voxelmap_bytes"] = search_->memory_bytes();
    out["min_xyz"] = Eigen::Vector3d(search_->min_xyz());
    out["max_xyz"] = Eigen::Vector3d(search_->max_xyz());
    return out;
  }

  // Scan points in the levelled base frame; they are range-filtered and thinned
  // here. Returns the k best poses (map <- levelled base) with their scores.
  py::dict search(
    const DoubleArray & scan, double min_range_m, double max_range_m, double scan_leaf_m,
    int k, double min_separation_m, double min_score_ratio, double relative_score_floor,
    int timeout_ms, std::size_t max_queue_nodes, int num_threads, double max_tilt_rad) const
  {
    auto raw = to_points(scan);
    bbs3d::SearchParams params;
    params.k = k;
    params.min_separation_m = min_separation_m;
    params.min_score_ratio = min_score_ratio;
    params.relative_score_floor = relative_score_floor;
    params.timeout_ms = timeout_ms;
    params.max_queue_nodes = max_queue_nodes;
    params.num_threads = num_threads;
    params.max_tilt_rad = max_tilt_rad;
    bbs3d::SearchResult result;
    {
      py::gil_scoped_release release;
      const auto prepared = bbs3d::voxel_centroid_filter(
        bbs3d::range_filter(raw, min_range_m, max_range_m), scan_leaf_m);
      result = search_->search(prepared, params);
    }
    py::list candidates;
    for (const auto & candidate : result.candidates) {
      py::dict item;
      item["pose"] = candidate.pose;
      item["score"] = candidate.score;
      item["score_ratio"] = candidate.score_ratio;
      candidates.append(item);
    }
    py::dict out;
    out["candidates"] = candidates;
    out["timed_out"] = result.timed_out;
    out["queue_capped"] = result.queue_capped;
    out["expanded"] = result.expanded;
    out["max_queue"] = result.max_queue;
    out["initial"] = result.initial;
    out["scan_points"] = result.scan_points;
    out["elapsed_ms"] = result.elapsed_ms;
    return out;
  }

private:
  void build(bbs3d::Points points, double map_leaf_m, double min_level_res_m, int max_level)
  {
    if (map_leaf_m > 0.0) {
      points = bbs3d::voxel_centroid_filter(points, map_leaf_m);
    }
    map_points_ = points.size();
    search_ = std::make_unique<bbs3d::TopkSearch>(points, min_level_res_m, max_level);
  }

  std::unique_ptr<bbs3d::TopkSearch> search_;
  std::size_t loaded_points_{0};
  std::size_t map_points_{0};
};

}  // namespace

PYBIND11_MODULE(bbs3d_cpp, m)
{
  m.doc() = "3D branch-and-bound global search (3D-BBS voxel maps) for the G2 engine bbs3d";
  py::class_<MapSearcher>(m, "MapSearcher")
  .def(
    py::init<const std::string &, double, double, int>(), py::arg("map_path"),
    py::arg("map_leaf_m") = 0.1, py::arg("min_level_res_m") = 0.5, py::arg("max_level") = 6)
  .def(
    py::init<const DoubleArray &, double, double, int>(), py::arg("map_points"),
    py::arg("map_leaf_m") = 0.1, py::arg("min_level_res_m") = 0.5, py::arg("max_level") = 6)
  .def("info", &MapSearcher::info)
  .def(
    "search", &MapSearcher::search, py::arg("scan"), py::arg("min_range_m") = 1.0,
    py::arg("max_range_m") = 30.0, py::arg("scan_leaf_m") = 0.5, py::arg("k") = 8,
    py::arg("min_separation_m") = 3.0, py::arg("min_score_ratio") = 0.0,
    py::arg("relative_score_floor") = 0.9, py::arg("timeout_ms") = 10000,
    py::arg("max_queue_nodes") = 4000000, py::arg("num_threads") = 4,
    py::arg("max_tilt_rad") = 0.02);
}
