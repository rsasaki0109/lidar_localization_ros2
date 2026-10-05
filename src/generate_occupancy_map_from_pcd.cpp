// Generate a 2D occupancy map (PGM + YAML) from a 3D PCD/PLY point cloud map.
// See include/lidar_localization/occupancy_map_generation.hpp for the rules.

#include <pcl/io/auto_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "lidar_localization/occupancy_map_generation.hpp"

namespace ll = lidar_localization;

namespace
{

constexpr const char * kUsage =
  "usage: generate_occupancy_map_from_pcd --pcd MAP --output-dir DIR [options]\n"
  "  --map-name NAME             output base name (default occupancy_map)\n"
  "  --resolution M              cell size (default 0.2)\n"
  "  --obstacle-height-m M       occupied above this height (default 0.4)\n"
  "  --max-obstacle-height-m M   ignore points higher than this above the ground, e.g.\n"
  "                              a ceiling indoors (default 0: no limit)\n"
  "  --min-points-per-cell N     points needed to observe a cell (default 2)\n"
  "  --inflate-radius-m M        inflate occupied cells (default 0.6)\n"
  "  --padding-m M               padding around the map extent (default 5.0)\n"
  "  --reference-csv CSV         crop around a route (position_x/y/z columns)\n"
  "                              and classify relative to its median height\n"
  "  --route-padding-m M         padding around the route (default 20.0)\n"
  "  --ground-band-m M           ground band above the route height (default 1.0)\n"
  "  --x-min/--x-max/--y-min/--y-max M   explicit crop instead of the map extent\n";

std::vector<std::array<double, 3>> readRoute(const std::string & path)
{
  std::ifstream csv(path);
  if (!csv) {
    throw std::runtime_error("cannot open reference csv: " + path);
  }
  std::string line;
  std::getline(csv, line);
  std::map<std::string, std::size_t> columns;
  {
    std::stringstream header(line);
    std::string name;
    for (std::size_t index = 0; std::getline(header, name, ','); ++index) {
      columns[name] = index;
    }
  }
  const std::array<const char *, 3> keys{"position_x", "position_y", "position_z"};
  for (const char * key : keys) {
    if (columns.count(key) == 0) {
      throw std::runtime_error(std::string("reference csv has no ") + key + " column: " + path);
    }
  }
  std::vector<std::array<double, 3>> route;
  while (std::getline(csv, line)) {
    std::vector<std::string> fields;
    std::stringstream row(line);
    std::string field;
    while (std::getline(row, field, ',')) {
      fields.push_back(field);
    }
    std::array<double, 3> position{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const std::size_t column = columns[keys[axis]];
      if (column >= fields.size()) {
        throw std::runtime_error("short row in reference csv: " + path);
      }
      position[axis] = std::stod(fields[column]);
    }
    route.push_back(position);
  }
  return route;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::map<std::string, std::string> args;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "-h" || key == "--help") {
      std::cout << kUsage;
      return 0;
    }
    if (key.rfind("--", 0) != 0 || i + 1 >= argc) {
      std::cerr << "unexpected argument: " << key << "\n" << kUsage;
      return 2;
    }
    args[key.substr(2)] = argv[++i];
  }
  const auto get = [&args](const std::string & key, const std::string & fallback) {
      const auto found = args.find(key);
      return found == args.end() ? fallback : found->second;
    };
  if (args.count("pcd") == 0 || args.count("output-dir") == 0) {
    std::cerr << kUsage;
    return 2;
  }

  try {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::load(args["pcd"], cloud) != 0) {
      throw std::runtime_error("cannot read point cloud: " + args["pcd"]);
    }
    std::vector<std::array<float, 3>> points;
    points.reserve(cloud.size());
    for (const auto & point : cloud) {
      points.push_back({point.x, point.y, point.z});
    }

    ll::OccupancyMapOptions options;
    options.resolution_m = std::stod(get("resolution", "0.2"));
    options.obstacle_height_m = std::stod(get("obstacle-height-m", "0.4"));
    options.min_points_per_cell = std::stoi(get("min-points-per-cell", "2"));
    options.inflate_radius_m = std::stod(get("inflate-radius-m", "0.6"));
    options.ground_band_m = std::stod(get("ground-band-m", "1.0"));
    options.max_obstacle_height_m = std::stod(get("max-obstacle-height-m", "0.0"));

    ll::OccupancyMapBounds bounds;
    if (args.count("reference-csv") != 0) {
      bounds = ll::occupancyBoundsFromRoute(
        readRoute(args["reference-csv"]), std::stod(get("route-padding-m", "20.0")));
    } else if (args.count("x-min") && args.count("x-max") && args.count("y-min") &&
      args.count("y-max"))
    {
      bounds = {std::stod(args["x-min"]), std::stod(args["x-max"]), std::stod(args["y-min"]),
        std::stod(args["y-max"]), std::nullopt};
    } else {
      bounds = ll::occupancyBoundsFromPoints(points, std::stod(get("padding-m", "5.0")));
    }

    const auto map = ll::generateOccupancyMap(points, bounds, options);
    const std::string directory = args["output-dir"];
    const std::string name = get("map-name", "occupancy_map");
    std::filesystem::create_directories(directory);
    ll::writeOccupancyMap(map, directory, name);

    std::size_t occupied = 0;
    std::size_t free = 0;
    for (const auto pixel : map.pixels) {
      occupied += pixel == ll::kOccupancyOccupied;
      free += pixel == ll::kOccupancyFree;
    }
    std::cout << "yaml: " << directory << "/" << name << ".yaml\n"
              << "size: " << map.width << " x " << map.height << " cells at "
              << map.resolution_m << " m, " << map.point_count << " points\n"
              << "cells: " << occupied << " occupied, " << free << " free, "
              << map.pixels.size() - occupied - free << " unknown\n";
    if (bounds.route_ground_z) {
      std::cout << "route ground z: " << *bounds.route_ground_z << "\n";
    }
  } catch (const std::exception & error) {
    std::cerr << "error: " << error.what() << "\n";
    return 1;
  }
  return 0;
}
