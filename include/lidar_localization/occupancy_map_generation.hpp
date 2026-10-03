#ifndef LIDAR_LOCALIZATION_OCCUPANCY_MAP_GENERATION_HPP_
#define LIDAR_LOCALIZATION_OCCUPANCY_MAP_GENERATION_HPP_

// 2D occupancy map (ROS map_server PGM + YAML) from a 3D point cloud map, for
// local re-acquisition, global localization and Nav2.
//
// With a route ground height (median z of a reference trajectory), a cell is
// classified only where ground near that height was observed: occupied when it
// also has points at least obstacle_height_m above the ground, free when it
// has none. Without one, a cell is occupied when its z extent reaches
// obstacle_height_m. Occupied cells are inflated; all others stay unknown.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace lidar_localization
{

struct OccupancyMapOptions
{
  double resolution_m{0.2};
  double obstacle_height_m{0.4};
  int min_points_per_cell{2};
  double inflate_radius_m{0.6};
  double ground_band_m{1.0};
};

struct OccupancyMapBounds
{
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
  std::optional<double> route_ground_z;
};

// PGM pixel values, as written by ROS map_server for a trinary map.
constexpr std::uint8_t kOccupancyOccupied = 0;
constexpr std::uint8_t kOccupancyUnknown = 205;
constexpr std::uint8_t kOccupancyFree = 254;

struct GeneratedOccupancyMap
{
  int width{0};
  int height{0};
  double resolution_m{0.0};
  double origin_x_m{0.0};
  double origin_y_m{0.0};
  // Image order: row 0 is the highest y.
  std::vector<std::uint8_t> pixels;
  std::size_t point_count{0};
};

// Crop to the padded route extent; the ground height is the median route z.
inline OccupancyMapBounds occupancyBoundsFromRoute(
  const std::vector<std::array<double, 3>> & route, double padding_m)
{
  if (route.empty()) {
    throw std::runtime_error("reference route has no positions");
  }
  OccupancyMapBounds bounds{route[0][0], route[0][0], route[0][1], route[0][1], std::nullopt};
  std::vector<double> zs;
  zs.reserve(route.size());
  for (const auto & position : route) {
    bounds.min_x = std::min(bounds.min_x, position[0]);
    bounds.max_x = std::max(bounds.max_x, position[0]);
    bounds.min_y = std::min(bounds.min_y, position[1]);
    bounds.max_y = std::max(bounds.max_y, position[1]);
    zs.push_back(position[2]);
  }
  bounds.min_x -= padding_m;
  bounds.max_x += padding_m;
  bounds.min_y -= padding_m;
  bounds.max_y += padding_m;
  const auto middle = zs.begin() + static_cast<std::ptrdiff_t>(zs.size() / 2);
  std::nth_element(zs.begin(), middle, zs.end());
  double median = *middle;
  if (zs.size() % 2 == 0) {
    median = 0.5 * (median + *std::max_element(zs.begin(), middle));
  }
  bounds.route_ground_z = median;
  return bounds;
}

inline OccupancyMapBounds occupancyBoundsFromPoints(
  const std::vector<std::array<float, 3>> & points, double padding_m)
{
  if (points.empty()) {
    throw std::runtime_error("point cloud is empty");
  }
  OccupancyMapBounds bounds{points[0][0], points[0][0], points[0][1], points[0][1], std::nullopt};
  for (const auto & point : points) {
    bounds.min_x = std::min<double>(bounds.min_x, point[0]);
    bounds.max_x = std::max<double>(bounds.max_x, point[0]);
    bounds.min_y = std::min<double>(bounds.min_y, point[1]);
    bounds.max_y = std::max<double>(bounds.max_y, point[1]);
  }
  bounds.min_x -= padding_m;
  bounds.max_x += padding_m;
  bounds.min_y -= padding_m;
  bounds.max_y += padding_m;
  return bounds;
}

inline GeneratedOccupancyMap generateOccupancyMap(
  const std::vector<std::array<float, 3>> & points, const OccupancyMapBounds & bounds,
  const OccupancyMapOptions & options)
{
  if (!(options.resolution_m > 0.0)) {
    throw std::runtime_error("occupancy resolution must be positive");
  }
  GeneratedOccupancyMap map;
  map.resolution_m = options.resolution_m;
  map.origin_x_m = bounds.min_x;
  map.origin_y_m = bounds.min_y;
  map.width = static_cast<int>(std::ceil((bounds.max_x - bounds.min_x) / options.resolution_m));
  map.height = static_cast<int>(std::ceil((bounds.max_y - bounds.min_y) / options.resolution_m));
  if (map.width <= 0 || map.height <= 0) {
    throw std::runtime_error("occupancy bounds are empty");
  }

  const std::size_t cells = static_cast<std::size_t>(map.width) * map.height;
  std::vector<int> count(cells, 0);
  std::vector<int> low_count(cells, 0);
  std::vector<int> high_count(cells, 0);
  std::vector<float> min_z(cells, INFINITY);
  std::vector<float> max_z(cells, -INFINITY);
  for (const auto & point : points) {
    if (point[0] < bounds.min_x || point[0] > bounds.max_x ||
      point[1] < bounds.min_y || point[1] > bounds.max_y)
    {
      continue;
    }
    const int ix = std::clamp(
      static_cast<int>((point[0] - bounds.min_x) / options.resolution_m), 0, map.width - 1);
    const int iy = std::clamp(
      static_cast<int>((point[1] - bounds.min_y) / options.resolution_m), 0, map.height - 1);
    const std::size_t cell = static_cast<std::size_t>(iy) * map.width + ix;
    ++count[cell];
    min_z[cell] = std::min(min_z[cell], point[2]);
    max_z[cell] = std::max(max_z[cell], point[2]);
    if (bounds.route_ground_z) {
      if (point[2] <= *bounds.route_ground_z + options.ground_band_m) {
        ++low_count[cell];
      }
      if (point[2] >= *bounds.route_ground_z + options.obstacle_height_m) {
        ++high_count[cell];
      }
    }
    ++map.point_count;
  }
  if (map.point_count == 0) {
    throw std::runtime_error("no points inside the occupancy bounds");
  }

  std::vector<bool> occupied(cells, false);
  std::vector<bool> free(cells, false);
  for (std::size_t cell = 0; cell < cells; ++cell) {
    if (bounds.route_ground_z) {
      if (low_count[cell] < options.min_points_per_cell) {
        continue;
      }
      occupied[cell] = high_count[cell] >= options.min_points_per_cell;
      free[cell] = high_count[cell] == 0;
    } else if (count[cell] >= options.min_points_per_cell) {
      occupied[cell] = max_z[cell] - min_z[cell] >= options.obstacle_height_m;
      free[cell] = !occupied[cell];
    }
  }

  const int radius = static_cast<int>(std::ceil(options.inflate_radius_m / options.resolution_m));
  std::vector<bool> inflated = occupied;
  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      if (!occupied[static_cast<std::size_t>(y) * map.width + x]) {
        continue;
      }
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          const int yy = y + dy;
          const int xx = x + dx;
          if (dx * dx + dy * dy <= radius * radius && yy >= 0 && yy < map.height && xx >= 0 &&
            xx < map.width)
          {
            inflated[static_cast<std::size_t>(yy) * map.width + xx] = true;
          }
        }
      }
    }
  }

  map.pixels.assign(cells, kOccupancyUnknown);
  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      const std::size_t cell = static_cast<std::size_t>(y) * map.width + x;
      const std::size_t pixel = static_cast<std::size_t>(map.height - 1 - y) * map.width + x;
      if (inflated[cell]) {
        map.pixels[pixel] = kOccupancyOccupied;
      } else if (free[cell]) {
        map.pixels[pixel] = kOccupancyFree;
      }
    }
  }
  return map;
}

// Writes <directory>/<name>.pgm (binary P5) and <name>.yaml next to it.
inline void writeOccupancyMap(
  const GeneratedOccupancyMap & map, const std::string & directory, const std::string & name)
{
  const std::string pgm_path = directory + "/" + name + ".pgm";
  std::ofstream pgm(pgm_path, std::ios::binary);
  pgm << "P5\n" << map.width << " " << map.height << "\n255\n";
  pgm.write(
    reinterpret_cast<const char *>(map.pixels.data()),
    static_cast<std::streamsize>(map.pixels.size()));
  if (!pgm) {
    throw std::runtime_error("cannot write " + pgm_path);
  }
  const std::string yaml_path = directory + "/" + name + ".yaml";
  std::ofstream yaml(yaml_path);
  yaml.precision(9);
  yaml << "image: " << name << ".pgm\n"
       << "resolution: " << map.resolution_m << "\n"
       << "origin: [" << map.origin_x_m << ", " << map.origin_y_m << ", 0.0]\n"
       << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n";
  if (!yaml) {
    throw std::runtime_error("cannot write " + yaml_path);
  }
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_OCCUPANCY_MAP_GENERATION_HPP_
