#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "lidar_localization/local_reacquisition_policy.hpp"
#include "lidar_localization/occupancy_map_generation.hpp"

namespace ll = lidar_localization;

using Points = std::vector<std::array<float, 3>>;

std::uint8_t pixelAt(const ll::GeneratedOccupancyMap & map, int x, int y)
{
  return map.pixels[static_cast<std::size_t>(map.height - 1 - y) * map.width + x];
}

void add(Points & points, float x, float y, float z, int copies = 2)
{
  for (int i = 0; i < copies; ++i) {
    points.push_back({x, y, z});
  }
}

void testRouteBounds()
{
  const auto bounds = ll::occupancyBoundsFromRoute({{0, 0, 1}, {4, -2, 3}, {1, 5, 2}, {2, 1, 9}}, 1.0);
  assert(bounds.min_x == -1.0 && bounds.max_x == 5.0);
  assert(bounds.min_y == -3.0 && bounds.max_y == 6.0);
  assert(bounds.route_ground_z && *bounds.route_ground_z == 2.5);  // median of 1, 2, 3, 9
}

void testRouteGroundClassification()
{
  // 4 x 2 cells of 0.5 m; route ground at z = 0.
  Points points;
  add(points, 0.2, 0.2, 0.0);   // cell (0, 0): ground + wall -> occupied
  add(points, 0.2, 0.2, 1.0);
  add(points, 1.7, 0.7, 0.1);   // cell (3, 1): ground only -> free
  add(points, 1.2, 0.2, 0.0, 1);  // cell (2, 0): one point -> unknown
  add(points, 0.7, 0.7, 5.0);   // cell (1, 1): canopy only, no ground -> unknown
  add(points, 9.0, 9.0, 0.0);   // outside the bounds -> ignored
  ll::OccupancyMapOptions options;
  options.resolution_m = 0.5;
  options.inflate_radius_m = 0.0;
  const ll::OccupancyMapBounds bounds{0.0, 2.0, 0.0, 1.0, 0.0};
  const auto map = ll::generateOccupancyMap(points, bounds, options);
  assert(map.width == 4 && map.height == 2);
  assert(map.point_count == 9);
  assert(pixelAt(map, 0, 0) == ll::kOccupancyOccupied);
  assert(pixelAt(map, 3, 1) == ll::kOccupancyFree);
  assert(pixelAt(map, 2, 0) == ll::kOccupancyUnknown);
  assert(pixelAt(map, 1, 1) == ll::kOccupancyUnknown);
}

void testHeightExtentClassificationAndInflation()
{
  // 5 x 5 cells of 1 m, no route: a cell is occupied when its z extent reaches 0.4 m.
  Points points;
  for (int x = 0; x < 5; ++x) {
    for (int y = 0; y < 5; ++y) {
      add(points, x + 0.5F, y + 0.5F, 0.0);
    }
  }
  add(points, 2.5, 2.5, 0.5);  // centre cell spans 0.5 m
  ll::OccupancyMapOptions options;
  options.resolution_m = 1.0;
  options.inflate_radius_m = 1.0;
  const ll::OccupancyMapBounds bounds{0.0, 5.0, 0.0, 5.0, std::nullopt};
  const auto map = ll::generateOccupancyMap(points, bounds, options);
  // The centre and its four neighbours are occupied; diagonals stay free.
  assert(pixelAt(map, 2, 2) == ll::kOccupancyOccupied);
  assert(pixelAt(map, 1, 2) == ll::kOccupancyOccupied);
  assert(pixelAt(map, 2, 3) == ll::kOccupancyOccupied);
  assert(pixelAt(map, 1, 1) == ll::kOccupancyFree);
  assert(pixelAt(map, 0, 0) == ll::kOccupancyFree);
}

void testWrittenMapLoadsForReacquisition()
{
  Points points;
  add(points, 0.5, 0.5, 0.0);
  add(points, 0.5, 0.5, 1.0);  // occupied at (0, 0)
  add(points, 2.5, 1.5, 0.0);  // free at (2, 1)
  ll::OccupancyMapOptions options;
  options.resolution_m = 1.0;
  options.inflate_radius_m = 0.0;
  const ll::OccupancyMapBounds bounds{-1.0, 3.0, 0.0, 2.0, std::nullopt};
  const auto generated = ll::generateOccupancyMap(points, bounds, options);
  ll::writeOccupancyMap(generated, "/tmp", "ll_generated_occupancy");

  const auto loaded = ll::loadOccupancyGridMap("/tmp/ll_generated_occupancy.yaml");
  assert(loaded.grid.width == 4 && loaded.grid.height == 2);
  assert(loaded.resolution_m == 1.0 && loaded.origin_x_m == -1.0 && loaded.origin_y_m == 0.0);
  // Loader grid row 0 is the lowest y, matching the generator's cell rows.
  assert(loaded.grid.at(0, 1) == 1);
  assert(loaded.grid.at(1, 3) == 0);
  std::remove("/tmp/ll_generated_occupancy.yaml");
  std::remove("/tmp/ll_generated_occupancy.pgm");
}

int main()
{
  testRouteBounds();
  testRouteGroundClassification();
  testHeightExtentClassificationAndInflation();
  testWrittenMapLoadsForReacquisition();
  return 0;
}
