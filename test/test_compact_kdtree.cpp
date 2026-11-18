// CompactKdTree: the same squared distances as PCL's FLANN tree for nearest-neighbour
// and radius searches, over clouds with clusters, duplicates, NaN points and an index
// subset, for queries near the points and far outside them.
#include "lidar_localization/compact_kdtree.hpp"

#include <pcl/search/kdtree.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <vector>

namespace
{

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

Point makePoint(float x, float y, float z)
{
  Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = 1.0f;
  return p;
}

// A ground plane on a grid, two dense clusters, exact duplicates and NaN points.
Cloud::Ptr makeCloud(std::mt19937 & rng)
{
  Cloud::Ptr cloud(new Cloud());
  std::normal_distribution<float> noise(0.0f, 0.05f);
  for (float x = -20.0f; x <= 20.0f; x += 0.5f) {
    for (float y = -20.0f; y <= 20.0f; y += 0.5f) {
      cloud->push_back(makePoint(x, y, noise(rng)));
    }
  }
  std::normal_distribution<float> cluster(0.0f, 0.3f);
  for (int i = 0; i < 3000; ++i) {
    cloud->push_back(makePoint(5.0f + cluster(rng), -3.0f + cluster(rng), 1.0f + cluster(rng)));
    cloud->push_back(makePoint(-8.0f + cluster(rng), 9.0f + cluster(rng), 2.0f));
  }
  for (int i = 0; i < 50; ++i) {
    cloud->push_back(makePoint(1.0f, 1.0f, 1.0f));
  }
  const float nan = std::numeric_limits<float>::quiet_NaN();
  cloud->push_back(makePoint(nan, 0.0f, 0.0f));
  cloud->push_back(makePoint(0.0f, 0.0f, std::numeric_limits<float>::infinity()));
  cloud->is_dense = false;
  return cloud;
}

std::vector<Point> makeQueries(const Cloud & cloud, std::mt19937 & rng)
{
  std::vector<Point> queries;
  std::uniform_int_distribution<std::size_t> pick(0, cloud.size() - 3);
  std::normal_distribution<float> near(0.0f, 0.2f);
  std::uniform_real_distribution<float> far(-60.0f, 60.0f);
  for (int i = 0; i < 2000; ++i) {
    Point p = cloud[pick(rng)];
    p.x += near(rng);
    p.y += near(rng);
    p.z += near(rng);
    queries.push_back(p);
    queries.push_back(makePoint(far(rng), far(rng), far(rng)));
  }
  queries.push_back(cloud[0]);  // on a point
  queries.push_back(makePoint(1.0f, 1.0f, 1.0f));  // on the duplicates
  return queries;
}

void checkSame(
  const pcl::search::KdTree<Point> & flann, const lidar_localization::CompactKdTree<Point> & compact,
  const Cloud & cloud, const std::vector<Point> & queries)
{
  pcl::Indices flann_indices;
  pcl::Indices compact_indices;
  std::vector<float> flann_distances;
  std::vector<float> compact_distances;
  for (const Point & q : queries) {
    for (const int k : {1, 7}) {
      const int a = flann.nearestKSearch(q, k, flann_indices, flann_distances);
      const int b = compact.nearestKSearch(q, k, compact_indices, compact_distances);
      assert(a == b);
      for (int i = 0; i < a; ++i) {
        // Bit-identical squared distances.
        assert(flann_distances[i] == compact_distances[i]);
        // The index names a point at that distance.
        const Point & p = cloud[compact_indices[i]];
        const float dx = q.x - p.x;
        const float dy = q.y - p.y;
        const float dz = q.z - p.z;
        float d = dx * dx;
        d += dy * dy;
        d += dz * dz;
        assert(d == compact_distances[i]);
      }
    }
    for (const double radius : {0.3, 1.5}) {
      const int a = flann.radiusSearch(q, radius, flann_indices, flann_distances);
      const int b = compact.radiusSearch(q, radius, compact_indices, compact_distances);
      assert(a == b);
      std::sort(flann_distances.begin(), flann_distances.end());
      assert(flann_distances == compact_distances);
      assert(std::is_sorted(compact_distances.begin(), compact_distances.end()));
      if (b > 3) {
        assert(compact.radiusSearch(q, radius, compact_indices, compact_distances, 3) == 3);
        assert(std::equal(compact_distances.begin(), compact_distances.end(),
          flann_distances.begin()));
      }
    }
  }
}

void testMatchesFlann()
{
  std::mt19937 rng(7);
  const Cloud::Ptr cloud = makeCloud(rng);
  const std::vector<Point> queries = makeQueries(*cloud, rng);

  pcl::search::KdTree<Point> flann;
  flann.setInputCloud(cloud);
  lidar_localization::CompactKdTree<Point> compact;
  compact.setInputCloud(cloud);
  assert(compact.getInputCloud() == cloud);
  assert(compact.size() == cloud->size() - 2);  // without the NaN and inf points
  // About 6 bytes a point against FLANN's copies of the coordinates.
  assert(compact.memoryBytes() < 8 * cloud->size());
  checkSame(flann, compact, *cloud, queries);

  // An index subset: every third point.
  pcl::IndicesPtr subset(new pcl::Indices());
  for (std::size_t i = 0; i < cloud->size(); i += 3) {
    subset->push_back(static_cast<pcl::index_t>(i));
  }
  pcl::search::KdTree<Point> flann_subset;
  flann_subset.setInputCloud(cloud, subset);
  lidar_localization::CompactKdTree<Point> compact_subset;
  compact_subset.setInputCloud(cloud, subset);
  checkSame(flann_subset, compact_subset, *cloud, queries);

  // A second cloud replaces the first.
  Cloud::Ptr moved(new Cloud(*cloud));
  for (auto & p : moved->points) {
    p.x += 3.0f;
  }
  flann.setInputCloud(moved);
  compact.setInputCloud(moved);
  checkSame(flann, compact, *moved, queries);
}

void testEdgeCases()
{
  lidar_localization::CompactKdTree<Point> tree;
  pcl::Indices indices;
  std::vector<float> distances;
  // No cloud, an empty cloud: nothing found.
  assert(tree.nearestKSearch(makePoint(0, 0, 0), 1, indices, distances) == 0);
  tree.setInputCloud(Cloud::Ptr(new Cloud()));
  assert(tree.nearestKSearch(makePoint(0, 0, 0), 1, indices, distances) == 0);
  assert(tree.radiusSearch(makePoint(0, 0, 0), 1.0, indices, distances) == 0);

  // Fewer points than k; a non-finite query.
  Cloud::Ptr two(new Cloud());
  two->push_back(makePoint(0, 0, 0));
  two->push_back(makePoint(3, 0, 0));
  tree.setInputCloud(two);
  assert(tree.nearestKSearch(makePoint(1, 0, 0), 5, indices, distances) == 2);
  assert(indices[0] == 0 && distances[0] == 1.0f && indices[1] == 1 && distances[1] == 4.0f);
  assert(
    tree.nearestKSearch(
      makePoint(std::numeric_limits<float>::quiet_NaN(), 0, 0), 1, indices, distances) == 0);
  // A point at exactly the radius does not count, as in FLANN.
  assert(tree.radiusSearch(makePoint(1, 0, 0), 2.0, indices, distances) == 1);
  assert(tree.radiusSearch(makePoint(1, 0, 0), 2.001, indices, distances) == 2);

  // As pcl::Registration holds it.
  pcl::search::KdTree<Point>::Ptr base(new lidar_localization::CompactKdTree<Point>());
  base->setInputCloud(two);
  assert(base->nearestKSearch(makePoint(2.5f, 0, 0), 1, indices, distances) == 1);
  assert(indices[0] == 1 && distances[0] == 0.25f);
}

}  // namespace

int main()
{
  testMatchesFlann();
  testEdgeCases();
  return 0;
}
