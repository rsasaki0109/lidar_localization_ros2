#ifndef LIDAR_LOCALIZATION_COMPACT_KDTREE_HPP_
#define LIDAR_LOCALIZATION_COMPACT_KDTREE_HPP_

#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include <pcl/pcl_config.h>

#if !PCL_VERSION_COMPARE(>=, 1, 11, 0)
// PCL 1.10 (ROS 2 Foxy) predates pcl::index_t and pcl::Indices; its search interface
// takes std::vector<int>, which these aliases name.
namespace pcl
{
using index_t = int;
using Indices = std::vector<int>;
}  // namespace pcl
#endif

namespace lidar_localization
{

// An exact nearest-neighbour search over a point cloud that does not copy it: the tree
// holds a 4-byte index per point and about 1.5 bytes of nodes per point, and reads the
// coordinates from the cloud it indexes. pcl::search::KdTree (FLANN) copies the
// coordinates twice (its flat array and FLANN's reordered dataset) and adds an index
// mapping: about 40 bytes a point, 70-80 MB for a 2.0M-point map.
//
// It returns the same squared distances as FLANN's exact search (epsilon 0), computed
// the same way in float, so the fitness score does not change. Among points at the
// same distance the index may differ.
//
// It derives from pcl::search::KdTree only so that pcl::Registration takes it as its
// target search method; the FLANN tree of the base class stays empty. The searches are
// const and safe to call from several threads. The cloud must outlive the tree and not
// change while it is indexed; pcl::Registration holds both.
template<typename PointT>
class CompactKdTree : public pcl::search::KdTree<PointT>
{
public:
  using Base = pcl::search::KdTree<PointT>;
  using PointCloudConstPtr = typename Base::PointCloudConstPtr;
  using Ptr = std::shared_ptr<CompactKdTree<PointT>>;
  using Base::nearestKSearch;
  using Base::radiusSearch;

  static constexpr std::uint32_t kLeafSize = 16;

  CompactKdTree()
  : Base(true) {}

  // pcl::search::Search::setInputCloud returns bool since PCL 1.11 and void before.
  using SetInputCloudResult = decltype(std::declval<pcl::search::Search<PointT> &>()
    .setInputCloud(PointCloudConstPtr(), pcl::IndicesConstPtr()));

  SetInputCloudResult setInputCloud(
    const PointCloudConstPtr & cloud,
    const pcl::IndicesConstPtr & indices = pcl::IndicesConstPtr()) override
  {
    this->input_ = cloud;
    this->indices_ = indices;
    build();
    return static_cast<SetInputCloudResult>(true);
  }

  int nearestKSearch(
    const PointT & point, int k, pcl::Indices & k_indices,
    std::vector<float> & k_sqr_distances) const override
  {
    k_indices.clear();
    k_sqr_distances.clear();
    if (k <= 0 || nodes_.empty() || !finite(point)) {
      return 0;
    }
    KnnResult result(static_cast<std::size_t>(k));
    const float query[3] = {point.x, point.y, point.z};
    float plane_distances[3] = {0.0f, 0.0f, 0.0f};
    search(0, query, boxDistance(query, plane_distances), plane_distances, result);
    k_indices.assign(result.indices.begin(), result.indices.end());
    k_sqr_distances.assign(result.distances.begin(), result.distances.end());
    return static_cast<int>(k_indices.size());
  }

  int radiusSearch(
    const PointT & point, double radius, pcl::Indices & k_indices,
    std::vector<float> & k_sqr_distances, unsigned int max_nn = 0) const override
  {
    k_indices.clear();
    k_sqr_distances.clear();
    if (nodes_.empty() || !finite(point) || !(radius >= 0.0)) {
      return 0;
    }
    RadiusResult result(static_cast<float>(radius * radius));
    const float query[3] = {point.x, point.y, point.z};
    float plane_distances[3] = {0.0f, 0.0f, 0.0f};
    search(0, query, boxDistance(query, plane_distances), plane_distances, result);
    std::vector<std::pair<float, pcl::index_t>> found(result.found);
    std::sort(found.begin(), found.end());
    if (max_nn > 0 && found.size() > max_nn) {
      found.resize(max_nn);
    }
    for (const auto & [distance, index] : found) {
      k_indices.push_back(index);
      k_sqr_distances.push_back(distance);
    }
    return static_cast<int>(k_indices.size());
  }

  // Indexed points.
  std::size_t size() const {return order_.size();}

  // Heap bytes the tree holds itself, without the cloud.
  std::size_t memoryBytes() const
  {
    return order_.capacity() * sizeof(std::uint32_t) + nodes_.capacity() * sizeof(Node);
  }

private:
  // An inner node splits along dim: its first child holds the points up to low, the
  // second those from high on. A leaf (dim 3) holds order_[begin, end).
  struct Node
  {
    float low;
    float high;
    std::uint32_t begin;
    std::uint32_t end;  // inner node: index of the second child; the first is next
    std::uint8_t dim;
  };

  struct KnnResult
  {
    explicit KnnResult(std::size_t k)
    : capacity(k) {indices.reserve(k); distances.reserve(k);}
    float worst() const
    {
      return distances.size() < capacity ? std::numeric_limits<float>::infinity() :
             distances.back();
    }
    void add(float distance, pcl::index_t index)
    {
      if (distance >= worst()) {
        return;
      }
      auto at = std::upper_bound(distances.begin(), distances.end(), distance);
      const auto offset = at - distances.begin();
      if (distances.size() == capacity) {
        distances.pop_back();
        indices.pop_back();
      }
      distances.insert(distances.begin() + offset, distance);
      indices.insert(indices.begin() + offset, index);
    }
    std::size_t capacity;
    std::vector<pcl::index_t> indices;
    std::vector<float> distances;
  };

  struct RadiusResult
  {
    explicit RadiusResult(float limit)
    : limit_sq(limit) {}
    // Points at exactly the radius do not count, as in FLANN.
    float worst() const {return limit_sq;}
    void add(float distance, pcl::index_t index)
    {
      if (distance < limit_sq) {
        found.emplace_back(distance, index);
      }
    }
    float limit_sq;
    std::vector<std::pair<float, pcl::index_t>> found;
  };

  static bool finite(const PointT & p)
  {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  }

  float coordinate(std::uint32_t index, int dim) const
  {
    const PointT & p = (*this->input_)[index];
    return dim == 0 ? p.x : (dim == 1 ? p.y : p.z);
  }

  // The squared distance from the query to the bounding box of all points, per axis.
  float boxDistance(const float query[3], float plane_distances[3]) const
  {
    float distance = 0.0f;
    for (int d = 0; d < 3; ++d) {
      float gap = 0.0f;
      if (query[d] < box_min_[d]) {
        gap = box_min_[d] - query[d];
      } else if (query[d] > box_max_[d]) {
        gap = query[d] - box_max_[d];
      }
      plane_distances[d] = gap * gap;
      distance += plane_distances[d];
    }
    return distance;
  }

  void build()
  {
    order_.clear();
    nodes_.clear();
    const auto & cloud = this->input_;
    if (!cloud || cloud->empty() ||
      cloud->size() > std::numeric_limits<std::uint32_t>::max())
    {
      return;
    }
    auto add_point = [this, &cloud](std::size_t index) {
        if (index < cloud->size() && finite((*cloud)[index])) {
          order_.push_back(static_cast<std::uint32_t>(index));
        }
      };
    if (this->indices_) {
      order_.reserve(this->indices_->size());
      for (const auto index : *this->indices_) {
        add_point(static_cast<std::size_t>(index));
      }
    } else {
      order_.reserve(cloud->size());
      for (std::size_t i = 0; i < cloud->size(); ++i) {
        add_point(i);
      }
    }
    order_.shrink_to_fit();
    if (order_.empty()) {
      return;
    }
    for (int d = 0; d < 3; ++d) {
      box_min_[d] = std::numeric_limits<float>::infinity();
      box_max_[d] = -std::numeric_limits<float>::infinity();
    }
    for (const auto index : order_) {
      for (int d = 0; d < 3; ++d) {
        box_min_[d] = std::min(box_min_[d], coordinate(index, d));
        box_max_[d] = std::max(box_max_[d], coordinate(index, d));
      }
    }
    nodes_.reserve(2 * (order_.size() / kLeafSize + 1));
    float low[3] = {box_min_[0], box_min_[1], box_min_[2]};
    float high[3] = {box_max_[0], box_max_[1], box_max_[2]};
    buildNode(0, static_cast<std::uint32_t>(order_.size()), low, high);
    nodes_.shrink_to_fit();
  }

  // Splits order_[begin, end) at its median along the widest side of its box.
  void buildNode(std::uint32_t begin, std::uint32_t end, float low[3], float high[3])
  {
    const std::size_t node = nodes_.size();
    nodes_.push_back(Node{0.0f, 0.0f, begin, end, 3});
    int dim = 0;
    for (int d = 1; d < 3; ++d) {
      if (high[d] - low[d] > high[dim] - low[dim]) {
        dim = d;
      }
    }
    if (end - begin <= kLeafSize || !(high[dim] > low[dim])) {
      return;
    }
    const std::uint32_t middle = begin + (end - begin) / 2;
    auto less = [this, dim](std::uint32_t a, std::uint32_t b) {
        return coordinate(a, dim) < coordinate(b, dim);
      };
    std::nth_element(order_.begin() + begin, order_.begin() + middle, order_.begin() + end, less);
    float left_high = -std::numeric_limits<float>::infinity();
    for (std::uint32_t i = begin; i < middle; ++i) {
      left_high = std::max(left_high, coordinate(order_[i], dim));
    }
    const float right_low = coordinate(order_[middle], dim);
    const float saved_high = high[dim];
    high[dim] = left_high;
    buildNode(begin, middle, low, high);
    high[dim] = saved_high;
    const float saved_low = low[dim];
    low[dim] = right_low;
    const auto second = static_cast<std::uint32_t>(nodes_.size());
    buildNode(middle, end, low, high);
    low[dim] = saved_low;
    nodes_[node] = Node{left_high, right_low, 0, second, static_cast<std::uint8_t>(dim)};
  }

  // Visits the nearer child first and the other one only if its box can hold a point
  // closer than the current worst result. min_distance is the squared distance from the
  // query to the node's box, plane_distances its per-axis terms (as in nanoflann).
  template<typename Result>
  void search(
    std::size_t node_index, const float query[3], float min_distance,
    float plane_distances[3], Result & result) const
  {
    const Node & node = nodes_[node_index];
    if (node.dim == 3) {
      for (std::uint32_t i = node.begin; i < node.end; ++i) {
        const PointT & p = (*this->input_)[order_[i]];
        const float dx = query[0] - p.x;
        const float dy = query[1] - p.y;
        const float dz = query[2] - p.z;
        float distance = dx * dx;
        distance += dy * dy;
        distance += dz * dz;
        result.add(distance, static_cast<pcl::index_t>(order_[i]));
      }
      return;
    }
    const int dim = node.dim;
    const float to_low = query[dim] - node.low;
    const float to_high = query[dim] - node.high;
    const bool first_nearer = to_low + to_high < 0.0f;
    const std::size_t nearer = first_nearer ? node_index + 1 : node.end;
    const std::size_t farther = first_nearer ? node.end : node_index + 1;
    const float cut = first_nearer ? to_high : to_low;
    search(nearer, query, min_distance, plane_distances, result);
    const float saved = plane_distances[dim];
    const float farther_distance = min_distance - saved + cut * cut;
    if (farther_distance < result.worst()) {
      plane_distances[dim] = cut * cut;
      search(farther, query, farther_distance, plane_distances, result);
      plane_distances[dim] = saved;
    }
  }

  std::vector<std::uint32_t> order_;
  std::vector<Node> nodes_;
  float box_min_[3]{};
  float box_max_[3]{};
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_COMPACT_KDTREE_HPP_
