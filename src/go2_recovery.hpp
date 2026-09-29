#pragma once
#include <deque>
#include <mutex>
#include <vector>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
struct Go2Imu
{
  int64_t stamp, receipt;
  Eigen::Vector3d w;
};
struct Go2Recovery
{
  std::mutex imu_mutex;
  std::deque<Go2Imu> imu_history;
  uint64_t input_generation = 0;
  bool tried = false, pending = false;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr previous;
  std::vector<Go2Imu> previous_imu;
  Eigen::Matrix4f pose;
  double stamp = 0;
  uint64_t generation = 0;
  // Caller holds component state lock. IMU callbacks take only imu_mutex.
  void reset(uint64_t next_generation)
  {
    std::lock_guard<std::mutex> lock(imu_mutex);
    imu_history.clear();
    input_generation = next_generation;
    tried = false;
    pending = false;
    previous.reset();
    previous_imu.clear();
    generation = next_generation;
  }
};

// Receipt timestamps use steady-clock nanoseconds, independently of ROS source time.
int64_t go2RecoveryNow();
void go2ReceiveImu(const sensor_msgs::msg::Imu &, Go2Recovery &, uint64_t generation);
std::vector<Go2Imu> go2ImuSnapshot(int64_t cutoff, Go2Recovery &, uint64_t generation);
std::vector<Eigen::Matrix4f> go2BbsSeeds(
  const std::string &, const sensor_msgs::msg::PointCloud2 &,
  const Eigen::Matrix4d &, const Eigen::Matrix4d &);
Eigen::Matrix4f go2Delta(
  const sensor_msgs::msg::PointCloud2 &, const sensor_msgs::msg::PointCloud2 &,
  const std::vector<Go2Imu> &, const std::vector<Go2Imu> &);
