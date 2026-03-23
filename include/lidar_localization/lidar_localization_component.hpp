#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <Eigen/Geometry>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/ply_io.h>

#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>

#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
#include <small_gicp/pcl/pcl_registration.hpp>
#endif

#include "lidar_localization/lidar_undistortion.hpp"
#include "lidar_localization/twist_ekf.hpp"
#include "lidar_localization/twist_gtsam_smoother.hpp"
#include "lidar_localization/imu_gtsam_smoother.hpp"

using namespace std::chrono_literals;

class PCLLocalization : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PCLLocalization(const rclcpp::NodeOptions & options);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & state);

  void initializeParameters();
  void initializePubSub();
  void initializeRegistration();
  void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void twistReceived(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);
  void imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  // void gnssReceived();

  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Clock clock_;
  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener tflistener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr
    initial_pose_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    initial_map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr
    map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr
    odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::ConstSharedPtr
    twist_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr
    cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr
    imu_sub_;

  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI> * registration_{nullptr};
  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr pcl_registration_;
  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp_registration_;
  pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp_registration_;
#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
  small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>::Ptr small_gicp_registration_;
#endif
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr corrent_pose_with_cov_stamped_ptr_;
  nav_msgs::msg::Path::SharedPtr path_ptr_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_scan_ptr_;

  bool map_recieved_{false};
  bool initialpose_recieved_{false};
  pcl::PointCloud<pcl::PointXYZI>::Ptr full_map_cloud_ptr_;
  bool use_local_map_crop_{false};
  double local_map_radius_{150.0};

  // NDT initializer for GICP-based methods
  bool use_ndt_initializer_{false};
  int ndt_init_scan_count_{0};
  int ndt_init_scans_required_{5};
  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_initializer_;

  // IMU preintegration smoother
  bool use_imu_preintegration_{false};
  ImuGtsamSmoother imu_smoother_;
  std::deque<ImuSample> imu_buffer_;
  double last_imu_stamp_{0.0};
  double last_scan_stamp_for_imu_{0.0};

  // parameters
  std::string global_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string registration_method_;
  double scan_max_range_;
  double scan_min_range_;
  double scan_period_;
  double score_threshold_;
  double ndt_resolution_;
  double ndt_step_size_;
  double transform_epsilon_;
  double voxel_leaf_size_;
  int gicp_corr_randomness_;
  double gicp_max_correspondence_distance_;
  double vgicp_voxel_resolution_;
  bool use_pcd_map_{false};
  std::string map_path_;
  bool set_initial_pose_{false};
  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_z_;
  double initial_pose_qx_;
  double initial_pose_qy_;
  double initial_pose_qz_;
  double initial_pose_qw_;

  bool use_odom_{false};
  double last_odom_received_time_;
  bool use_twist_prediction_{false};
  bool twist_prediction_use_angular_velocity_{true};
  double max_twist_prediction_dt_{0.5};
  bool use_imu_{false};
  bool use_twist_ekf_{false};
  bool use_gtsam_smoother_{false};
  bool enable_debug_{false};
  bool enable_map_odom_tf_{false};
  bool predict_pose_from_previous_delta_{true};
  bool reject_above_score_threshold_{true};

  int ndt_num_threads_;
  int ndt_max_iterations_;

  double pose_publish_frequency_;
  bool enable_timer_publishing_{false};

  // imu
  LidarUndistortion lidar_undistortion_;

  // twist EKF
  TwistEkf twist_ekf_;
  // GTSAM smoother
  TwistGtsamSmoother gtsam_smoother_;
  float last_ndt_roll_{0.0f};
  float last_ndt_pitch_{0.0f};
  bool have_last_accepted_pose_{false};
  Eigen::Matrix4f last_accepted_pose_matrix_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f predicted_pose_matrix_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f last_relative_motion_matrix_{Eigen::Matrix4f::Identity()};
  std::size_t consecutive_rejected_updates_{0};
  double predicted_pose_time_sec_{0.0};
  geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr latest_twist_msg_;

  rclcpp::TimerBase::SharedPtr pose_publish_timer_;
  void timerPublishPose();
  Eigen::Matrix4f currentPoseMatrix() const;
  Eigen::Matrix4f applyTwistPrediction(const Eigen::Matrix4f & pose_matrix, double dt_sec) const;
  void resetPredictionState(const Eigen::Matrix4f & pose_matrix, double stamp_sec);
  void updatePredictionState(const Eigen::Matrix4f & accepted_pose_matrix, double stamp_sec);
  void advancePredictionWithoutMeasurement(double stamp_sec);
  void fillPoseCovariance(double fitness_score);
  void publishAlignmentStatus(
    const builtin_interfaces::msg::Time & stamp,
    uint8_t level,
    const std::string & message,
    bool has_converged,
    double fitness_score,
    double alignment_time_sec,
    std::size_t filtered_point_count);
};
