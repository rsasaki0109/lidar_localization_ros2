#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
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
#include "std_msgs/msg/bool.hpp"

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

#ifdef LIDAR_LOCALIZATION_HAVE_NAV2_BOND
#include "bondcpp/bond.hpp"
#endif

using namespace std::chrono_literals;

class PCLLocalization : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PCLLocalization(const rclcpp::NodeOptions & options);
  ~PCLLocalization() override;

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
  void releaseRuntimeResources(bool leak_target_clouds_for_shutdown = false);
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
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr
    reinitialization_request_pub_;
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
  pcl::PointXYZI map_min_pt_{};
  pcl::PointXYZI map_max_pt_{};
  bool map_bounds_valid_{false};
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> recent_source_clouds_;
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> recent_target_clouds_;
  bool use_local_map_crop_{false};
  bool enable_local_map_crop_{false};
  double local_map_radius_{150.0};
  std::size_t local_map_min_points_{100};
  int consecutive_crop_failures_{0};
  bool crop_failure_guard_active_{false};
  std::chrono::steady_clock::time_point last_crop_out_of_bounds_log_time_{};
  std::chrono::steady_clock::time_point last_crop_failure_streak_log_time_{};

  // NDT initializer for GICP-based methods
  bool use_ndt_initializer_{false};
  int ndt_init_scan_count_{0};
  int ndt_init_scans_required_{5};
  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_initializer_;

  // IMU preintegration smoother
  bool use_imu_preintegration_{false};
  double imu_prediction_correction_guard_translation_m_{2.0};
  double imu_prediction_correction_guard_yaw_deg_{4.0};
  ImuGtsamSmoother imu_smoother_;
  double last_imu_stamp_{0.0};
  double last_scan_stamp_for_imu_{0.0};
  bool imu_preintegration_fallback_mode_{false};

#ifdef LIDAR_LOCALIZATION_HAVE_NAV2_BOND
  // Nav2 lifecycle manager bond
  std::unique_ptr<bond::Bond> bond_;
  bool use_bond_{false};
#endif

  // parameters
  std::string global_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string registration_method_;
  double scan_max_range_;
  double scan_min_range_;
  double scan_period_;
  int cloud_queue_depth_{1};
  double min_scan_interval_sec_{0.0};
  rclcpp::Time last_cloud_process_time_{0, 0, RCL_ROS_TIME};
  double score_threshold_;
  double ndt_resolution_;
  double ndt_step_size_;
  double transform_epsilon_;
  double voxel_leaf_size_;
  bool enable_scan_voxel_filter_{true};
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
  bool enable_consistency_recovery_gate_{false};
  int consistency_recovery_min_rejections_{10};
  double consistency_recovery_score_margin_{2.0};
  double consistency_recovery_max_translation_m_{0.05};
  double consistency_recovery_max_yaw_deg_{0.5};
  bool enable_post_reject_strict_score_threshold_{false};
  int post_reject_strict_min_rejections_{100};
  double post_reject_strict_score_threshold_{5.5};
  bool enable_open_loop_strict_score_threshold_{false};
  double open_loop_strict_min_accepted_gap_sec_{15.0};
  double open_loop_strict_min_seed_translation_m_{100.0};
  double open_loop_strict_score_threshold_{5.25};
  bool enable_borderline_seed_rejection_gate_{false};
  double borderline_seed_gate_score_threshold_{5.25};
  double borderline_seed_gate_min_seed_translation_m_{1.0};
  bool enable_rejected_seed_update_{false};
  int rejected_seed_update_min_rejections_{0};
  double rejected_seed_update_max_fitness_{10.0};
  double rejected_seed_update_max_correction_translation_m_{2.0};
  double rejected_seed_update_max_correction_yaw_deg_{2.0};
  bool enable_recovery_retry_from_last_pose_{false};
  int recovery_retry_from_last_pose_min_rejections_{1};
  double recovery_retry_from_last_pose_max_accepted_gap_sec_{1.0};
  double recovery_retry_from_last_pose_max_seed_translation_m_{1000000000.0};
  bool enable_reinitialization_request_output_{true};
  double reinitialization_trigger_threshold_{0.95};
  double reinitialization_trigger_gap_scale_sec_{30.0};
  double reinitialization_trigger_seed_translation_scale_m_{100.0};
  double reinitialization_trigger_reject_streak_scale_{200.0};
  double reinitialization_trigger_fitness_explosion_threshold_{1000.0};

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
  double last_accepted_pose_time_sec_{0.0};
  double predicted_pose_time_sec_{0.0};
  geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr latest_twist_msg_;
  bool shutting_down_{false};
  bool reinitialization_requested_{false};
  std::string reinitialization_request_reason_{"not_requested"};
  double reinitialization_request_score_{0.0};
  bool enable_reinitialization_request_latch_{true};
  bool reinitialization_request_latched_{false};
  std::string reinitialization_request_latch_reason_{"not_requested"};
  double reinitialization_request_latch_score_{0.0};
  double reinitialization_request_latch_stamp_sec_{0.0};

  enum class RecoverySupervisorState : uint8_t
  {
    kTracking = 0,
    kDegraded = 1,
    kRecovering = 2,
    kReinitializationRequested = 3,
  };
  RecoverySupervisorState recovery_supervisor_state_{RecoverySupervisorState::kTracking};
  std::string recovery_supervisor_action_{"idle"};
  double recovery_supervisor_state_entered_stamp_sec_{0.0};
  std::size_t recovery_supervisor_transition_count_{0};

  rclcpp::TimerBase::SharedPtr pose_publish_timer_;
  void timerPublishPose();
  Eigen::Matrix4f currentPoseMatrix() const;
  Eigen::Matrix4f applyTwistPrediction(const Eigen::Matrix4f & pose_matrix, double dt_sec) const;
  void resetPredictionState(const Eigen::Matrix4f & pose_matrix, double stamp_sec);
  void updatePredictionState(const Eigen::Matrix4f & accepted_pose_matrix, double stamp_sec);
  void advancePredictionWithoutMeasurement(double stamp_sec);
  void updatePredictionFromRejectedMeasurement(
    const Eigen::Matrix4f & rejected_pose_matrix,
    double stamp_sec);
  void setCurrentPoseFromMatrix(
    const Eigen::Matrix4f & pose_matrix,
    const builtin_interfaces::msg::Time & stamp);
  void publishCurrentPose(const builtin_interfaces::msg::Time & stamp);
  void fillPoseCovariance(double fitness_score);
  double computeEffectiveScoreThreshold(
    double accepted_gap_sec,
    double seed_translation_since_accept_m,
    bool * post_reject_strict_active = nullptr,
    bool * open_loop_strict_active = nullptr) const;
  bool computeBorderlineSeedGateActive(
    double fitness_score,
    double seed_translation_since_accept_m,
    double effective_score_threshold) const;
  struct ReinitializationRequestDecision
  {
    bool requested{false};
    std::string reason{"not_requested"};
    double score{0.0};
  };
  ReinitializationRequestDecision computeReinitializationRequest(
    const builtin_interfaces::msg::Time & stamp,
    const std::string & status_message,
    double fitness_score,
    double seed_translation_since_accept_m,
    double accepted_gap_sec) const;
  ReinitializationRequestDecision applyReinitializationRequestLatch(
    const builtin_interfaces::msg::Time & stamp,
    const ReinitializationRequestDecision & decision);
  const char * recoverySupervisorStateName(RecoverySupervisorState state) const;
  bool isRecoveryFailureStatus(const std::string & status_message) const;
  RecoverySupervisorState classifyRecoverySupervisorState(
    uint8_t level,
    const std::string & status_message,
    const ReinitializationRequestDecision & reinitialization_request) const;
  std::string classifyRecoverySupervisorAction(
    uint8_t level,
    const std::string & status_message,
    const ReinitializationRequestDecision & reinitialization_request) const;
  void updateRecoverySupervisorState(
    const builtin_interfaces::msg::Time & stamp,
    RecoverySupervisorState next_state,
    const std::string & action);
  void publishReinitializationRequest(
    const builtin_interfaces::msg::Time & stamp,
    const ReinitializationRequestDecision & decision);
  void publishAlignmentStatus(
    const builtin_interfaces::msg::Time & stamp,
    uint8_t level,
    const std::string & message,
    bool has_converged,
    double fitness_score,
    double alignment_time_sec,
    std::size_t filtered_point_count,
    double correction_translation_m = std::numeric_limits<double>::quiet_NaN(),
    double correction_yaw_deg = std::numeric_limits<double>::quiet_NaN(),
    double seed_translation_since_accept_m = std::numeric_limits<double>::quiet_NaN(),
    double seed_yaw_since_accept_deg = std::numeric_limits<double>::quiet_NaN(),
    double accepted_gap_sec = std::numeric_limits<double>::quiet_NaN(),
    bool imu_prediction_active = false);
};
