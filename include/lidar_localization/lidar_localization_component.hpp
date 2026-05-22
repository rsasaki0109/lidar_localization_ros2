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
#include "lidar_localization/alignment_retry_policy.hpp"
#include "lidar_localization/alignment_pipeline_policy.hpp"
#include "lidar_localization/alignment_status_policy.hpp"
#include "lidar_localization/imu_preintegration_guard_policy.hpp"
#include "lidar_localization/measurement_gate_policy.hpp"
#include "lidar_localization/localization_update_policy.hpp"
#include "lidar_localization/pose_backend_result_policy.hpp"
#include "lidar_localization/pose_backend_selection_policy.hpp"
#include "lidar_localization/registration_observation_policy.hpp"
#include "lidar_localization/registration_seed_policy.hpp"
#include "lidar_localization/recovery_supervisor.hpp"
#include "lidar_localization/scan_preprocessing_policy.hpp"

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
  using RecoverySupervisorState = lidar_localization::RecoverySupervisorState;
  using ReinitializationRequestDecision =
    lidar_localization::ReinitializationRequestDecision;

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
  bool imu_preintegration_use_base_frame_transform_{false};
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
  bool viz_downsample_{false};
  double viz_voxel_leaf_size_{0.5};
  bool predict_pose_from_previous_delta_{true};
  lidar_localization::MeasurementGateParamConfig measurement_gate_config_;
  lidar_localization::RecoveryRetryFromLastPoseParams recovery_retry_from_last_pose_config_;
  bool enable_reinitialization_request_output_{true};
  lidar_localization::ReinitializationTriggerParams reinitialization_trigger_config_;

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

  RecoverySupervisorState recovery_supervisor_state_{RecoverySupervisorState::kTracking};
  std::string recovery_supervisor_action_{"idle"};
  double recovery_supervisor_state_entered_stamp_sec_{0.0};
  std::size_t recovery_supervisor_transition_count_{0};

  rclcpp::TimerBase::SharedPtr pose_publish_timer_;
  struct PreparedScanCloud
  {
    lidar_localization::ScanPreparationStatus status{
      lidar_localization::ScanPreparationStatus::kReady};
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    std::size_t filtered_point_count{0};
  };
  struct SelectedRegistrationSeed
  {
    Eigen::Matrix4f init_guess{Eigen::Matrix4f::Identity()};
    bool imu_prediction_ready{false};
  };
  struct ImuPreintegrationBackendUpdate
  {
    Eigen::Matrix4f pose{Eigen::Matrix4f::Identity()};
    bool updated{true};
    lidar_localization::ImuPreintegrationBackendState state;
    double smoother_measurement_translation_delta_m{std::numeric_limits<double>::quiet_NaN()};
    double smoother_measurement_rotation_delta_deg{std::numeric_limits<double>::quiet_NaN()};
  };
  struct PlanarSmootherBackendUpdate
  {
    Eigen::Matrix4f pose{Eigen::Matrix4f::Identity()};
    bool updated{true};
    const char * rejected_status_message{"smoother_update_rejected"};
  };
  struct PoseBackendApplyContext
  {
    const lidar_localization::RegistrationObservation & observation;
    const lidar_localization::AlignmentAttempt & attempt;
    const lidar_localization::MeasurementGateDecision & gate_result;
    const builtin_interfaces::msg::Time & stamp;
    std::size_t filtered_point_count;
    double stamp_sec;
    bool imu_prediction_ready;
  };
  struct AlignmentStatusPublishInput
  {
    const builtin_interfaces::msg::Time & stamp;
    uint8_t level;
    const std::string & message;
    bool has_converged;
    double fitness_score;
    double alignment_time_sec;
    std::size_t filtered_point_count;
    double correction_translation_m;
    double correction_yaw_deg;
    double seed_translation_since_accept_m;
    double seed_yaw_since_accept_deg;
    double accepted_gap_sec;
    bool imu_prediction_active;
  };
  struct AlignmentStatusEvaluation
  {
    lidar_localization::AlignmentStatusInput status_input;
    lidar_localization::AlignmentStatusPreparation status_preparation;
    ReinitializationRequestDecision reinitialization_request;
  };
  void timerPublishPose();
  bool admitScanMessage(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
    double * scan_stamp_sec);
  PreparedScanCloud prepareScanForRegistration(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
    double scan_stamp_sec);
  void handleScanPreparationFailure(
    const builtin_interfaces::msg::Time & stamp,
    const PreparedScanCloud & prepared_scan,
    double scan_stamp_sec);
  SelectedRegistrationSeed selectRegistrationSeed(double scan_stamp_sec);
  Eigen::Matrix4f refineSeedWithNdtInitializer(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & source_cloud,
    const Eigen::Matrix4f & init_guess);
  bool setInputTargetForPose(const Eigen::Matrix4f & center_pose_matrix);
  lidar_localization::AlignmentAttempt runAlignmentAttempt(
    const Eigen::Matrix4f & attempt_init_guess,
    const Eigen::Matrix4f & crop_center_pose_matrix,
    double scan_stamp_sec);
  lidar_localization::AlignmentPipelineResult runAlignmentPipelineForScan(
    const Eigen::Matrix4f & init_guess,
    double scan_stamp_sec);
  lidar_localization::MeasurementGateDecision evaluateMeasurementGateForAttempt(
    const lidar_localization::AlignmentAttempt & attempt);
  void logAlignmentPipelineRecovery(
    const lidar_localization::AlignmentPipelineResult & pipeline_result);
  bool handleTerminalAlignmentPipelineResult(
    const builtin_interfaces::msg::Time & stamp,
    const lidar_localization::AlignmentPipelineResult & pipeline_result,
    std::size_t filtered_point_count,
    double scan_stamp_sec,
    bool imu_prediction_ready);
  bool applyAcceptedAlignmentPipelineResult(
    const builtin_interfaces::msg::Time & stamp,
    const lidar_localization::AlignmentPipelineResult & pipeline_result,
    std::size_t filtered_point_count,
    double scan_stamp_sec,
    bool imu_prediction_ready);
  void setRegistrationSourceCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr & source_cloud);
  void printAlignmentDebugInfo(
    const Eigen::Matrix4f & init_guess,
    const lidar_localization::AlignmentAttempt & selected_attempt,
    std::size_t filtered_point_count) const;
  Eigen::Matrix4f currentPoseMatrix() const;
  Eigen::Matrix4f applyTwistPrediction(const Eigen::Matrix4f & pose_matrix, double dt_sec) const;
  void resetPredictionState(const Eigen::Matrix4f & pose_matrix, double stamp_sec);
  void updatePredictionState(const Eigen::Matrix4f & accepted_pose_matrix, double stamp_sec);
  void advancePredictionWithoutMeasurement(double stamp_sec);
  void updatePredictionFromRejectedMeasurement(
    const Eigen::Matrix4f & rejected_pose_matrix,
    double stamp_sec);
  bool applyRegistrationPoseBackend(
    const lidar_localization::RegistrationObservation & observation,
    const lidar_localization::AlignmentAttempt & attempt,
    const lidar_localization::MeasurementGateDecision & gate_result,
    const builtin_interfaces::msg::Time & stamp,
    std::size_t filtered_point_count,
    double stamp_sec,
    bool imu_prediction_ready);
  lidar_localization::PoseBackendKind selectRegistrationPoseBackend() const;
  bool dispatchRegistrationPoseBackend(
    lidar_localization::PoseBackendKind backend,
    const PoseBackendApplyContext & context);
  bool applyGtsamPoseBackend(const PoseBackendApplyContext & context);
  void updateBackendRollPitch(
    const lidar_localization::RegistrationObservation & observation);
  PlanarSmootherBackendUpdate updateGtsamPoseBackend(
    const lidar_localization::RegistrationObservation & observation,
    const lidar_localization::AlignmentAttempt & attempt,
    double stamp_sec);
  PlanarSmootherBackendUpdate updateTwistEkfPoseBackend(
    const lidar_localization::RegistrationObservation & observation,
    const lidar_localization::AlignmentAttempt & attempt,
    double stamp_sec);
  bool applyPlanarSmootherPoseBackendUpdate(
    const PlanarSmootherBackendUpdate & update,
    const PoseBackendApplyContext & context);
  bool applyImuPreintegrationPoseBackend(const PoseBackendApplyContext & context);
  lidar_localization::ImuPreintegrationGuardParams imuPreintegrationGuardParams() const;
  void initializeImuPreintegrationSmootherIfNeeded(
    const lidar_localization::RegistrationObservation & observation,
    double stamp_sec);
  ImuPreintegrationBackendUpdate updateImuPreintegrationBackend(
    const lidar_localization::RegistrationObservation & observation,
    const lidar_localization::AlignmentAttempt & attempt,
    double stamp_sec,
    bool imu_prediction_ready);
  void logImuPreintegrationBackendWarnings(
    const ImuPreintegrationBackendUpdate & update,
    const lidar_localization::AlignmentAttempt & attempt);
  lidar_localization::PoseBackendResult makeImuPreintegrationPoseBackendResult(
    const ImuPreintegrationBackendUpdate & update,
    uint8_t status_level,
    const std::string & status_message) const;
  bool applyTwistEkfPoseBackend(const PoseBackendApplyContext & context);
  bool applyRawRegistrationPoseBackend(const PoseBackendApplyContext & context);
  bool applyPoseBackendResult(
    const lidar_localization::PoseBackendResult & result,
    const builtin_interfaces::msg::Time & stamp,
    double stamp_sec,
    double fitness_score);
  bool applyPoseBackendPredictionOnlyResult(
    const lidar_localization::PoseBackendResult & result,
    double stamp_sec);
  void applyAcceptedPoseBackendResult(
    const lidar_localization::PoseBackendResult & result,
    const builtin_interfaces::msg::Time & stamp,
    double stamp_sec,
    double fitness_score);
  void setCurrentPoseFromMatrix(
    const Eigen::Matrix4f & pose_matrix,
    const builtin_interfaces::msg::Time & stamp);
  void publishCurrentPose(const builtin_interfaces::msg::Time & stamp);
  void publishPoseMessage(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose);
  void appendCurrentPoseToPath(
    const builtin_interfaces::msg::Time & stamp,
    const geometry_msgs::msg::Pose & pose);
  void publishPathMessage();
  bool publishPoseTransform(
    const builtin_interfaces::msg::Time & stamp,
    const geometry_msgs::msg::Pose & pose);
  bool publishMapToOdomTransform(
    const builtin_interfaces::msg::Time & stamp,
    const geometry_msgs::msg::TransformStamped & map_to_base_link_stamped);
  void fillPoseCovariance(double fitness_score);
  void publishAlignmentStatusForAttempt(
    const builtin_interfaces::msg::Time & stamp,
    uint8_t level,
    const std::string & message,
    const lidar_localization::AlignmentAttempt & attempt,
    std::size_t filtered_point_count,
    bool imu_prediction_active);
  ReinitializationRequestDecision applyReinitializationRequestLatch(
    const builtin_interfaces::msg::Time & stamp,
    const ReinitializationRequestDecision & decision);
  void updateRecoverySupervisorState(
    const builtin_interfaces::msg::Time & stamp,
    RecoverySupervisorState next_state,
    const std::string & action);
  void publishReinitializationRequest(
    const builtin_interfaces::msg::Time & stamp,
    const ReinitializationRequestDecision & decision);
  lidar_localization::MeasurementGateParams measurementGateParams() const;
  lidar_localization::ReinitializationTriggerParams reinitializationTriggerParams() const;
  lidar_localization::RecoveryRetryFromLastPoseParams recoveryRetryFromLastPoseParams() const;
  lidar_localization::AlignmentStatusInput makeAlignmentStatusInput(
    const AlignmentStatusPublishInput & input) const;
  lidar_localization::AlignmentStatusRuntimeContext makeAlignmentStatusRuntimeContext(
    double stamp_sec) const;
  diagnostic_msgs::msg::DiagnosticStatus makeAlignmentDiagnosticStatus(
    const lidar_localization::AlignmentStatusInput & status_input) const;
  AlignmentStatusEvaluation evaluateAlignmentStatus(
    const builtin_interfaces::msg::Time & stamp,
    const AlignmentStatusPublishInput & publish_input);
  lidar_localization::AlignmentDiagnosticValuesInput prepareAlignmentDiagnosticValuesInput(
    const lidar_localization::AlignmentStatusInput & status_input,
    const lidar_localization::AlignmentStatusPreparation & status_preparation,
    const ReinitializationRequestDecision & reinitialization_request) const;
  void appendAlignmentDiagnosticValues(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const lidar_localization::AlignmentDiagnosticValuesInput & diagnostic_values_input) const;
  diagnostic_msgs::msg::DiagnosticArray makeAlignmentDiagnosticArray(
    const builtin_interfaces::msg::Time & stamp,
    const diagnostic_msgs::msg::DiagnosticStatus & status) const;
  void publishAlignmentDiagnosticStatus(
    const builtin_interfaces::msg::Time & stamp,
    const diagnostic_msgs::msg::DiagnosticStatus & status);
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
