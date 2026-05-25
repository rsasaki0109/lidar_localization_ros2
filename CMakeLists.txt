cmake_minimum_required(VERSION 3.5)
project(lidar_localization_ros2 VERSION 1.0.0)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(tf2_sensor_msgs  REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(small_gicp QUIET CONFIG)
find_package(nav2_util QUIET)
find_package(bondcpp QUIET)
find_package(OpenMP)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


add_library(lidar_localization_component SHARED
src/lidar_localization_component.cpp
)

ament_target_dependencies(lidar_localization_component
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  diagnostic_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
  ndt_omp_ros2
)

if(small_gicp_FOUND)
  target_compile_definitions(lidar_localization_component PUBLIC LIDAR_LOCALIZATION_HAVE_SMALL_GICP=1)
  target_link_libraries(lidar_localization_component small_gicp::small_gicp)
endif()

if(nav2_util_FOUND AND bondcpp_FOUND)
  target_compile_definitions(lidar_localization_component PUBLIC LIDAR_LOCALIZATION_HAVE_NAV2_BOND=1)
  ament_target_dependencies(lidar_localization_component nav2_util bondcpp)
  message(STATUS "Nav2 bond support enabled")
endif()



include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

add_executable(lidar_localization_node src/lidar_localization_node.cpp)
target_link_libraries(lidar_localization_node
  lidar_localization_component
  ${PCL_LIBRARIES}
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

add_executable(relocalization_ndt_score_jobs src/relocalization_ndt_score_jobs.cpp)
ament_target_dependencies(relocalization_ndt_score_jobs
  ndt_omp_ros2
)
target_link_libraries(relocalization_ndt_score_jobs
  -Wl,--start-group
  ${ndt_omp_ros2_LIBRARIES}
  ${PCL_LIBRARIES}
  -Wl,--end-group
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

ament_export_libraries(lidar_localization_component)

install(TARGETS
  lidar_localization_node
  relocalization_ndt_score_jobs
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  lidar_localization_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(PROGRAMS
  scripts/benchmark_analyze_drift_window
  scripts/benchmark_compare_runs
  scripts/check_mid360_legged_bringup.py
  scripts/compare_nav2_reinit_supervisor_runs.py
  scripts/convert_boreas_sequence_to_rosbag2.py
  scripts/benchmark_extract_pose_reference_from_rosbag2
  scripts/benchmark_from_manifest
  scripts/benchmark_make_graph_dataset
  scripts/benchmark_pose_recorder
  scripts/benchmark_diagnostic_recorder
  scripts/build_relocalization_demo_report.py
  scripts/benchmark_runner
  scripts/benchmark_sweep_localizer
  scripts/benchmark_eval_trajectory
  scripts/benchmark_eval_evo_ape
  scripts/augment_pointcloud_intensity.py
  scripts/make_disabled_relocalization_attempts.py
  scripts/observe_relocalization_reset_execution.py
  scripts/make_route_grid_relocalization_attempts.py
  scripts/make_registration_relocalization_jobs.py
  scripts/scaffold_mapless_public_dataset_bundle.py
  scripts/fetch_official_autoware_istanbul_dataset.sh
  scripts/fetch_koide_hard_pointcloud_localization_dataset.sh
  scripts/fetch_official_hdl_localization_sample.sh
  scripts/build_gt_aligned_map_from_reference_csv.py
  scripts/tum_trajectory_to_pose_reference_csv.py
  scripts/tum_trajectory_to_pose_reference_csv_for_rosbag2.py
  scripts/generate_occupancy_map_from_pcd.py
  scripts/make_localization_comparison_report.py
  scripts/publish_cmd_vel_odom.py
  scripts/publish_identity_odom.py
  scripts/publish_odom_from_localization.py
  scripts/publish_odom_from_twist.py
  scripts/publish_pose_from_odom.py
  scripts/publish_relocalization_reset_commands.py
  scripts/republish_initialpose_on_reinit.py
  scripts/relay_localization_inputs_with_current_stamp.py
  scripts/record_hdl_localization_outputs.py
  scripts/run_borderline_gate_experiments.py
  scripts/run_experiment_suite.py
  scripts/run_imu_guard_experiments.py
  scripts/run_nav2_reinit_supervisor_sweep.sh
  scripts/run_nav2_reinit_supervisor_regression.sh
  scripts/run_recovery_action_experiments.py
  scripts/run_reinit_trigger_experiments.py
  scripts/run_public_regression_suite.sh
  scripts/run_release_regression_suite.sh
  scripts/run_nav2_demo_smoke
  scripts/run_nav2_replay_smoke
  scripts/send_nav2_goal.py
  scripts/summarize_localization_health.py
  scripts/summarize_relocalization_attempts.py
  scripts/summarize_registration_relocalization_scores.py
  scripts/compare_registration_relocalization_score_summaries.py
  scripts/run_registration_ordering_comparison.py
  scripts/make_relocalization_reset_commands.py
  scripts/select_relocalization_reset_candidates.py
  scripts/validate_relocalization_reset_commands.py
  scripts/validate_relocalization_reset_candidate_plan.py
  scripts/score_relocalization_candidates_with_reference.py
  scripts/resolve_registration_relocalization_scans.py
  scripts/run_autoware_istanbul_60s_predictor_compare.sh
  scripts/run_hdl_localization_docker_benchmark.sh
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  scripts/lidar_localization_mid360
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

install(DIRECTORY
  launch
  docs
  experiments
  param
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  add_executable(test_registration_seed_policy
    test/test_registration_seed_policy.cpp)
  target_include_directories(test_registration_seed_policy PRIVATE
    include)
  add_test(
    NAME registration_seed_policy
    COMMAND test_registration_seed_policy
  )

  add_executable(test_scan_admission_policy
    test/test_scan_admission_policy.cpp)
  target_include_directories(test_scan_admission_policy PRIVATE
    include)
  add_test(
    NAME scan_admission_policy
    COMMAND test_scan_admission_policy
  )

  add_executable(test_ndt_initializer_policy
    test/test_ndt_initializer_policy.cpp)
  target_include_directories(test_ndt_initializer_policy PRIVATE
    include)
  add_test(
    NAME ndt_initializer_policy
    COMMAND test_ndt_initializer_policy
  )

  add_executable(test_registration_backend_policy
    test/test_registration_backend_policy.cpp)
  target_include_directories(test_registration_backend_policy PRIVATE
    include)
  add_test(
    NAME registration_backend_policy
    COMMAND test_registration_backend_policy
  )

  add_executable(test_alignment_diagnostics_policy
    test/test_alignment_diagnostics_policy.cpp)
  target_include_directories(test_alignment_diagnostics_policy PRIVATE
    include)
  add_test(
    NAME alignment_diagnostics_policy
    COMMAND test_alignment_diagnostics_policy
  )

  add_executable(test_alignment_diagnostic_ros_adapter
    test/test_alignment_diagnostic_ros_adapter.cpp)
  target_include_directories(test_alignment_diagnostic_ros_adapter PRIVATE
    include)
  ament_target_dependencies(test_alignment_diagnostic_ros_adapter
    diagnostic_msgs)
  add_test(
    NAME alignment_diagnostic_ros_adapter
    COMMAND test_alignment_diagnostic_ros_adapter
  )

  add_executable(test_alignment_status_policy
    test/test_alignment_status_policy.cpp)
  target_include_directories(test_alignment_status_policy PRIVATE
    include)
  add_test(
    NAME alignment_status_policy
    COMMAND test_alignment_status_policy
  )

  add_executable(test_alignment_attempt_policy
    test/test_alignment_attempt_policy.cpp)
  target_include_directories(test_alignment_attempt_policy PRIVATE
    include)
  target_link_libraries(test_alignment_attempt_policy
    Eigen3::Eigen)
  add_test(
    NAME alignment_attempt_policy
    COMMAND test_alignment_attempt_policy
  )

  add_executable(test_alignment_retry_policy
    test/test_alignment_retry_policy.cpp)
  target_include_directories(test_alignment_retry_policy PRIVATE
    include)
  add_test(
    NAME alignment_retry_policy
    COMMAND test_alignment_retry_policy
  )

  add_executable(test_alignment_pipeline_policy
    test/test_alignment_pipeline_policy.cpp)
  target_include_directories(test_alignment_pipeline_policy PRIVATE
    include)
  target_link_libraries(test_alignment_pipeline_policy
    Eigen3::Eigen)
  add_test(
    NAME alignment_pipeline_policy
    COMMAND test_alignment_pipeline_policy
  )

  add_executable(test_registration_observation_policy
    test/test_registration_observation_policy.cpp)
  target_include_directories(test_registration_observation_policy PRIVATE
    include)
  target_link_libraries(test_registration_observation_policy
    Eigen3::Eigen)
  add_test(
    NAME registration_observation_policy
    COMMAND test_registration_observation_policy
  )

  add_executable(test_pose_backend_selection_policy
    test/test_pose_backend_selection_policy.cpp)
  target_include_directories(test_pose_backend_selection_policy PRIVATE
    include)
  add_test(
    NAME pose_backend_selection_policy
    COMMAND test_pose_backend_selection_policy
  )

  add_executable(test_pose_backend_result_policy
    test/test_pose_backend_result_policy.cpp)
  target_include_directories(test_pose_backend_result_policy PRIVATE
    include)
  target_link_libraries(test_pose_backend_result_policy
    Eigen3::Eigen)
  add_test(
    NAME pose_backend_result_policy
    COMMAND test_pose_backend_result_policy
  )

  add_executable(test_imu_preintegration_guard_policy
    test/test_imu_preintegration_guard_policy.cpp)
  target_include_directories(test_imu_preintegration_guard_policy PRIVATE
    include)
  add_test(
    NAME imu_preintegration_guard_policy
    COMMAND test_imu_preintegration_guard_policy
  )

  add_executable(test_prediction_state_policy
    test/test_prediction_state_policy.cpp)
  target_include_directories(test_prediction_state_policy PRIVATE
    include)
  target_link_libraries(test_prediction_state_policy
    Eigen3::Eigen)
  add_test(
    NAME prediction_state_policy
    COMMAND test_prediction_state_policy
  )

  add_executable(test_pose_publish_policy
    test/test_pose_publish_policy.cpp)
  target_include_directories(test_pose_publish_policy PRIVATE
    include)
  ament_target_dependencies(test_pose_publish_policy
    geometry_msgs
    nav_msgs
    tf2_geometry_msgs)
  target_link_libraries(test_pose_publish_policy
    Eigen3::Eigen)
  add_test(
    NAME pose_publish_policy
    COMMAND test_pose_publish_policy
  )

  add_executable(test_pose_covariance_policy
    test/test_pose_covariance_policy.cpp)
  target_include_directories(test_pose_covariance_policy PRIVATE
    include)
  target_link_libraries(test_pose_covariance_policy
    Eigen3::Eigen)
  add_test(
    NAME pose_covariance_policy
    COMMAND test_pose_covariance_policy
  )

  add_executable(test_reinitialization_latch_policy
    test/test_reinitialization_latch_policy.cpp)
  target_include_directories(test_reinitialization_latch_policy PRIVATE
    include)
  add_test(
    NAME reinitialization_latch_policy
    COMMAND test_reinitialization_latch_policy
  )

  add_executable(test_reinitialization_request_output_policy
    test/test_reinitialization_request_output_policy.cpp)
  target_include_directories(test_reinitialization_request_output_policy PRIVATE
    include)
  add_test(
    NAME reinitialization_request_output_policy
    COMMAND test_reinitialization_request_output_policy
  )

  add_executable(test_recovery_supervisor_state_policy
    test/test_recovery_supervisor_state_policy.cpp)
  target_include_directories(test_recovery_supervisor_state_policy PRIVATE
    include)
  add_test(
    NAME recovery_supervisor_state_policy
    COMMAND test_recovery_supervisor_state_policy
  )

  add_executable(test_parameter_validation_policy
    test/test_parameter_validation_policy.cpp)
  target_include_directories(test_parameter_validation_policy PRIVATE
    include)
  add_test(
    NAME parameter_validation_policy
    COMMAND test_parameter_validation_policy
  )

  add_executable(test_point_cloud_conversion
    test/test_point_cloud_conversion.cpp)
  target_include_directories(test_point_cloud_conversion PRIVATE
    include
    ${PCL_INCLUDE_DIRS})
  ament_target_dependencies(test_point_cloud_conversion
    pcl_conversions
    sensor_msgs)
  target_link_libraries(test_point_cloud_conversion
    ${PCL_LIBRARIES})
  add_test(
    NAME point_cloud_conversion
    COMMAND test_point_cloud_conversion
  )

  add_executable(test_local_map_target_policy
    test/test_local_map_target_policy.cpp)
  target_include_directories(test_local_map_target_policy PRIVATE
    include
    ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_local_map_target_policy
    ${PCL_LIBRARIES})
  add_test(
    NAME local_map_target_policy
    COMMAND test_local_map_target_policy
  )

  add_executable(test_map_initialization_policy
    test/test_map_initialization_policy.cpp)
  target_include_directories(test_map_initialization_policy PRIVATE
    include
    ${PCL_INCLUDE_DIRS})
  ament_target_dependencies(test_map_initialization_policy
    pcl_conversions
    sensor_msgs)
  target_link_libraries(test_map_initialization_policy
    ${PCL_LIBRARIES})
  add_test(
    NAME map_initialization_policy
    COMMAND test_map_initialization_policy
  )

  add_executable(test_measurement_gate_policy
    test/test_measurement_gate_policy.cpp)
  target_include_directories(test_measurement_gate_policy PRIVATE
    include)
  add_test(
    NAME measurement_gate_policy
    COMMAND test_measurement_gate_policy
  )

  add_executable(test_localization_update_policy
    test/test_localization_update_policy.cpp)
  target_include_directories(test_localization_update_policy PRIVATE
    include)
  add_test(
    NAME localization_update_policy
    COMMAND test_localization_update_policy
  )

  add_executable(test_scan_preprocessing_policy
    test/test_scan_preprocessing_policy.cpp)
  target_include_directories(test_scan_preprocessing_policy PRIVATE
    include)
  add_test(
    NAME scan_preprocessing_policy
    COMMAND test_scan_preprocessing_policy
  )

  add_executable(test_recovery_supervisor
    test/test_recovery_supervisor.cpp)
  target_include_directories(test_recovery_supervisor PRIVATE
    include)
  add_test(
    NAME recovery_supervisor
    COMMAND test_recovery_supervisor
  )
  add_test(
    NAME mid360_bringup_model
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mid360_bringup_model.py
  )
  add_test(
    NAME registration_relocalization_job_ordering
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_registration_relocalization_job_ordering.py
  )
endif()

ament_package()
