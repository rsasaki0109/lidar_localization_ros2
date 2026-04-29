cmake_minimum_required(VERSION 3.5)
project(lidar_localization_ros2 VERSION 1.0.0)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

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
link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

ament_export_libraries(lidar_localization_component)

install(TARGETS
  lidar_localization_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  lidar_localization_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(PROGRAMS
  scripts/benchmark_analyze_drift_window
  scripts/benchmark_compare_runs
  scripts/compare_nav2_reinit_supervisor_runs.py
  scripts/convert_boreas_sequence_to_rosbag2.py
  scripts/benchmark_extract_pose_reference_from_rosbag2
  scripts/benchmark_from_manifest
  scripts/benchmark_make_graph_dataset
  scripts/benchmark_pose_recorder
  scripts/benchmark_diagnostic_recorder
  scripts/benchmark_runner
  scripts/benchmark_sweep_localizer
  scripts/benchmark_eval_trajectory
  scripts/augment_pointcloud_intensity.py
  scripts/scaffold_mapless_public_dataset_bundle.py
  scripts/fetch_official_autoware_istanbul_dataset.sh
  scripts/fetch_official_hdl_localization_sample.sh
  scripts/generate_occupancy_map_from_pcd.py
  scripts/make_localization_comparison_report.py
  scripts/publish_cmd_vel_odom.py
  scripts/publish_identity_odom.py
  scripts/publish_odom_from_localization.py
  scripts/publish_odom_from_twist.py
  scripts/publish_pose_from_odom.py
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
  scripts/run_autoware_istanbul_60s_predictor_compare.sh
  scripts/run_hdl_localization_docker_benchmark.sh
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  docs
  experiments
  param
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
