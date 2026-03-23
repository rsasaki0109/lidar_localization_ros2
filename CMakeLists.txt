cmake_minimum_required(VERSION 3.5)
project(lidar_localization_ros2)

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
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
  ndt_omp_ros2
)

if(small_gicp_FOUND)
  target_compile_definitions(lidar_localization_component PRIVATE LIDAR_LOCALIZATION_HAVE_SMALL_GICP=1)
  target_link_libraries(lidar_localization_component small_gicp::small_gicp)
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
  scripts/benchmark_extract_pose_reference_from_rosbag2
  scripts/benchmark_make_graph_dataset
  scripts/benchmark_pose_recorder
  scripts/benchmark_diagnostic_recorder
  scripts/benchmark_runner
  scripts/benchmark_sweep_localizer
  scripts/benchmark_eval_trajectory
  scripts/fetch_official_autoware_istanbul_dataset.sh
  scripts/fetch_official_hdl_localization_sample.sh
  scripts/make_localization_comparison_report.py
  scripts/record_hdl_localization_outputs.py
  scripts/run_hdl_localization_docker_benchmark.sh
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  docs
  param
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
