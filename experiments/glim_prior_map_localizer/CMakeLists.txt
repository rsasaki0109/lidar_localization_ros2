cmake_minimum_required(VERSION 3.16)
project(glim_prior_map_localizer LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(robin REQUIRED)
find_package(kiss_matcher REQUIRED)
find_package(spdlog REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(glim_prior_map_localizer SHARED
  src/prior_map_localizer.cpp
  src/vgicp_refiner.cpp)
target_include_directories(glim_prior_map_localizer PRIVATE
  include
  ${GTSAM_INCLUDE_DIRS})
target_link_libraries(glim_prior_map_localizer PRIVATE
  glim::glim
  kiss_matcher::kiss_matcher_core
  robin::robin
  spdlog::spdlog
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs__rosidl_generator_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  std_msgs::std_msgs__rosidl_generator_cpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_generator_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  ${GTSAM_LIBRARIES})

add_executable(glim_prior_map_offline_match
  src/offline_match.cpp
  src/vgicp_refiner.cpp)
target_include_directories(glim_prior_map_offline_match PRIVATE
  include)
target_link_libraries(glim_prior_map_offline_match PRIVATE
  glim::glim
  kiss_matcher::kiss_matcher_core
  robin::robin)

if(BUILD_TESTING)
  add_executable(test_prior_map_overlap_policy
    test/test_prior_map_overlap_policy.cpp)
  target_include_directories(test_prior_map_overlap_policy PRIVATE include)
  target_link_libraries(test_prior_map_overlap_policy PRIVATE Eigen3::Eigen)
  add_test(NAME prior_map_overlap_policy COMMAND test_prior_map_overlap_policy)
endif()

install(TARGETS glim_prior_map_localizer glim_prior_map_offline_match
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
