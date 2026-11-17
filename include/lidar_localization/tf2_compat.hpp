#ifndef LIDAR_LOCALIZATION_TF2_COMPAT_HPP_
#define LIDAR_LOCALIZATION_TF2_COMPAT_HPP_

// ROS 2 Foxy ships the tf2 conversion headers as .h only; Galactic and later
// add the .hpp names (and deprecate the .h ones). Each include below takes the
// .hpp header where it exists, so Humble and Jazzy build exactly as before.

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#endif  // LIDAR_LOCALIZATION_TF2_COMPAT_HPP_
