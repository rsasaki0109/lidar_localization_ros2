"""User-facing localization bringup with guarded startup initialization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _localization_include(context):
    package_share = get_package_share_directory("lidar_localization_ros2")
    profile = LaunchConfiguration("profile").perform(context)
    launch_by_profile = {
        "standalone": "lidar_localization.launch.py",
        "nav2": "nav2_lidar_localization.launch.py",
        "mid360": "mid360_legged_localization.launch.py",
    }
    if profile not in launch_by_profile:
        raise ValueError(f"unsupported quickstart profile: {profile}")
    arguments = {
        "localization_param_dir": LaunchConfiguration("localization_param_dir"),
        "cloud_topic": LaunchConfiguration("cloud_topic"),
        "imu_topic": LaunchConfiguration("imu_topic"),
        "global_frame_id": LaunchConfiguration("global_frame_id"),
        "odom_frame_id": LaunchConfiguration("odom_frame_id"),
        "base_frame_id": LaunchConfiguration("base_frame_id"),
        "lidar_frame_id": LaunchConfiguration("lidar_frame_id"),
        "imu_frame_id": LaunchConfiguration("imu_frame_id"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "publish_lidar_tf": LaunchConfiguration("publish_lidar_tf"),
        "publish_imu_tf": LaunchConfiguration("publish_imu_tf"),
    }
    if profile == "mid360":
        arguments["map_path"] = LaunchConfiguration("map_path")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", launch_by_profile[profile])),
        launch_arguments=arguments.items(),
    )]


def _bringup_check(context):
    profile = LaunchConfiguration("profile").perform(context)
    arguments = [
        "--profile", profile,
        "--duration-sec", "5.0",
        "--cloud-topic", LaunchConfiguration("cloud_topic"),
        "--imu-topic", LaunchConfiguration("imu_topic"),
        "--lidar-frame", LaunchConfiguration("lidar_frame_id"),
        "--imu-frame", LaunchConfiguration("imu_frame_id"),
        "--global-frame", LaunchConfiguration("global_frame_id"),
        "--odom-frame", LaunchConfiguration("odom_frame_id"),
        "--base-frame", LaunchConfiguration("base_frame_id"),
    ]
    if profile in {"nav2", "mid360"}:
        arguments.append("--require-odom-base-tf")
    if profile == "nav2":
        arguments.append("--require-map-odom-tf")
    if profile == "mid360":
        arguments.extend(["--require-imu", "--require-imu-base-tf"])
    return [TimerAction(
        period=3.0,
        actions=[Node(
            package="lidar_localization_ros2",
            executable="check_lidar_localization_bringup.py",
            name="quickstart_bringup_check",
            arguments=arguments,
            condition=IfCondition(LaunchConfiguration("run_bringup_check")),
            output="screen",
        )],
    )]


def generate_launch_description():
    package_share = get_package_share_directory("lidar_localization_ros2")
    declarations = [
        DeclareLaunchArgument("profile", default_value="standalone"),
        DeclareLaunchArgument("localization_param_dir"),
        DeclareLaunchArgument("map_path"),
        DeclareLaunchArgument("occupancy_yaml", default_value=""),
        DeclareLaunchArgument("pose_state_path"),
        DeclareLaunchArgument("cloud_topic", default_value="/velodyne_points"),
        DeclareLaunchArgument("imu_topic", default_value="/imu"),
        DeclareLaunchArgument("pose_topic", default_value="/pcl_pose"),
        DeclareLaunchArgument("global_frame_id", default_value="map"),
        DeclareLaunchArgument("odom_frame_id", default_value="odom"),
        DeclareLaunchArgument("base_frame_id", default_value="base_link"),
        DeclareLaunchArgument("lidar_frame_id", default_value="velodyne"),
        DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("publish_lidar_tf", default_value="true"),
        DeclareLaunchArgument("publish_imu_tf", default_value="false"),
        DeclareLaunchArgument("restore_saved_pose", default_value="true"),
        DeclareLaunchArgument("initial_pose_preconfigured", default_value="false"),
        DeclareLaunchArgument("enable_global_initialization", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("run_bringup_check", default_value="true"),
        DeclareLaunchArgument("saved_pose_max_age_sec", default_value="0.0"),
        DeclareLaunchArgument("min_candidate_score", default_value="0.6"),
        DeclareLaunchArgument("min_score_margin", default_value="0.05"),
        DeclareLaunchArgument("max_candidate_age_sec", default_value="30.0"),
        DeclareLaunchArgument("global_query_timeout_sec", default_value="30.0"),
        DeclareLaunchArgument("verification_samples", default_value="3"),
        DeclareLaunchArgument("verification_fitness_threshold", default_value="1.5"),
        DeclareLaunchArgument("max_global_attempts", default_value="6"),
        DeclareLaunchArgument("global_consensus_samples", default_value="2"),
        DeclareLaunchArgument("global_consensus_translation_m", default_value="2.0"),
        DeclareLaunchArgument("global_consensus_yaw_deg", default_value="20.0"),
        DeclareLaunchArgument("g2_use_cpp_backend", default_value="true"),
        DeclareLaunchArgument("g2_enable_registration_scoring", default_value="true"),
        DeclareLaunchArgument("require_global_registration_scoring", default_value="true"),
        DeclareLaunchArgument("g2_registration_score_gate", default_value="6.0"),
        DeclareLaunchArgument("g2_registration_refine_candidates", default_value="false"),
        DeclareLaunchArgument("g2_registration_seed_z_m", default_value="0.0"),
        DeclareLaunchArgument("g2_max_scan_points", default_value="256"),
        DeclareLaunchArgument("g2_angular_resolution_deg", default_value="10.0"),
        DeclareLaunchArgument("g2_max_candidates", default_value="8"),
        DeclareLaunchArgument("g2_nms_radius_m", default_value="3.0"),
    ]

    global_localization = Node(
        package="lidar_localization_ros2",
        executable="global_localization_node.py",
        name="global_localization_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_global_initialization")),
        parameters=[{
            "occupancy_yaml": LaunchConfiguration("occupancy_yaml"),
            "map_path": LaunchConfiguration("map_path"),
            "cloud_topic": LaunchConfiguration("cloud_topic"),
            "global_frame_id": LaunchConfiguration("global_frame_id"),
            "use_cpp_backend": ParameterValue(
                LaunchConfiguration("g2_use_cpp_backend"), value_type=bool),
            "enable_registration_scoring": ParameterValue(
                LaunchConfiguration("g2_enable_registration_scoring"), value_type=bool),
            "registration_score_gate": ParameterValue(
                LaunchConfiguration("g2_registration_score_gate"), value_type=float),
            "registration_refine_candidates": ParameterValue(
                LaunchConfiguration("g2_registration_refine_candidates"), value_type=bool),
            "registration_seed_z_m": ParameterValue(
                LaunchConfiguration("g2_registration_seed_z_m"), value_type=float),
            "seed_z_m": ParameterValue(
                LaunchConfiguration("g2_registration_seed_z_m"), value_type=float),
            "max_scan_points": ParameterValue(
                LaunchConfiguration("g2_max_scan_points"), value_type=int),
            "angular_resolution_deg": ParameterValue(
                LaunchConfiguration("g2_angular_resolution_deg"), value_type=float),
            "max_candidates": ParameterValue(
                LaunchConfiguration("g2_max_candidates"), value_type=int),
            "nms_radius_m": ParameterValue(
                LaunchConfiguration("g2_nms_radius_m"), value_type=float),
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
    )
    startup_initialization = Node(
        package="lidar_localization_ros2",
        executable="startup_initialization_node.py",
        name="startup_initialization",
        output="screen",
        parameters=[{
            "map_path": LaunchConfiguration("map_path"),
            "pose_state_path": LaunchConfiguration("pose_state_path"),
            "restore_saved_pose": ParameterValue(
                LaunchConfiguration("restore_saved_pose"), value_type=bool),
            "initial_pose_preconfigured": ParameterValue(
                LaunchConfiguration("initial_pose_preconfigured"), value_type=bool),
            "enable_global_initialization": ParameterValue(
                LaunchConfiguration("enable_global_initialization"), value_type=bool),
            "require_global_registration_scoring": ParameterValue(
                LaunchConfiguration("require_global_registration_scoring"),
                value_type=bool),
            "cloud_topic": LaunchConfiguration("cloud_topic"),
            "pose_topic": LaunchConfiguration("pose_topic"),
            "global_frame_id": LaunchConfiguration("global_frame_id"),
            "saved_pose_max_age_sec": ParameterValue(
                LaunchConfiguration("saved_pose_max_age_sec"), value_type=float),
            "min_candidate_score": ParameterValue(
                LaunchConfiguration("min_candidate_score"), value_type=float),
            "min_score_margin": ParameterValue(
                LaunchConfiguration("min_score_margin"), value_type=float),
            "max_candidate_age_sec": ParameterValue(
                LaunchConfiguration("max_candidate_age_sec"), value_type=float),
            "query_timeout_sec": ParameterValue(
                LaunchConfiguration("global_query_timeout_sec"), value_type=float),
            "verification_samples": ParameterValue(
                LaunchConfiguration("verification_samples"), value_type=int),
            "verification_fitness_threshold": ParameterValue(
                LaunchConfiguration("verification_fitness_threshold"), value_type=float),
            "max_global_attempts": ParameterValue(
                LaunchConfiguration("max_global_attempts"), value_type=int),
            "global_consensus_samples": ParameterValue(
                LaunchConfiguration("global_consensus_samples"), value_type=int),
            "global_consensus_translation_m": ParameterValue(
                LaunchConfiguration("global_consensus_translation_m"), value_type=float),
            "global_consensus_yaw_deg": ParameterValue(
                LaunchConfiguration("global_consensus_yaw_deg"), value_type=float),
            "default_z_m": ParameterValue(
                LaunchConfiguration("g2_registration_seed_z_m"), value_type=float),
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="quickstart_rviz",
        arguments=["-d", os.path.join(package_share, "rviz", "localization.rviz")],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        output="screen",
    )

    return LaunchDescription(
        declarations
        + [OpaqueFunction(function=_localization_include),
           OpaqueFunction(function=_bringup_check), global_localization,
           startup_initialization, rviz]
    )
