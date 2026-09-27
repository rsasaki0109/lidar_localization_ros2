import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from launch_param_overrides import resolve_parameter_overrides  # noqa: E402

# Node parameters that a launch argument overrides only when it is set.
# Empty arguments keep the parameter YAML value, or the fallback below.
PARAMETER_OVERRIDES = {
    'use_sim_time': (bool, False),
    'map_path': (str, '/map/map.pcd'),
    'registration_method': (str, 'NDT_OMP'),
    'ndt_num_threads': (int, 4),
    'global_frame_id': (str, 'map'),
    'odom_frame_id': (str, 'odom'),
    'base_frame_id': (str, 'base_link'),
    'enable_map_odom_tf': (bool, True),
    'use_imu_preintegration': (bool, True),
    'imu_preintegration_use_base_frame_transform': (bool, True),
    'use_continuous_time_deskew': (bool, True),
    'continuous_time_deskew_reference_time_sec': (float, 0.0),
    'set_initial_pose': (bool, False),
    'initial_pose_x': (float, 0.0),
    'initial_pose_y': (float, 0.0),
    'initial_pose_z': (float, 0.0),
    'initial_pose_qx': (float, 0.0),
    'initial_pose_qy': (float, 0.0),
    'initial_pose_qz': (float, 0.0),
    'initial_pose_qw': (float, 1.0),
}


def generate_launch_description():
    """Launch preset for Jetson + Livox MID-360 on legged robots."""

    default_param = os.path.join(
        get_package_share_directory('lidar_localization_ros2'),
        'param',
        'mid360_legged.yaml')

    localization_param_dir = LaunchConfiguration('localization_param_dir')

    base_frame_id = LaunchConfiguration('resolved_base_frame_id')

    cloud_topic = LaunchConfiguration('cloud_topic')
    twist_topic = LaunchConfiguration('twist_topic')
    imu_topic = LaunchConfiguration('imu_topic')


    publish_lidar_tf = LaunchConfiguration('publish_lidar_tf')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    lidar_tf_x = LaunchConfiguration('lidar_tf_x')
    lidar_tf_y = LaunchConfiguration('lidar_tf_y')
    lidar_tf_z = LaunchConfiguration('lidar_tf_z')
    lidar_tf_roll = LaunchConfiguration('lidar_tf_roll')
    lidar_tf_pitch = LaunchConfiguration('lidar_tf_pitch')
    lidar_tf_yaw = LaunchConfiguration('lidar_tf_yaw')

    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    imu_tf_x = LaunchConfiguration('imu_tf_x')
    imu_tf_y = LaunchConfiguration('imu_tf_y')
    imu_tf_z = LaunchConfiguration('imu_tf_z')
    imu_tf_roll = LaunchConfiguration('imu_tf_roll')
    imu_tf_pitch = LaunchConfiguration('imu_tf_pitch')
    imu_tf_yaw = LaunchConfiguration('imu_tf_yaw')

    lidar_tf = Node(
        name='mid360_lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', lidar_tf_x,
            '--y', lidar_tf_y,
            '--z', lidar_tf_z,
            '--roll', lidar_tf_roll,
            '--pitch', lidar_tf_pitch,
            '--yaw', lidar_tf_yaw,
            '--frame-id', base_frame_id,
            '--child-frame-id', lidar_frame_id,
        ],
        condition=IfCondition(publish_lidar_tf))

    imu_tf = Node(
        name='mid360_imu_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', imu_tf_x,
            '--y', imu_tf_y,
            '--z', imu_tf_z,
            '--roll', imu_tf_roll,
            '--pitch', imu_tf_pitch,
            '--yaw', imu_tf_yaw,
            '--frame-id', base_frame_id,
            '--child-frame-id', imu_frame_id,
        ],
        condition=IfCondition(publish_imu_tf))

    lidar_localization = Node(
        name='lidar_localization',
        namespace='',
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        parameters=[
            localization_param_dir,
            LaunchConfiguration('lidar_localization_override_file'),
        ],
        remappings=[
            ('/cloud', cloud_topic),
            ('/twist', twist_topic),
            ('/imu', imu_topic),
            ('/pcl_pose', '/localization/pose_with_covariance'),
        ],
        output='screen')

    startup = Node(
        package='lidar_localization_ros2',
        executable='start_lifecycle_node.py',
        arguments=['lidar_localization'],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('localization_param_dir', default_value=default_param),
        DeclareLaunchArgument(
            'map_path', default_value='',
            description='Empty keeps the parameter YAML value (fallback: /map/map.pcd).'),
        DeclareLaunchArgument(
            'registration_method', default_value='',
            description='Empty keeps the parameter YAML value (fallback: NDT_OMP).'),
        DeclareLaunchArgument(
            'ndt_num_threads', default_value='',
            description='Empty keeps the parameter YAML value (fallback: 4).'),
        DeclareLaunchArgument(
            'global_frame_id', default_value='',
            description='Empty keeps the parameter YAML value (fallback: map).'),
        DeclareLaunchArgument(
            'odom_frame_id', default_value='',
            description='Empty keeps the parameter YAML value (fallback: odom).'),
        DeclareLaunchArgument(
            'base_frame_id', default_value='',
            description='Empty keeps the parameter YAML value (fallback: base_link).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='',
            description='Empty keeps the parameter YAML value (fallback: false).'),
        DeclareLaunchArgument(
            'enable_map_odom_tf', default_value='',
            description='Empty keeps the parameter YAML value (fallback: true).'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/points'),
        DeclareLaunchArgument('twist_topic', default_value='/twist'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument(
            'use_imu_preintegration', default_value='',
            description='Empty keeps the parameter YAML value (fallback: true).'),
        DeclareLaunchArgument(
            'imu_preintegration_use_base_frame_transform', default_value='',
            description='Empty keeps the parameter YAML value (fallback: true).'),
        DeclareLaunchArgument(
            'use_continuous_time_deskew', default_value='',
            description='Deskew scans when point timing and motion data are ready; '
                        'otherwise keep the input scan unchanged.'
                        ' Empty keeps the parameter YAML value (fallback: true).'),
        DeclareLaunchArgument(
            'continuous_time_deskew_reference_time_sec', default_value='',
            description='Empty keeps the parameter YAML value (fallback: 0.0).'),
        DeclareLaunchArgument(
            'set_initial_pose', default_value='',
            description='Empty keeps the parameter YAML value (fallback: false).'),
        DeclareLaunchArgument(
            'initial_pose_x', default_value='',
            description='Empty keeps the parameter YAML value (fallback: 0.0).'),
        DeclareLaunchArgument(
            'initial_pose_y', default_value='',
            description='Empty keeps the parameter YAML value (fallback: 0.0).'),
        DeclareLaunchArgument(
            'initial_pose_z', default_value='',
            description='Empty keeps the parameter YAML value (fallback: 0.0).'),
        DeclareLaunchArgument(
            'initial_pose_qx', default_value='',
            description='Empty keeps the parameter YAML value (fallback: 0.0).'),
        DeclareLaunchArgument(
            'initial_pose_qy', default_value='',
            description='Empty keeps the parameter YAML value (fallback: 0.0).'),
        DeclareLaunchArgument(
            'initial_pose_qz', default_value='',
            description='Empty keeps the parameter YAML value (fallback: 0.0).'),
        DeclareLaunchArgument(
            'initial_pose_qw', default_value='',
            description='Empty keeps the parameter YAML value (fallback: 1.0).'),
        DeclareLaunchArgument('publish_lidar_tf', default_value='true'),
        DeclareLaunchArgument('lidar_frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('lidar_tf_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_z', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_yaw', default_value='0.0'),
        DeclareLaunchArgument('publish_imu_tf', default_value='false'),
        DeclareLaunchArgument('imu_frame_id', default_value='livox_imu_frame'),
        DeclareLaunchArgument('imu_tf_x', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_y', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_z', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_yaw', default_value='0.0'),
        resolve_parameter_overrides(
            'localization_param_dir', 'lidar_localization', PARAMETER_OVERRIDES),
        lidar_localization,
        lidar_tf,
        imu_tf,
        startup,
    ])
