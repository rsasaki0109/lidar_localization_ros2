import os

import launch
import launch.actions
import launch.events
import launch_ros
import launch_ros.events

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import lifecycle_msgs.msg


def generate_launch_description():
    """Launch preset for Jetson + Livox MID-360 on legged robots."""

    default_param = os.path.join(
        get_package_share_directory('lidar_localization_ros2'),
        'param',
        'mid360_legged.yaml')

    localization_param_dir = LaunchConfiguration(
        'localization_param_dir', default=default_param)
    map_path = LaunchConfiguration('map_path', default='/map/map.pcd')
    registration_method = LaunchConfiguration('registration_method', default='NDT_OMP')
    ndt_num_threads = LaunchConfiguration('ndt_num_threads', default='4')

    global_frame_id = LaunchConfiguration('global_frame_id', default='map')
    odom_frame_id = LaunchConfiguration('odom_frame_id', default='odom')
    base_frame_id = LaunchConfiguration('base_frame_id', default='base_link')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_map_odom_tf = LaunchConfiguration('enable_map_odom_tf', default='true')

    cloud_topic = LaunchConfiguration('cloud_topic', default='/livox/points')
    twist_topic = LaunchConfiguration('twist_topic', default='/twist')
    imu_topic = LaunchConfiguration('imu_topic', default='/livox/imu')
    use_imu_preintegration = LaunchConfiguration(
        'use_imu_preintegration', default='true')
    imu_preintegration_use_base_frame_transform = LaunchConfiguration(
        'imu_preintegration_use_base_frame_transform', default='true')

    set_initial_pose = LaunchConfiguration('set_initial_pose', default='false')
    initial_pose_x = LaunchConfiguration('initial_pose_x', default='0.0')
    initial_pose_y = LaunchConfiguration('initial_pose_y', default='0.0')
    initial_pose_z = LaunchConfiguration('initial_pose_z', default='0.0')
    initial_pose_qx = LaunchConfiguration('initial_pose_qx', default='0.0')
    initial_pose_qy = LaunchConfiguration('initial_pose_qy', default='0.0')
    initial_pose_qz = LaunchConfiguration('initial_pose_qz', default='0.0')
    initial_pose_qw = LaunchConfiguration('initial_pose_qw', default='1.0')

    publish_lidar_tf = LaunchConfiguration('publish_lidar_tf', default='true')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='livox_frame')
    lidar_tf_x = LaunchConfiguration('lidar_tf_x', default='0.0')
    lidar_tf_y = LaunchConfiguration('lidar_tf_y', default='0.0')
    lidar_tf_z = LaunchConfiguration('lidar_tf_z', default='0.0')
    lidar_tf_roll = LaunchConfiguration('lidar_tf_roll', default='0.0')
    lidar_tf_pitch = LaunchConfiguration('lidar_tf_pitch', default='0.0')
    lidar_tf_yaw = LaunchConfiguration('lidar_tf_yaw', default='0.0')

    publish_imu_tf = LaunchConfiguration('publish_imu_tf', default='false')
    imu_frame_id = LaunchConfiguration('imu_frame_id', default='livox_imu_frame')
    imu_tf_x = LaunchConfiguration('imu_tf_x', default='0.0')
    imu_tf_y = LaunchConfiguration('imu_tf_y', default='0.0')
    imu_tf_z = LaunchConfiguration('imu_tf_z', default='0.0')
    imu_tf_roll = LaunchConfiguration('imu_tf_roll', default='0.0')
    imu_tf_pitch = LaunchConfiguration('imu_tf_pitch', default='0.0')
    imu_tf_yaw = LaunchConfiguration('imu_tf_yaw', default='0.0')

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

    lidar_localization = LifecycleNode(
        name='lidar_localization',
        namespace='',
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        parameters=[
            localization_param_dir,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'map_path': map_path,
                'registration_method': registration_method,
                'ndt_num_threads': ParameterValue(ndt_num_threads, value_type=int),
                'global_frame_id': global_frame_id,
                'odom_frame_id': odom_frame_id,
                'base_frame_id': base_frame_id,
                'enable_map_odom_tf': ParameterValue(enable_map_odom_tf, value_type=bool),
                'use_imu_preintegration': ParameterValue(
                    use_imu_preintegration, value_type=bool),
                'imu_preintegration_use_base_frame_transform': ParameterValue(
                    imu_preintegration_use_base_frame_transform, value_type=bool),
                'set_initial_pose': ParameterValue(set_initial_pose, value_type=bool),
                'initial_pose_x': ParameterValue(initial_pose_x, value_type=float),
                'initial_pose_y': ParameterValue(initial_pose_y, value_type=float),
                'initial_pose_z': ParameterValue(initial_pose_z, value_type=float),
                'initial_pose_qx': ParameterValue(initial_pose_qx, value_type=float),
                'initial_pose_qy': ParameterValue(initial_pose_qy, value_type=float),
                'initial_pose_qz': ParameterValue(initial_pose_qz, value_type=float),
                'initial_pose_qw': ParameterValue(initial_pose_qw, value_type=float),
            },
        ],
        remappings=[
            ('/cloud', cloud_topic),
            ('/twist', twist_topic),
            ('/imu', imu_topic),
            ('/pcl_pose', '/localization/pose_with_covariance'),
        ],
        output='screen')

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                launch.actions.EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            lidar_localization),
                        transition_id=(
                            lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE),
                    )),
            ],
        ))

    configure_localization = TimerAction(
        period=1.0,
        actions=[
            launch.actions.EmitEvent(
                event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(
                        lidar_localization),
                    transition_id=(
                        lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE),
                ))
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('localization_param_dir', default_value=default_param),
        DeclareLaunchArgument('map_path', default_value='/map/map.pcd'),
        DeclareLaunchArgument('registration_method', default_value='NDT_OMP'),
        DeclareLaunchArgument('ndt_num_threads', default_value='4'),
        DeclareLaunchArgument('global_frame_id', default_value='map'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='base_link'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_map_odom_tf', default_value='true'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/points'),
        DeclareLaunchArgument('twist_topic', default_value='/twist'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('use_imu_preintegration', default_value='true'),
        DeclareLaunchArgument(
            'imu_preintegration_use_base_frame_transform', default_value='true'),
        DeclareLaunchArgument('set_initial_pose', default_value='false'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_qx', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_qy', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_qz', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_qw', default_value='1.0'),
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
        from_inactive_to_active,
        lidar_localization,
        lidar_tf,
        imu_tf,
        configure_localization,
    ])
