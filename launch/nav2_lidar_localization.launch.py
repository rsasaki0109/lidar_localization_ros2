import os

import launch
import launch.actions
import launch.events
import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch lidar_localization configured for Nav2 integration.

    Key differences from standard launch:
    - enable_map_odom_tf: true (publishes map→odom TF for Nav2)
    - Requires an odometry source publishing odom→base_link TF
    - Covariance populated from fitness score
    - Defaults to nav2_ndt_urban.yaml, which expects a twist topic and /initialpose
    """

    ld = LaunchDescription()

    # Arguments
    default_localization_param_dir = os.path.join(
        get_package_share_directory('lidar_localization_ros2'),
        'param', 'nav2_ndt_urban.yaml')
    localization_param_dir = LaunchConfiguration(
        'localization_param_dir',
        default=default_localization_param_dir)

    global_frame_id = LaunchConfiguration('global_frame_id', default='map')
    odom_frame_id = LaunchConfiguration('odom_frame_id', default='odom')
    base_frame_id = LaunchConfiguration('base_frame_id', default='base_link')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_map_odom_tf = LaunchConfiguration('enable_map_odom_tf', default='true')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/velodyne_points')
    twist_topic = LaunchConfiguration('twist_topic', default='/twist')
    imu_topic = LaunchConfiguration('imu_topic', default='/imu/data')
    publish_lidar_tf = LaunchConfiguration('publish_lidar_tf', default='true')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='velodyne')
    lidar_tf_x = LaunchConfiguration('lidar_tf_x', default='0.0')
    lidar_tf_y = LaunchConfiguration('lidar_tf_y', default='0.0')
    lidar_tf_z = LaunchConfiguration('lidar_tf_z', default='0.0')
    lidar_tf_roll = LaunchConfiguration('lidar_tf_roll', default='0.0')
    lidar_tf_pitch = LaunchConfiguration('lidar_tf_pitch', default='0.0')
    lidar_tf_yaw = LaunchConfiguration('lidar_tf_yaw', default='0.0')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf', default='true')
    imu_frame_id = LaunchConfiguration('imu_frame_id', default='imu_link')
    imu_tf_x = LaunchConfiguration('imu_tf_x', default='0.0')
    imu_tf_y = LaunchConfiguration('imu_tf_y', default='0.0')
    imu_tf_z = LaunchConfiguration('imu_tf_z', default='0.0')
    imu_tf_roll = LaunchConfiguration('imu_tf_roll', default='0.0')
    imu_tf_pitch = LaunchConfiguration('imu_tf_pitch', default='0.0')
    imu_tf_yaw = LaunchConfiguration('imu_tf_yaw', default='0.0')

    ld.add_action(DeclareLaunchArgument('localization_param_dir', default_value=default_localization_param_dir))
    ld.add_action(DeclareLaunchArgument('global_frame_id', default_value='map'))
    ld.add_action(DeclareLaunchArgument('odom_frame_id', default_value='odom'))
    ld.add_action(DeclareLaunchArgument('base_frame_id', default_value='base_link'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('enable_map_odom_tf', default_value='true'))
    ld.add_action(DeclareLaunchArgument('cloud_topic', default_value='/velodyne_points'))
    ld.add_action(DeclareLaunchArgument('twist_topic', default_value='/twist'))
    ld.add_action(DeclareLaunchArgument('imu_topic', default_value='/imu/data'))
    ld.add_action(DeclareLaunchArgument('publish_lidar_tf', default_value='true'))
    ld.add_action(DeclareLaunchArgument('lidar_frame_id', default_value='velodyne'))
    ld.add_action(DeclareLaunchArgument('lidar_tf_x', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('lidar_tf_y', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('lidar_tf_z', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('lidar_tf_roll', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('lidar_tf_pitch', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('lidar_tf_yaw', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('publish_imu_tf', default_value='true'))
    ld.add_action(DeclareLaunchArgument('imu_frame_id', default_value='imu_link'))
    ld.add_action(DeclareLaunchArgument('imu_tf_x', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('imu_tf_y', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('imu_tf_z', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('imu_tf_roll', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('imu_tf_pitch', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('imu_tf_yaw', default_value='0.0'))

    # Static TF: base_link → velodyne (adjust per robot)
    lidar_tf = Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', lidar_tf_x, '--y', lidar_tf_y, '--z', lidar_tf_z,
                   '--roll', lidar_tf_roll, '--pitch', lidar_tf_pitch, '--yaw', lidar_tf_yaw,
                   '--frame-id', base_frame_id, '--child-frame-id', lidar_frame_id],
        condition=IfCondition(publish_lidar_tf))

    # Static TF: base_link → imu_link (adjust per robot)
    imu_tf = Node(
        name='imu_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', imu_tf_x, '--y', imu_tf_y, '--z', imu_tf_z,
                   '--roll', imu_tf_roll, '--pitch', imu_tf_pitch, '--yaw', imu_tf_yaw,
                   '--frame-id', base_frame_id, '--child-frame-id', imu_frame_id],
        condition=IfCondition(publish_imu_tf))

    # Localization lifecycle node
    lidar_localization = LifecycleNode(
        name='lidar_localization',
        namespace='',
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        parameters=[localization_param_dir,
                    {
                        'use_sim_time': use_sim_time,
                        'enable_map_odom_tf': enable_map_odom_tf,
                        'global_frame_id': global_frame_id,
                        'odom_frame_id': odom_frame_id,
                        'base_frame_id': base_frame_id,
                    }],  # Nav2: publish map→odom
        remappings=[
            ('/cloud', cloud_topic),
            ('/twist', twist_topic),
            ('/imu', imu_topic),
            ('/pcl_pose', '/localization/pose_with_covariance'),
        ],
        output='screen')

    # Auto lifecycle transitions: unconfigured → configure → activate
    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                launch.actions.EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
            ],
        ))

    # Defer the configure request slightly so the lifecycle services are ready
    configure_localization = TimerAction(
        period=1.0,
        actions=[
            launch.actions.EmitEvent(
                event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                ))
        ],
    )

    ld.add_action(from_inactive_to_active)
    ld.add_action(lidar_localization)
    ld.add_action(lidar_tf)
    ld.add_action(imu_tf)
    ld.add_action(configure_localization)

    return ld
