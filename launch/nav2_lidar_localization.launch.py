import os

import launch
import launch.actions
import launch.events
import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    """

    ld = LaunchDescription()

    # Arguments
    localization_param_dir = LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(
            get_package_share_directory('lidar_localization_ros2'),
            'param', 'localization.yaml'))

    cloud_topic = LaunchConfiguration('cloud_topic', default='/velodyne_points')
    twist_topic = LaunchConfiguration('twist_topic', default='/twist')
    imu_topic = LaunchConfiguration('imu_topic', default='/imu/data')

    ld.add_action(DeclareLaunchArgument('localization_param_dir'))
    ld.add_action(DeclareLaunchArgument('cloud_topic', default_value='/velodyne_points'))
    ld.add_action(DeclareLaunchArgument('twist_topic', default_value='/twist'))
    ld.add_action(DeclareLaunchArgument('imu_topic', default_value='/imu/data'))

    # Static TF: base_link → velodyne (adjust per robot)
    lidar_tf = Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'velodyne'])

    # Static TF: base_link → imu_link (adjust per robot)
    imu_tf = Node(
        name='imu_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link'])

    # Localization lifecycle node
    lidar_localization = LifecycleNode(
        name='lidar_localization',
        namespace='',
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        parameters=[localization_param_dir,
                    {'enable_map_odom_tf': True}],  # Nav2: publish map→odom
        remappings=[
            ('/cloud', cloud_topic),
            ('/twist', twist_topic),
            ('/imu', imu_topic),
            ('/pcl_pose', '/localization/pose_with_covariance'),
        ],
        output='screen')

    # Auto lifecycle transitions: unconfigured → configure → activate
    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            goal_state='unconfigured',
            entities=[
                launch.actions.EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
            ],
        ))

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

    # Configure immediately after startup
    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ))

    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)
    ld.add_action(lidar_localization)
    ld.add_action(lidar_tf)
    ld.add_action(imu_tf)
    ld.add_action(to_inactive)

    return ld
