import os

import launch.actions
import launch.events

import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    localization_param_dir = LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(
            get_package_share_directory('lidar_localization_ros2'),
            'param',
            'localization.yaml'))
    cloud_topic = LaunchConfiguration(
        'cloud_topic',
        default='/velodyne_points')
    twist_topic = LaunchConfiguration(
        'twist_topic',
        default='/twist')
    imu_topic = LaunchConfiguration(
        'imu_topic',
        default='/imu')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_dataset_tf_tree = LaunchConfiguration('use_dataset_tf_tree', default='false')
    dataset_root_frame = LaunchConfiguration('dataset_root_frame', default='camera_base')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='velodyne')

    # Default robot: base_link → velodyne. Bagged datasets (e.g. Koide Zenodo 10122133): set
    # use_dataset_tf_tree:=true and dataset_root_frame:=camera_base so base_link attaches to the
    # bag’s /tf_static tree (depth_camera_link, imu_link, …).
    lidar_tf = Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', lidar_frame_id,
        ],
        condition=UnlessCondition(use_dataset_tf_tree))

    dataset_root_attach_tf = Node(
        name='dataset_root_attach',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', dataset_root_frame,
        ],
        condition=IfCondition(use_dataset_tf_tree))

    # Extrinsics from Koide indoor_* bags (Zenodo 10122133), duplicated here because /tf_static
    # from ros2 bag play often fails QoS matching with transform listeners.
    koide_camera_base_to_depth_tf = Node(
        name='koide_camera_base_to_depth',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', 'camera_base',
            '--child-frame-id', 'depth_camera_link',
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0017999999690800905',
            '--qx', '0.5254827454987588',
            '--qy', '-0.5254827454987588',
            '--qz', '0.473146789255815',
            '--qw', '-0.4731467892558148',
        ],
        condition=IfCondition(use_dataset_tf_tree))

    koide_depth_to_imu_tf = Node(
        name='koide_depth_to_imu',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', 'depth_camera_link',
            '--child-frame-id', 'imu_link',
            '--x', '0.003463566434548747',
            '--y', '0.0041740033449125195',
            '--z', '-0.05071645628165228',
            '--qx', '-0.47551892422054987',
            '--qy', '0.4736570557366866',
            '--qz', '0.5236230430890362',
            '--qw', '0.5247376978138559',
        ],
        condition=IfCondition(use_dataset_tf_tree))

    lidar_localization = LifecycleNode(
        name='lidar_localization',
        namespace='',
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        parameters=[
            localization_param_dir,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('/cloud', cloud_topic), ('/twist', twist_topic), ('/imu', imu_topic)],
        output='screen')

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_dataset_tf_tree', default_value='false'),
        DeclareLaunchArgument('dataset_root_frame', default_value='camera_base'),
        DeclareLaunchArgument('lidar_frame_id', default_value='velodyne'),
        from_unconfigured_to_inactive,
        from_inactive_to_active,
        lidar_localization,
        lidar_tf,
        dataset_root_attach_tf,
        koide_camera_base_to_depth_tf,
        koide_depth_to_imu_tf,
        to_inactive,
    ])
