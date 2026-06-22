import os

import launch.actions
import launch.events

import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_localization_param_dir = os.path.join(
        get_package_share_directory('lidar_localization_ros2'),
        'param',
        'localization.yaml')
    localization_param_dir = LaunchConfiguration(
        'localization_param_dir',
        default=default_localization_param_dir)
    cloud_topic = LaunchConfiguration(
        'cloud_topic',
        default='/velodyne_points')
    twist_topic = LaunchConfiguration(
        'twist_topic',
        default='/twist')
    imu_topic = LaunchConfiguration(
        'imu_topic',
        default='/imu')
    global_frame_id = LaunchConfiguration('global_frame_id', default='map')
    odom_frame_id = LaunchConfiguration('odom_frame_id', default='odom')
    base_frame_id = LaunchConfiguration('base_frame_id', default='base_link')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_dataset_tf_tree = LaunchConfiguration('use_dataset_tf_tree', default='false')
    dataset_root_frame = LaunchConfiguration('dataset_root_frame', default='camera_base')
    publish_lidar_tf = LaunchConfiguration('publish_lidar_tf', default='true')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='velodyne')
    lidar_tf_x = LaunchConfiguration('lidar_tf_x', default='0.0')
    lidar_tf_y = LaunchConfiguration('lidar_tf_y', default='0.0')
    lidar_tf_z = LaunchConfiguration('lidar_tf_z', default='0.0')
    lidar_tf_roll = LaunchConfiguration('lidar_tf_roll', default='0.0')
    lidar_tf_pitch = LaunchConfiguration('lidar_tf_pitch', default='0.0')
    lidar_tf_yaw = LaunchConfiguration('lidar_tf_yaw', default='0.0')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf', default='false')
    imu_frame_id = LaunchConfiguration('imu_frame_id', default='imu_link')
    use_imu_preintegration = LaunchConfiguration(
        'use_imu_preintegration', default='true')
    imu_preintegration_use_base_frame_transform = LaunchConfiguration(
        'imu_preintegration_use_base_frame_transform', default='false')
    use_continuous_time_deskew = LaunchConfiguration(
        'use_continuous_time_deskew', default='false')
    continuous_time_deskew_reference_time_sec = LaunchConfiguration(
        'continuous_time_deskew_reference_time_sec', default='0.0')
    imu_tf_x = LaunchConfiguration('imu_tf_x', default='0.0')
    imu_tf_y = LaunchConfiguration('imu_tf_y', default='0.0')
    imu_tf_z = LaunchConfiguration('imu_tf_z', default='0.0')
    imu_tf_roll = LaunchConfiguration('imu_tf_roll', default='0.0')
    imu_tf_pitch = LaunchConfiguration('imu_tf_pitch', default='0.0')
    imu_tf_yaw = LaunchConfiguration('imu_tf_yaw', default='0.0')

    # Default robot: base frame -> lidar. Bagged datasets (e.g. Koide Zenodo 10122133): set
    # use_dataset_tf_tree:=true and dataset_root_frame:=camera_base so the base frame attaches to
    # the bag's /tf_static tree (depth_camera_link, imu_link, ...).
    lidar_tf = Node(
        name='lidar_tf',
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
        condition=IfCondition(PythonExpression([
            "'", publish_lidar_tf, "' == 'true' and '", use_dataset_tf_tree, "' != 'true'"
        ])))

    imu_tf = Node(
        name='imu_tf',
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

    dataset_root_attach_tf = Node(
        name='dataset_root_attach',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', base_frame_id,
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
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'global_frame_id': global_frame_id,
                'odom_frame_id': odom_frame_id,
                'base_frame_id': base_frame_id,
                'use_imu_preintegration': ParameterValue(
                    use_imu_preintegration, value_type=bool),
                'imu_preintegration_use_base_frame_transform': ParameterValue(
                    imu_preintegration_use_base_frame_transform, value_type=bool),
                'use_continuous_time_deskew': ParameterValue(
                    use_continuous_time_deskew, value_type=bool),
                'continuous_time_deskew_reference_time_sec': ParameterValue(
                    continuous_time_deskew_reference_time_sec, value_type=float),
            },
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
        DeclareLaunchArgument(
            'localization_param_dir',
            default_value=default_localization_param_dir,
            description='Path to the lidar_localization_ros2 parameter YAML.'),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/velodyne_points',
            description='Input sensor_msgs/PointCloud2 topic remapped to /cloud.'),
        DeclareLaunchArgument(
            'twist_topic',
            default_value='/twist',
            description='Optional twist topic remapped to /twist.'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu',
            description='Optional IMU topic remapped to /imu.'),
        DeclareLaunchArgument('global_frame_id', default_value='map'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='base_link'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_imu_preintegration', default_value='true'),
        DeclareLaunchArgument(
            'imu_preintegration_use_base_frame_transform',
            default_value='false'),
        DeclareLaunchArgument('use_continuous_time_deskew', default_value='false'),
        DeclareLaunchArgument(
            'continuous_time_deskew_reference_time_sec',
            default_value='0.0'),
        DeclareLaunchArgument(
            'publish_lidar_tf',
            default_value='true',
            description='Publish a static base_frame_id -> lidar_frame_id transform.'),
        DeclareLaunchArgument('lidar_frame_id', default_value='velodyne'),
        DeclareLaunchArgument('lidar_tf_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_z', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'publish_imu_tf',
            default_value='false',
            description='Publish a static base_frame_id -> imu_frame_id transform.'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),
        DeclareLaunchArgument('imu_tf_x', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_y', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_z', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'use_dataset_tf_tree',
            default_value='false',
            description='Attach base_frame_id to a dataset-provided TF tree instead of lidar TF.'),
        DeclareLaunchArgument('dataset_root_frame', default_value='camera_base'),
        from_unconfigured_to_inactive,
        from_inactive_to_active,
        lidar_localization,
        lidar_tf,
        imu_tf,
        dataset_root_attach_tf,
        koide_camera_base_to_depth_tf,
        koide_depth_to_imu_tf,
        to_inactive,
    ])
